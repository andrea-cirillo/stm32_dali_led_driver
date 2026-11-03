/**
 ******************************************************************************
 * @file    led_monitor.c
 * @brief   LED output monitoring and fault handling.
 ******************************************************************************
 */
#include "led_monitor.h"
#include "app_config.h"
#include "adc_drv.h"
#include "bsp.h"
#include "light_ctrl.h"
#include "opdata.h"
#include "variant.h"

#define OVERSAMPLING           (4U)
#define VDDA_PERIOD_MS         (100U)
#define TEMP_PERIOD_MS         (1000U)

static const uint16_t k_nominalMa[CFG_VARIANT_COUNT] = CFG_VARIANT_NOMINAL_CURRENT_MA;

static volatile uint8_t  s_failure = 0U;
static volatile uint32_t s_vledMv = 0U;
static volatile uint32_t s_currentMa = 0U;
static volatile uint32_t s_powerMw = 0U;
static volatile int16_t  s_temperature = 25;

static uint32_t s_vddaMv = 3300U;
static uint32_t s_lastSampleMs = 0U;
static uint32_t s_lastVddaMs = 0U;
static uint32_t s_lastTempMs = 0U;
static uint8_t  s_candidate = 0U;
static uint8_t  s_debounce = 0U;

static uint8_t classify(uint32_t vMv)
{
  uint8_t f = 0U;

  if (vMv < CFG_FAULT_SHORT_CIRCUIT_MV)
  {
    f = MON_FAIL_SHORT_CIRCUIT;
  }
  else if (vMv > CFG_FAULT_OPEN_CIRCUIT_MV)
  {
    f = MON_FAIL_OPEN_CIRCUIT;
  }
  else if (vMv > CFG_FAULT_OVER_VOLTAGE_MV)
  {
    f = MON_FAIL_LOAD_INCREASE;
  }
  else if (vMv < CFG_FAULT_UNDER_VOLTAGE_MV)
  {
    f = MON_FAIL_LOAD_DECREASE;
  }
  else
  {
    f = 0U;
  }
  return f;
}

static void update_estimates(void)
{
  const uint32_t rel = Light_GetRelativeCurrentQ16();
  const uint32_t iMa = ((uint32_t)k_nominalMa[Variant_GetActive()] * rel) >> 16U;
  const bool on = Light_IsLampOn();
  /* V [mV] * I [mA] / 1000 = P [mW]; max 400 V * 2 A fits in 32 bit */
  const uint32_t pOutMw = on ? ((s_vledMv * iMa) / 1000UL) : 0U;

  s_currentMa = on ? iMa : 0U;
  s_powerMw = ((pOutMw * 100UL) / CFG_EFFICIENCY_PERCENT) + CFG_STANDBY_POWER_MW;
}

void Monitor_Init(void)
{
  ADC_DrvInit();
  s_vddaMv = ADC_DrvGetVddaMv();
  s_temperature = (int16_t)ADC_DrvGetTemperatureC(s_vddaMv);
  s_lastSampleMs = BSP_GetTickMs();
  s_lastVddaMs = s_lastSampleMs;
  s_lastTempMs = s_lastSampleMs;
}

void Monitor_Service(void)
{
  const uint32_t now = BSP_GetTickMs();

  if ((now - s_lastVddaMs) >= VDDA_PERIOD_MS)
  {
    s_lastVddaMs = now;
    s_vddaMv = ADC_DrvGetVddaMv();
  }
  if ((now - s_lastTempMs) >= TEMP_PERIOD_MS)
  {
    s_lastTempMs = now;
    s_temperature = (int16_t)ADC_DrvGetTemperatureC(s_vddaMv);
  }

  if ((now - s_lastSampleMs) >= CFG_MONITOR_PERIOD_MS)
  {
    uint32_t sum = 0U;
    uint32_t i;
    uint32_t adcMv;

    s_lastSampleMs = now;
    for (i = 0U; i < OVERSAMPLING; i++)
    {
      sum += ADC_DrvRead(ADC_CH_UVLED);
    }
    adcMv = ((sum / OVERSAMPLING) * s_vddaMv) / 4095UL;
    s_vledMv = (adcMv * CFG_UVLED_DIV_NUM) / CFG_UVLED_DIV_DEN;
    update_estimates();

    /* faults are evaluated only with the stage running and settled */
    if ((!Light_IsFaultLatched()) && (Light_GetOnTimeMs() >= CFG_FAULT_BLANKING_MS))
    {
      const uint8_t f = classify(s_vledMv);
      if ((f != 0U) && (f == s_candidate))
      {
        s_debounce++;
        if (s_debounce >= CFG_FAULT_DEBOUNCE_SAMPLES)
        {
          Light_SetFaultLatch();
          s_failure = f;
          OpData_OnLightSourceFault(f);
        }
      }
      else
      {
        s_candidate = f;
        s_debounce = (f != 0U) ? 1U : 0U;
      }
    }
    else
    {
      s_candidate = 0U;
      s_debounce = 0U;
    }
  }
}

uint8_t Monitor_GetFailureStatus(void)
{
  return s_failure;
}

uint32_t Monitor_GetLedVoltageMv(void)
{
  return s_vledMv;
}

uint32_t Monitor_GetLedCurrentMa(void)
{
  return s_currentMa;
}

uint32_t Monitor_GetInputPowerMw(void)
{
  return s_powerMw;
}

int16_t Monitor_GetTemperatureC(void)
{
  return s_temperature;
}

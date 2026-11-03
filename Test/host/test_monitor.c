/**
 * @file    test_monitor.c
 * @brief   Host test of the LED monitoring / fault latch (led_monitor.c),
 *          operating counters (opdata.c) and light output (light_ctrl.c).
 */
#include <stdio.h>
#include "mock_hw.h"
#include "adc_drv.h"
#include "light_ctrl.h"
#include "led_monitor.h"
#include "opdata.h"
#include "variant.h"
#include "app_config.h"

int g_failures = 0;
int g_checks = 0;

/* ---- fakes ------------------------------------------------------------------ */
static uint32_t s_vledMv = 120000U;
void ADC_DrvInit(void) { }
uint32_t ADC_DrvGetVddaMv(void) { return 3300U; }
int32_t ADC_DrvGetTemperatureC(uint32_t vddaMv) { (void)vddaMv; return 35; }
uint16_t ADC_DrvRead(uint32_t channel)
{
  /* inverse of the uVLED divider and of the ADC scaling (+0.5 LSB) */
  const uint32_t adcMv = (s_vledMv * CFG_UVLED_DIV_DEN) / CFG_UVLED_DIV_NUM;
  (void)channel;
  return (uint16_t)(((adcMv * 4095U) + 1650U) / 3300U);
}
uint8_t Variant_GetActive(void) { return 0U; }

static void run(uint32_t ms)
{
  while (ms-- > 0U)
  {
    g_tickMs++;
    Light_Tick1ms();
    Monitor_Service();
    OpData_Service();
  }
}

int main(void)
{
  const NVM_Counters_t *c;

  Mock_Reset();
  Light_Init();
  OpData_Init();
  Monitor_Init();
  c = OpData_Get();
  CHECK_EQ(c->gearStartCounter, 1);

  printf("normal operation\n");
  Light_FadeTo(254U, 0U, 85U);
  run(3000U);
  CHECK(g_powerStage);
  CHECK_EQ(Monitor_GetFailureStatus(), 0);
  CHECK((Monitor_GetLedVoltageMv() > 119000U) && (Monitor_GetLedVoltageMv() < 121000U));
  CHECK_EQ(Monitor_GetLedCurrentMa(), 349);     /* 350 mA nominal, 65535/65536 */
  CHECK_EQ(c->lsStartCounter, 1);
  CHECK_EQ(c->lsOnTimeS, 3);

  printf("blanking: no fault check right after switch-on\n");
  Light_FadeTo(0U, 0U, 85U);
  run(10U);
  s_vledMv = 0U;
  Light_FadeTo(254U, 0U, 85U);
  run(CFG_FAULT_BLANKING_MS - 50U);
  CHECK(g_powerStage);
  CHECK_EQ(c->lsStartCounter, 2);

  printf("short circuit -> OFF latched\n");
  run(200U);
  CHECK(!g_powerStage);
  CHECK(Light_IsFaultLatched());
  CHECK_EQ(Monitor_GetFailureStatus(), MON_FAIL_SHORT_CIRCUIT);
  CHECK_EQ(c->lsFailureCounter, 1);
  CHECK_EQ(c->lsShortCounter, 1);

  printf("latched until reset, DALI level changes ignored\n");
  s_vledMv = 120000U;
  Light_FadeTo(200U, 0U, 85U);
  run(3000U);
  CHECK(!g_powerStage);
  CHECK(!Light_IsLampOn());

  printf("counters persisted\n");
  {
    NVM_Counters_t copy;
    CHECK(NVM_LoadCounters(&copy));
    CHECK_EQ(copy.lsShortCounter, 1);
    CHECK_EQ(copy.gearStartCounter, 1);
  }

  printf("energy accounting\n");
  {
    const uint32_t e0 = c->activeEnergyWh;
    /* standby only: 0.5 W -> 1 Wh every 7200 s */
    run(7200U * 1000U);
    CHECK_EQ(c->activeEnergyWh - e0, 1);
  }

  printf("\ntest_monitor: %d checks, %d failures\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}

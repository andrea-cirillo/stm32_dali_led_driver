/**
 ******************************************************************************
 * @file    adc_drv.c
 * @brief   ADC driver (register level).
 ******************************************************************************
 */
#include "adc_drv.h"
#include "bsp.h"

/* Factory calibration values (STM32F030 datasheet, sect. 3.10) */
#define VREFINT_CAL_ADDR       (0x1FFFF7BAUL)  /* raw at VDDA = 3.3 V, 30 degC */
#define TS_CAL1_ADDR           (0x1FFFF7B8UL)  /* raw at VDDA = 3.3 V, 30 degC */
#define CAL_VDDA_MV            (3300UL)
/* Average slope 4.3 mV/degC -> 5.336 LSB/degC at 3.3 V, scaled by 1000 */
#define TS_SLOPE_LSB_X1000     (5336UL)

/**
 * @brief   Reads a 16-bit factory calibration word from system memory.
 * @param[in] addr  Address of the calibration value (VREFINT_CAL_ADDR or TS_CAL1_ADDR).
 * @return  Calibration value (12-bit ADC code measured at 3.3 V / 30 degC).
 * @note    MISRA C:2012 Rule 11.4 deviation: integer to pointer conversion for a
 * @note    fixed, read-only hardware address.
 */
static uint16_t read_cal(uint32_t addr)
{
  return *(const volatile uint16_t *)addr;
}

/* Public API (full description in adc_drv.h): ADC calibration and enable. */
void ADC_DrvInit(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

  /* asynchronous clock = HSI14 (CKMODE = 00) */
  ADC1->CFGR2 = 0U;

  /* calibration with ADC disabled */
  if ((ADC1->CR & ADC_CR_ADEN) != 0U)
  {
    ADC1->CR |= ADC_CR_ADDIS;
    while ((ADC1->CR & ADC_CR_ADEN) != 0U)
    {
      /* wait disable */
    }
  }
  ADC1->CR |= ADC_CR_ADCAL;
  while ((ADC1->CR & ADC_CR_ADCAL) != 0U)
  {
    /* wait calibration */
  }

  /* 12 bit, right aligned, single conversion, software trigger */
  ADC1->CFGR1 = 0U;
  /* 239.5 cycles sampling: required by the temperature sensor (>= 4 us) */
  ADC1->SMPR = 7U;
  ADC->CCR |= (ADC_CCR_TSEN | ADC_CCR_VREFEN);

  ADC1->ISR = ADC_ISR_ADRDY;
  ADC1->CR |= ADC_CR_ADEN;
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0U)
  {
    /* wait ready */
  }
}

/* Public API (full description in adc_drv.h): single blocking conversion. */
uint16_t ADC_DrvRead(uint32_t channel)
{
  ADC1->CHSELR = (uint32_t)1UL << channel;
  ADC1->ISR = ADC_ISR_EOC | ADC_ISR_OVR;
  ADC1->CR |= ADC_CR_ADSTART;
  while ((ADC1->ISR & ADC_ISR_EOC) == 0U)
  {
    /* ~20 us */
  }
  return (uint16_t)(ADC1->DR & 0x0FFFU);
}

/* Public API (full description in adc_drv.h): VDDA from VREFINT. */
uint32_t ADC_DrvGetVddaMv(void)
{
  const uint32_t raw = ADC_DrvRead(ADC_CH_VREFINT);
  uint32_t vdda = CAL_VDDA_MV;

  if (raw != 0U)
  {
    vdda = (CAL_VDDA_MV * (uint32_t)read_cal(VREFINT_CAL_ADDR)) / raw;
  }
  return vdda;
}

/* Public API (full description in adc_drv.h): die temperature. */
int32_t ADC_DrvGetTemperatureC(uint32_t vddaMv)
{
  const uint32_t raw = ADC_DrvRead(ADC_CH_TEMP);
  /* normalise the reading to the calibration supply (3.3 V) */
  const uint32_t norm = (raw * vddaMv) / CAL_VDDA_MV;
  const uint32_t cal = read_cal(TS_CAL1_ADDR);
  int32_t t;

  /* unsigned arithmetic only: avoids the signed division library code */
  if (cal >= norm)
  {
    t = 30L + (int32_t)(((cal - norm) * 1000UL) / TS_SLOPE_LSB_X1000);
  }
  else
  {
    t = 30L - (int32_t)(((norm - cal) * 1000UL) / TS_SLOPE_LSB_X1000);
  }
  return t;
}

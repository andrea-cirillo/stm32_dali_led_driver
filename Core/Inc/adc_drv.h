/**
 ******************************************************************************
 * @file    adc_drv.h
 * @brief   ADC driver: uVLED (PA1 / ADC_IN1), internal temperature sensor
 *          and VREFINT (used to compute the actual VDDA).
 ******************************************************************************
 */
#ifndef ADC_DRV_H
#define ADC_DRV_H

#include <stdint.h>

#define ADC_CH_UVLED           (1U)
#define ADC_CH_TEMP            (16U)
#define ADC_CH_VREFINT         (17U)

void ADC_DrvInit(void);

/** @brief  Single conversion (blocking, ~20 us). Returns the 12-bit result. */
uint16_t ADC_DrvRead(uint32_t channel);

/** @brief  VDDA in mV computed from VREFINT and its factory calibration. */
uint32_t ADC_DrvGetVddaMv(void);

/** @brief  Die temperature in degC from the factory calibrated sensor. */
int32_t ADC_DrvGetTemperatureC(uint32_t vddaMv);

#endif /* ADC_DRV_H */

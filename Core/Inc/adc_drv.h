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

/**
 * @brief   Calibrates and enables the ADC.
 * @details Clock: HSI14 asynchronous. Runs the self calibration, then
 *          configures 12-bit right aligned single conversions with software
 *          trigger and 239.5 cycles sampling time (~17 us, required by the
 *          temperature sensor), enables VREFINT and the temperature sensor.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_Init() already called (HSI14 running).
 * @note    Context: main, once. Blocking for a few microseconds.
 */
void ADC_DrvInit(void);

/**
 * @brief   Converts one channel (blocking).
 * @param[in] channel  ADC channel number: ADC_CH_UVLED, ADC_CH_TEMP or
 *                     ADC_CH_VREFINT (0..18 accepted by the hardware).
 * @return  12-bit conversion result (0..4095).
 * @note    Context: main loop only. Duration ~20 us.
 */
uint16_t ADC_DrvRead(uint32_t channel);

/**
 * @brief   Measures the actual analog supply voltage.
 * @details VDDA = 3300 mV * VREFINT_CAL / VREFINT_raw, where VREFINT_CAL is
 *          the factory value measured at VDDA = 3.3 V.
 * @par     Parameters
 *          None.
 * @return  VDDA in millivolts (3300 if the reading is 0).
 * @note    Context: main loop only.
 */
uint32_t ADC_DrvGetVddaMv(void);

/**
 * @brief   Measures the die temperature.
 * @details T = 30 degC + (TS_CAL1 - TS_raw_normalised) / 5.336 LSB/degC,
 *          the raw reading being normalised to the 3.3 V calibration supply.
 *          Accuracy is about +/- 5 degC (single point factory calibration).
 * @param[in] vddaMv  Actual VDDA in millivolts (from ADC_DrvGetVddaMv()).
 * @return  Temperature in degrees Celsius.
 * @note    Context: main loop only.
 */
int32_t ADC_DrvGetTemperatureC(uint32_t vddaMv);

#endif /* ADC_DRV_H */

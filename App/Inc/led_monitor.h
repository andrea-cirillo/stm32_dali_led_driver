/**
 ******************************************************************************
 * @file    led_monitor.h
 * @brief   LED output monitoring through uVLED (PA1): open circuit, short
 *          circuit, over voltage and under voltage detection, internal
 *          temperature, estimated current / power.
 *
 *          Every confirmed fault latches the power stage OFF until the next
 *          reset (the check is repeated after every power-up).
 *
 *          Fault bits use the IEC 62386-207 "QUERY FAILURE STATUS" layout:
 *            bit0 short circuit, bit1 open circuit,
 *            bit2 load decrease (under voltage), bit3 load increase (over voltage)
 ******************************************************************************
 */
#ifndef LED_MONITOR_H
#define LED_MONITOR_H

#include <stdint.h>

#define MON_FAIL_SHORT_CIRCUIT   (0x01U)
#define MON_FAIL_OPEN_CIRCUIT    (0x02U)
#define MON_FAIL_LOAD_DECREASE   (0x04U)
#define MON_FAIL_LOAD_INCREASE   (0x08U)

/**
 * @brief   Initialises the ADC and takes the first VDDA / temperature
 *          readings.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_Init() already called.
 * @note    Context: main, once.
 */
void Monitor_Init(void);

/**
 * @brief   Monitoring task, to be called continuously by the main loop.
 * @details - every 100 ms: VDDA from VREFINT;
 *          - every 1 s: die temperature;
 *          - every CFG_MONITOR_PERIOD_MS: uVLED (4 samples averaged),
 *            conversion to LED voltage, update of the current / power
 *            estimates and fault classification.
 *          A fault is evaluated only while the power stage has been on for
 *          at least CFG_FAULT_BLANKING_MS, and confirmed after
 *          CFG_FAULT_DEBOUNCE_SAMPLES identical consecutive samples. Then
 *          the power stage is latched OFF (Light_SetFaultLatch()) and the
 *          light source failure counters are updated.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only (blocking ADC conversions, ~100 us).
 */
void Monitor_Service(void);

/**
 * @brief   Returns the confirmed failure condition.
 * @par     Parameters
 *          None.
 * @return  Bit field MON_FAIL_xxx (IEC 62386-207 QUERY FAILURE STATUS
 *          layout), 0 = no failure. Stays set until reset.
 * @note    Context: any.
 */
uint8_t  Monitor_GetFailureStatus(void);

/**
 * @brief   Returns the last measured LED voltage.
 * @par     Parameters
 *          None.
 * @return  LED voltage in millivolts.
 * @note    Context: any.
 */
uint32_t Monitor_GetLedVoltageMv(void);

/**
 * @brief   Returns the estimated LED current.
 * @details Nominal current of the active variant multiplied by the relative
 *          current requested by the dimming level (no current measurement
 *          is available on the board).
 * @par     Parameters
 *          None.
 * @return  Estimated LED current in milliamperes, 0 when the lamp is off.
 * @note    Context: any.
 */
uint32_t Monitor_GetLedCurrentMa(void);

/**
 * @brief   Returns the estimated active input power.
 * @details P_in = V_LED * I_LED / efficiency + standby power
 *          (CFG_EFFICIENCY_PERCENT, CFG_STANDBY_POWER_MW).
 * @par     Parameters
 *          None.
 * @return  Estimated input power in milliwatts.
 * @note    Context: any. Used for IEC 62386-252 power and energy.
 */
uint32_t Monitor_GetInputPowerMw(void);

/**
 * @brief   Returns the last measured control gear (MCU die) temperature.
 * @par     Parameters
 *          None.
 * @return  Temperature in degrees Celsius.
 * @note    Context: any.
 */
int16_t  Monitor_GetTemperatureC(void);

#endif /* LED_MONITOR_H */

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

void Monitor_Init(void);

/** @brief  Main loop service (sampling every CFG_MONITOR_PERIOD_MS). */
void Monitor_Service(void);

uint8_t  Monitor_GetFailureStatus(void);
uint32_t Monitor_GetLedVoltageMv(void);
uint32_t Monitor_GetLedCurrentMa(void);
uint32_t Monitor_GetInputPowerMw(void);
int16_t  Monitor_GetTemperatureC(void);

#endif /* LED_MONITOR_H */

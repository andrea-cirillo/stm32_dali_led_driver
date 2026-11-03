/**
 ******************************************************************************
 * @file    main.h
 * @brief   DALI-2 LED driver firmware - application entry points.
 *
 *          Execution model (bare metal, no RTOS):
 *            EXTI0_1 / TIM3 (priority 0)  DALI bit timing (dali_phy)
 *            SysTick        (priority 1)  1 ms: DALI command handling, fades,
 *                                         light output (dali_gear, light_ctrl)
 *            main loop                    ADC monitoring and faults, EEPROM,
 *                                         NFC, operating counters, watchdog
 ******************************************************************************
 */
#ifndef MAIN_H
#define MAIN_H

/** @brief  1 ms application tick, called by SysTick_Handler. */
void APP_Tick1ms(void);

/** @brief  NTAG 5 ED pin event, called by EXTI0_1_IRQHandler. */
void APP_NfcEventIrq(void);

#endif /* MAIN_H */

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

/**
 * @brief   1 ms application tick.
 * @details Increments the millisecond time base and, once the application
 *          initialisation is complete, runs Gear_Tick1ms() (DALI frame
 *          processing, timers, fade engine and output update).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick_Handler only.
 */
void APP_Tick1ms(void);

/**
 * @brief   NTAG 5 event detection (ED pin falling edge).
 * @details Forwards the event to the variant manager, which reads the tag
 *          from the main loop once the RF activity has settled.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: EXTI0_1_IRQHandler only.
 */
void APP_NfcEventIrq(void);

#endif /* MAIN_H */

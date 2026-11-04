/**
 ******************************************************************************
 * @file    stm32f0xx_it.h
 * @brief   Interrupt service routines (register level, no HAL).
 ******************************************************************************
 */
#ifndef STM32F0XX_IT_H
#define STM32F0XX_IT_H

/**
 * @brief   Non maskable interrupt (e.g. clock security system).
 * @details Fatal: forces the power stage OFF and resets the MCU.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          Does not return.
 */
void NMI_Handler(void);

/**
 * @brief   Hard fault exception.
 * @details Fatal: forces the power stage OFF and resets the MCU, so a
 *          software fault can never leave the converter running uncontrolled.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          Does not return.
 */
void HardFault_Handler(void);

/**
 * @brief   Supervisor call exception (not used).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 */
void SVC_Handler(void);

/**
 * @brief   PendSV exception (not used).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 */
void PendSV_Handler(void);

/**
 * @brief   1 ms system tick: forwards to APP_Tick1ms() (time base, DALI
 *          command processing, fades and light output).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Priority BSP_PRIO_SYSTICK (1).
 */
void SysTick_Handler(void);

/**
 * @brief   EXTI lines 0 and 1.
 * @details Line 1 (PF1, MCU_RXD): DALI receive edge -> DALI_PHY_RxEdgeIrq().
 *          Line 0 (PA0, NFC_EventDetect): NTAG 5 event -> APP_NfcEventIrq().
 *          The pending flags are cleared before dispatching.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Priority BSP_PRIO_DALI_PHY (0).
 */
void EXTI0_1_IRQHandler(void);

/**
 * @brief   TIM3 global interrupt: DALI receive timeout (CC1) and transmit
 *          half-bit timing (CC2), forwarded to DALI_PHY_TimerIrq().
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Priority BSP_PRIO_DALI_PHY (0).
 */
void TIM3_IRQHandler(void);

#endif /* STM32F0XX_IT_H */

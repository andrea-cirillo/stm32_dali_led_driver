/**
 ******************************************************************************
 * @file    pwm_drv.h
 * @brief   PWM_OUT (PA4, TIM14 CH1) driver. 48 MHz / CFG_PWM_PERIOD_COUNTS.
 *          Higher duty = lower LED current (PWM_OUT offsets CS_out).
 ******************************************************************************
 */
#ifndef PWM_DRV_H
#define PWM_DRV_H

#include <stdint.h>

/**
 * @brief   Starts TIM14 CH1 in PWM mode 1 with preload.
 * @details Period = CFG_PWM_PERIOD_COUNTS timer clocks (11.7 kHz with 4096),
 *          initial duty = CFG_PWM_MAX_DUTY (100 %, i.e. minimum LED current).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_Init() already called (PA4 in AF4).
 * @note    Context: main, once.
 */
void PWM_DrvInit(void);

/**
 * @brief   Sets the PWM_OUT compare value.
 * @details The value is preloaded and takes effect at the next timer
 *          update, so there are no glitches.
 * @param[in] duty  Compare value 0..CFG_PWM_MAX_DUTY: 0 = 0 % (maximum LED
 *                  current), CFG_PWM_MAX_DUTY = 100 % (minimum current).
 *                  Larger values are clamped.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick (light_ctrl). Single register write.
 */
void PWM_DrvSetDuty(uint16_t duty);

#endif /* PWM_DRV_H */

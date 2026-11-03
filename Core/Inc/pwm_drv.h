/**
 ******************************************************************************
 * @file    pwm_drv.h
 * @brief   PWM_OUT (PA4, TIM14 CH1) driver. 48 MHz / CFG_PWM_PERIOD_COUNTS.
 *          Higher duty = lower LED current.
 ******************************************************************************
 */
#ifndef PWM_DRV_H
#define PWM_DRV_H

#include <stdint.h>

void PWM_DrvInit(void);

/** @brief  Compare value 0..CFG_PWM_MAX_DUTY (preloaded, glitch free). */
void PWM_DrvSetDuty(uint16_t duty);

#endif /* PWM_DRV_H */

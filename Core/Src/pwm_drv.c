/**
 ******************************************************************************
 * @file    pwm_drv.c
 * @brief   TIM14 CH1 PWM driver (register level).
 ******************************************************************************
 */
#include "pwm_drv.h"
#include "bsp.h"
#include "app_config.h"

void PWM_DrvInit(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  TIM14->CR1 = TIM_CR1_ARPE;
  TIM14->PSC = 0U;
  TIM14->ARR = CFG_PWM_PERIOD_COUNTS - 1U;
  /* Start at maximum duty = minimum current while the stage is off */
  TIM14->CCR1 = CFG_PWM_MAX_DUTY;
  /* PWM mode 1, preload enabled */
  TIM14->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
  TIM14->CCER = TIM_CCER_CC1E;
  TIM14->EGR = TIM_EGR_UG;
  TIM14->CR1 |= TIM_CR1_CEN;
}

void PWM_DrvSetDuty(uint16_t duty)
{
  /* CCR > ARR gives a constant 100 % high output */
  TIM14->CCR1 = (duty > CFG_PWM_MAX_DUTY) ? CFG_PWM_MAX_DUTY : duty;
}

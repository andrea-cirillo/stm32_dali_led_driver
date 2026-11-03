/**
 ******************************************************************************
 * @file    stm32f0xx_it.c
 * @brief   Interrupt service routines (register level, no HAL).
 ******************************************************************************
 */
#include "stm32f0xx_it.h"
#include "bsp.h"
#include "dali_phy.h"
#include "main.h"

/* Fatal error: power stage OFF, then system reset. */
static void fatal_reset(void)
{
  BSP_PowerStageEnable(false);
  NVIC_SystemReset();
}

void NMI_Handler(void)
{
  fatal_reset();
}

void HardFault_Handler(void)
{
  fatal_reset();
}

void SVC_Handler(void)
{
  /* not used */
}

void PendSV_Handler(void)
{
  /* not used */
}

void SysTick_Handler(void)
{
  APP_Tick1ms();
}

void EXTI0_1_IRQHandler(void)
{
  const uint32_t pr = EXTI->PR;

  if ((pr & EXTI_PR_PR1) != 0U)
  {
    EXTI->PR = EXTI_PR_PR1;
    DALI_PHY_RxEdgeIrq();
  }
  if ((pr & EXTI_PR_PR0) != 0U)
  {
    EXTI->PR = EXTI_PR_PR0;
    APP_NfcEventIrq();
  }
}

void TIM3_IRQHandler(void)
{
  DALI_PHY_TimerIrq();
}

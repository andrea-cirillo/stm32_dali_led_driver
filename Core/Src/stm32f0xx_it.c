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

/**
 * @brief   Fatal error reaction: forces the power stage OFF and resets the MCU.
 * @details Used by NMI and HardFault, so that no fault can leave the converter
 *          running with an uncontrolled output.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          Does not return.
 * @note    Context: exception handlers.
 */
static void fatal_reset(void)
{
  BSP_PowerStageEnable(false);
  NVIC_SystemReset();
}

/* Public API (full description in stm32f0xx_it.h): non maskable interrupt, fatal. */
void NMI_Handler(void)
{
  fatal_reset();
}

/* Public API (full description in stm32f0xx_it.h): hard fault, fatal. */
void HardFault_Handler(void)
{
  fatal_reset();
}

/* Public API (full description in stm32f0xx_it.h): not used. */
void SVC_Handler(void)
{
  /* not used */
}

/* Public API (full description in stm32f0xx_it.h): not used. */
void PendSV_Handler(void)
{
  /* not used */
}

/* Public API (full description in stm32f0xx_it.h): 1 ms application tick. */
void SysTick_Handler(void)
{
  APP_Tick1ms();
}

/* Public API (full description in stm32f0xx_it.h): DALI RX edge (line 1) and NFC event (line 0). */
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

/* Public API (full description in stm32f0xx_it.h): DALI timing. */
void TIM3_IRQHandler(void)
{
  DALI_PHY_TimerIrq();
}

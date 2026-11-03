/**
 ******************************************************************************
 * @file    bsp.h
 * @brief   Board support: clock tree, GPIO map, system tick, watchdog and
 *          power stage (OFF) control. Register level, no HAL.
 *
 *          Pin map (STM32F030F4P6, TSSOP20):
 *            PF0  MCU_TXD          DALI transmit (GPIO output)
 *            PF1  MCU_RXD          DALI receive  (GPIO input, EXTI1)
 *            PA0  NFC_EventDetect  NTAG 5 ED pin (input, EXTI0, active low)
 *            PA1  uVLED            ADC_IN1, LED output voltage
 *            PA3  OFF              power stage shutdown, HIGH = converter OFF
 *            PA4  PWM_OUT          TIM14_CH1, current sense offset
 *            PA6  STCP             shift register storage clock
 *            PA7  SHCP             shift register shift clock
 *            PB1  CurrentLimit_DS  shift register serial data
 *            PA9  SCL / PA10 SDA   I2C1 (M24C04 EEPROM + NTAG 5)
 ******************************************************************************
 */
#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f0xx.h"

#define BSP_SYSCLK_HZ          (48000000UL)

/* ---- GPIO map ------------------------------------------------------------ */
#define BSP_DALI_TX_PORT       GPIOF
#define BSP_DALI_TX_PIN        (0U)
#define BSP_DALI_RX_PORT       GPIOF
#define BSP_DALI_RX_PIN        (1U)
#define BSP_NFC_ED_PORT        GPIOA
#define BSP_NFC_ED_PIN         (0U)
#define BSP_UVLED_PORT         GPIOA
#define BSP_UVLED_PIN          (1U)
#define BSP_OFF_PORT           GPIOA
#define BSP_OFF_PIN            (3U)
#define BSP_PWM_PORT           GPIOA
#define BSP_PWM_PIN            (4U)
#define BSP_SR_STCP_PORT       GPIOA
#define BSP_SR_STCP_PIN        (6U)
#define BSP_SR_SHCP_PORT       GPIOA
#define BSP_SR_SHCP_PIN        (7U)
#define BSP_SR_DS_PORT         GPIOB
#define BSP_SR_DS_PIN          (1U)
#define BSP_I2C_SCL_PIN        (9U)
#define BSP_I2C_SDA_PIN        (10U)

/* Single bit mask of a pin number. */
#define BSP_PIN_MASK(pin)      ((uint32_t)1UL << (pin))

/* ---- Interrupt priorities (Cortex-M0: 0 = highest, 3 = lowest) ----------- */
#define BSP_PRIO_DALI_PHY      (0U)   /* EXTI0_1 (RX edges) and TIM3 (timing) */
#define BSP_PRIO_SYSTICK       (1U)   /* 1 ms application tick                */

/**
 * @brief  Must be the first call in main(): forces the power stage OFF so the
 *         converter cannot start with an undefined current limit selection.
 */
void BSP_EarlyInit(void);

/** @brief  Clock tree (48 MHz from HSI/2 x 12), GPIO, SysTick. */
void BSP_Init(void);

/** @brief  Milliseconds since reset (wraps after 49 days). */
uint32_t BSP_GetTickMs(void);

/** @brief  Busy wait, watchdog-safe for short delays (< 100 ms). */
void BSP_DelayMs(uint32_t ms);

/** @brief  Called by SysTick_Handler only. */
void BSP_TickIncrement(void);

/** @brief  true = enable the power stage (OFF low), false = shut it down. */
void BSP_PowerStageEnable(bool enable);
bool BSP_PowerStageIsEnabled(void);

void BSP_WatchdogInit(void);
void BSP_WatchdogRefresh(void);

/** @brief  Short critical sections (PRIMASK based, nestable). */
uint32_t BSP_EnterCritical(void);
void BSP_ExitCritical(uint32_t primask);

/** @brief  Reads the 96-bit unique device ID. */
void BSP_GetUid(uint8_t uid[12]);

#endif /* BSP_H */

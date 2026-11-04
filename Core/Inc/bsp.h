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

/** Single bit mask of a pin number (0..15). */
#define BSP_PIN_MASK(pin)      ((uint32_t)1UL << (pin))

/* ---- Interrupt priorities (Cortex-M0: 0 = highest, 3 = lowest) ----------- */
#define BSP_PRIO_DALI_PHY      (0U)   /* EXTI0_1 (RX edges) and TIM3 (timing) */
#define BSP_PRIO_SYSTICK       (1U)   /* 1 ms application tick                */

/**
 * @brief   Forces the power stage OFF as early as possible after reset.
 * @details Enables the GPIOA/GPIOB/GPIOF clocks, drives PA3 (OFF) HIGH and
 *          configures it as push-pull output. Until this point the pin is a
 *          floating input and the board pull-down keeps the converter ON, so
 *          this must be the very first instruction of main(): it shortens
 *          the window in which the converter could run with an undefined
 *          current limitation (shift register content is random at power-up).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, before any other initialisation.
 */
void BSP_EarlyInit(void);

/**
 * @brief   Initialises clock tree, GPIO and the 1 ms SysTick.
 * @details - Flash: 1 wait state + prefetch.
 *          - SYSCLK = PLL = HSI/2 x 12 = 48 MHz, AHB = APB = 48 MHz.
 *          - HSI14 enabled as asynchronous ADC clock.
 *          - I2C1 kernel clock = HSI 8 MHz.
 *          - All GPIOs configured according to the pin map (unused pins in
 *            analog mode to minimise consumption).
 *          - SysTick: 1 ms period, priority BSP_PRIO_SYSTICK, interrupt on.
 *          Updates the CMSIS variable SystemCoreClock.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_EarlyInit() already called.
 * @note    Context: main, once. Blocks until PLL and HSI14 are ready.
 */
void BSP_Init(void);

/**
 * @brief   Returns the milliseconds elapsed since BSP_Init().
 * @par     Parameters
 *          None.
 * @return  Free running millisecond counter. It wraps after ~49.7 days:
 *          always compare times with unsigned differences (now - start).
 * @note    Context: any. 32-bit read, atomic on Cortex-M0.
 */
uint32_t BSP_GetTickMs(void);

/**
 * @brief   Busy waits for the requested number of milliseconds.
 * @param[in] ms  Delay in milliseconds. Keep it well below the watchdog
 *                period (~1 s): the watchdog is not refreshed here.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only (needs the SysTick interrupt running).
 */
void BSP_DelayMs(uint32_t ms);

/**
 * @brief   Increments the millisecond counter.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick interrupt only (called by APP_Tick1ms()).
 */
void BSP_TickIncrement(void);

/**
 * @brief   Enables or disables the LED power stage through the OFF pin.
 * @param[in] enable  true: OFF LOW, converter allowed to run.
 *                    false: OFF HIGH, converter shut down.
 * @par     Return value
 *          None (void).
 * @note    Context: any. Uses the atomic BSRR/BRR registers.
 */
void BSP_PowerStageEnable(bool enable);

/**
 * @brief   Reads back the commanded state of the power stage.
 * @par     Parameters
 *          None.
 * @return  true if the OFF pin is driven LOW (stage enabled), false otherwise.
 * @note    Context: any. Reflects the output latch, not a measurement.
 */
bool BSP_PowerStageIsEnabled(void);

/**
 * @brief   Starts the independent watchdog (IWDG).
 * @details LSI ~40 kHz, prescaler /32, reload 1250: timeout ~1 s
 *          (0.8 s .. 1.3 s over the LSI tolerance). Once started the IWDG
 *          cannot be stopped; BSP_WatchdogRefresh() must be called by the
 *          main loop.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once, after the slow initialisation steps.
 */
void BSP_WatchdogInit(void);

/**
 * @brief   Reloads the watchdog counter.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop. Must run at least every 800 ms.
 */
void BSP_WatchdogRefresh(void);

/**
 * @brief   Enters a critical section (all maskable interrupts disabled).
 * @details Saves PRIMASK and disables interrupts, so the sections can be
 *          nested. Keep the protected code very short (a few microseconds):
 *          it delays the DALI edge time stamping.
 * @par     Parameters
 *          None.
 * @return  Previous PRIMASK value, to be passed to BSP_ExitCritical().
 * @note    Context: any.
 */
uint32_t BSP_EnterCritical(void);

/**
 * @brief   Leaves a critical section.
 * @param[in] primask  Value returned by the matching BSP_EnterCritical().
 * @par     Return value
 *          None (void).
 * @note    Context: any.
 */
void BSP_ExitCritical(uint32_t primask);

/**
 * @brief   Reads the 96-bit unique device identifier of the STM32.
 * @param[out] uid  Destination buffer, 12 bytes, filled with the UID bytes
 *                  in address order (base 0x1FFFF7AC).
 * @par     Return value
 *          None (void).
 * @note    Context: any.
 */
void BSP_GetUid(uint8_t uid[12]);

#endif /* BSP_H */

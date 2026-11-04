/**
 ******************************************************************************
 * @file    bsp.c
 * @brief   Board support package (register level).
 ******************************************************************************
 */
#include "bsp.h"
#include "app_config.h"

/* GPIO MODER values */
#define GPIO_MODE_INPUT        (0UL)
#define GPIO_MODE_OUTPUT       (1UL)
#define GPIO_MODE_AF           (2UL)
#define GPIO_MODE_ANALOG       (3UL)

/* Unique device ID base address (STM32F030, RM0360 sect. 30.1) */
#define BSP_UID_BASE_ADDR      (0x1FFFF7ACUL)

/* IWDG: LSI ~40 kHz / 32 = 1.25 kHz, reload 1250 -> ~1 s (0.8 s .. 1.3 s) */
#define IWDG_KEY_ENABLE        (0x0000CCCCUL)
#define IWDG_KEY_ACCESS        (0x00005555UL)
#define IWDG_KEY_RELOAD        (0x0000AAAAUL)
#define IWDG_PRESCALER_32      (3UL)
#define IWDG_RELOAD_VALUE      (1250UL)

static volatile uint32_t s_tickMs = 0U;

/**
 * @brief   Sets the MODER field of one GPIO pin.
 * @details Read-modify-write of the 2-bit mode field; the other pins are untouched.
 * @param[in] port  GPIO port (GPIOA, GPIOB, GPIOF).
 * @param[in] pin   Pin number 0..15.
 * @param[in] mode  GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF or GPIO_MODE_ANALOG.
 * @par     Return value
 *          None (void).
 * @note    Context: main (initialisation only, not interrupt safe).
 */
static void gpio_set_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
  uint32_t moder = port->MODER;
  moder &= ~(3UL << (pin * 2UL));
  moder |= (mode << (pin * 2UL));
  port->MODER = moder;
}

/**
 * @brief   Selects the alternate function of one GPIO pin.
 * @details Writes the 4-bit AFRL / AFRH field. The pin must also be set to
 *          GPIO_MODE_AF with gpio_set_mode().
 * @param[in] port  GPIO port.
 * @param[in] pin   Pin number 0..15.
 * @param[in] af    Alternate function number 0..7 (e.g. 4 = I2C1 / TIM14).
 * @par     Return value
 *          None (void).
 * @note    Context: main (initialisation only).
 */
static void gpio_set_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
  const uint32_t idx = pin >> 3U;
  const uint32_t shift = (pin & 7UL) * 4UL;
  uint32_t afr = port->AFR[idx];
  afr &= ~(0xFUL << shift);
  afr |= (af << shift);
  port->AFR[idx] = afr;
}

/* Public API (full description in bsp.h): forces the power stage OFF right after reset. */
void BSP_EarlyInit(void)
{
  /* Power stage OFF as soon as possible (OFF pin HIGH). */
  RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN);
  BSP_OFF_PORT->BSRR = BSP_PIN_MASK(BSP_OFF_PIN);
  gpio_set_mode(BSP_OFF_PORT, BSP_OFF_PIN, GPIO_MODE_OUTPUT);
}

/**
 * @brief   Configures the clock tree: SYSCLK 48 MHz from PLL (HSI/2 x 12).
 * @details Sets 1 flash wait state with prefetch, starts the PLL and switches
 *          SYSCLK to it, updates SystemCoreClock, starts HSI14 for the ADC,
 *          selects HSI as I2C1 kernel clock and enables the SYSCFG clock.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once. Busy waits for PLL / HSI14 ready.
 */
static void clock_init(void)
{
  /* Flash: 1 wait state + prefetch for 48 MHz */
  FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

  /* PLL: HSI/2 x 12 = 48 MHz */
  RCC->CFGR &= ~(RCC_CFGR_PLLMUL | RCC_CFGR_PLLSRC | RCC_CFGR_HPRE | RCC_CFGR_PPRE);
  RCC->CFGR |= RCC_CFGR_PLLMUL12;
  RCC->CR |= RCC_CR_PLLON;
  while ((RCC->CR & RCC_CR_PLLRDY) == 0U)
  {
    /* wait PLL lock */
  }
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
  {
    /* wait switch */
  }
  SystemCoreClock = BSP_SYSCLK_HZ;

  /* HSI14 dedicated ADC clock */
  RCC->CR2 |= RCC_CR2_HSI14ON;
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0U)
  {
    /* wait HSI14 */
  }

  /* I2C1 kernel clock = HSI 8 MHz (reset value of I2C1SW) */
  RCC->CFGR3 &= ~RCC_CFGR3_I2C1SW;

  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
}

/**
 * @brief   Configures every GPIO according to the board pin map.
 * @details DALI TX released (idle level depends on DALI_CFG_TX_ACTIVE_HIGH),
 *          DALI RX and NFC ED inputs, uVLED and unused pins analog, shift
 *          register lines low, PWM_OUT on AF4 (TIM14), I2C1 on AF4 open drain.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once. OFF (PA3) is configured by BSP_EarlyInit().
 */
static void gpio_init(void)
{
  /* DALI TX: idle = bus released */
#if (DALI_CFG_TX_ACTIVE_HIGH != 0)
  BSP_DALI_TX_PORT->BRR = BSP_PIN_MASK(BSP_DALI_TX_PIN);
#else
  BSP_DALI_TX_PORT->BSRR = BSP_PIN_MASK(BSP_DALI_TX_PIN);
#endif
  gpio_set_mode(BSP_DALI_TX_PORT, BSP_DALI_TX_PIN, GPIO_MODE_OUTPUT);
  gpio_set_mode(BSP_DALI_RX_PORT, BSP_DALI_RX_PIN, GPIO_MODE_INPUT);

  /* NFC ED: input, external pull-up R109 */
  gpio_set_mode(BSP_NFC_ED_PORT, BSP_NFC_ED_PIN, GPIO_MODE_INPUT);

  /* uVLED analog, unused PA2 / PA5 analog (lowest consumption) */
  gpio_set_mode(BSP_UVLED_PORT, BSP_UVLED_PIN, GPIO_MODE_ANALOG);
  gpio_set_mode(GPIOA, 2U, GPIO_MODE_ANALOG);
  gpio_set_mode(GPIOA, 5U, GPIO_MODE_ANALOG);

  /* Shift register lines: outputs, low */
  BSP_SR_STCP_PORT->BRR = BSP_PIN_MASK(BSP_SR_STCP_PIN) | BSP_PIN_MASK(BSP_SR_SHCP_PIN);
  BSP_SR_DS_PORT->BRR = BSP_PIN_MASK(BSP_SR_DS_PIN);
  gpio_set_mode(BSP_SR_STCP_PORT, BSP_SR_STCP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_mode(BSP_SR_SHCP_PORT, BSP_SR_SHCP_PIN, GPIO_MODE_OUTPUT);
  gpio_set_mode(BSP_SR_DS_PORT, BSP_SR_DS_PIN, GPIO_MODE_OUTPUT);

  /* PWM_OUT: AF4 = TIM14_CH1 */
  gpio_set_af(BSP_PWM_PORT, BSP_PWM_PIN, 4UL);
  gpio_set_mode(BSP_PWM_PORT, BSP_PWM_PIN, GPIO_MODE_AF);

  /* I2C1: AF4, open drain, external pull-ups R81 / R84 */
  GPIOA->OTYPER |= BSP_PIN_MASK(BSP_I2C_SCL_PIN) | BSP_PIN_MASK(BSP_I2C_SDA_PIN);
  gpio_set_af(GPIOA, BSP_I2C_SCL_PIN, 4UL);
  gpio_set_af(GPIOA, BSP_I2C_SDA_PIN, 4UL);
  gpio_set_mode(GPIOA, BSP_I2C_SCL_PIN, GPIO_MODE_AF);
  gpio_set_mode(GPIOA, BSP_I2C_SDA_PIN, GPIO_MODE_AF);
}

/* Public API (full description in bsp.h): clock tree, GPIO and 1 ms SysTick. */
void BSP_Init(void)
{
  clock_init();
  gpio_init();

  /* SysTick 1 ms */
  SysTick->LOAD = (BSP_SYSCLK_HZ / 1000UL) - 1UL;
  SysTick->VAL = 0U;
  NVIC_SetPriority(SysTick_IRQn, BSP_PRIO_SYSTICK);
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/* Public API (full description in bsp.h): milliseconds since start-up. */
uint32_t BSP_GetTickMs(void)
{
  return s_tickMs;
}

/* Public API (full description in bsp.h): millisecond counter increment (SysTick). */
void BSP_TickIncrement(void)
{
  s_tickMs++;
}

/* Public API (full description in bsp.h): busy wait in milliseconds. */
void BSP_DelayMs(uint32_t ms)
{
  const uint32_t start = s_tickMs;
  while ((s_tickMs - start) < ms)
  {
    /* busy wait */
  }
}

/* Public API (full description in bsp.h): drives the OFF pin. */
void BSP_PowerStageEnable(bool enable)
{
  if (enable)
  {
    BSP_OFF_PORT->BRR = BSP_PIN_MASK(BSP_OFF_PIN);
  }
  else
  {
    BSP_OFF_PORT->BSRR = BSP_PIN_MASK(BSP_OFF_PIN);
  }
}

/* Public API (full description in bsp.h): reads back the OFF pin latch. */
bool BSP_PowerStageIsEnabled(void)
{
  return ((BSP_OFF_PORT->ODR & BSP_PIN_MASK(BSP_OFF_PIN)) == 0U);
}

/* Public API (full description in bsp.h): starts the ~1 s independent watchdog. */
void BSP_WatchdogInit(void)
{
  IWDG->KR = IWDG_KEY_ENABLE;
  IWDG->KR = IWDG_KEY_ACCESS;
  IWDG->PR = IWDG_PRESCALER_32;
  IWDG->RLR = IWDG_RELOAD_VALUE;
  while (IWDG->SR != 0U)
  {
    /* wait register update */
  }
  IWDG->KR = IWDG_KEY_RELOAD;
}

/* Public API (full description in bsp.h): reloads the watchdog. */
void BSP_WatchdogRefresh(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}

/* Public API (full description in bsp.h): nestable critical section entry. */
uint32_t BSP_EnterCritical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

/* Public API (full description in bsp.h): critical section exit. */
void BSP_ExitCritical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

/* Public API (full description in bsp.h): reads the 96-bit device UID. */
void BSP_GetUid(uint8_t uid[12])
{
  /* MISRA 11.4 deviation: fixed system memory address of the device UID. */
  const volatile uint8_t *const p = (const volatile uint8_t *)BSP_UID_BASE_ADDR;
  uint32_t i;
  for (i = 0U; i < 12U; i++)
  {
    uid[i] = p[i];
  }
}

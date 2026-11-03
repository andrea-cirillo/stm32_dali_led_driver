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

static void gpio_set_mode(GPIO_TypeDef *port, uint32_t pin, uint32_t mode)
{
  uint32_t moder = port->MODER;
  moder &= ~(3UL << (pin * 2UL));
  moder |= (mode << (pin * 2UL));
  port->MODER = moder;
}

static void gpio_set_af(GPIO_TypeDef *port, uint32_t pin, uint32_t af)
{
  const uint32_t idx = pin >> 3U;
  const uint32_t shift = (pin & 7UL) * 4UL;
  uint32_t afr = port->AFR[idx];
  afr &= ~(0xFUL << shift);
  afr |= (af << shift);
  port->AFR[idx] = afr;
}

void BSP_EarlyInit(void)
{
  /* Power stage OFF as soon as possible (OFF pin HIGH). */
  RCC->AHBENR |= (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN);
  BSP_OFF_PORT->BSRR = BSP_PIN_MASK(BSP_OFF_PIN);
  gpio_set_mode(BSP_OFF_PORT, BSP_OFF_PIN, GPIO_MODE_OUTPUT);
}

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

uint32_t BSP_GetTickMs(void)
{
  return s_tickMs;
}

void BSP_TickIncrement(void)
{
  s_tickMs++;
}

void BSP_DelayMs(uint32_t ms)
{
  const uint32_t start = s_tickMs;
  while ((s_tickMs - start) < ms)
  {
    /* busy wait */
  }
}

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

bool BSP_PowerStageIsEnabled(void)
{
  return ((BSP_OFF_PORT->ODR & BSP_PIN_MASK(BSP_OFF_PIN)) == 0U);
}

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

void BSP_WatchdogRefresh(void)
{
  IWDG->KR = IWDG_KEY_RELOAD;
}

uint32_t BSP_EnterCritical(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

void BSP_ExitCritical(uint32_t primask)
{
  __set_PRIMASK(primask);
}

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

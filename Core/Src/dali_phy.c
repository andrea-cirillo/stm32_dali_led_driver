/**
 ******************************************************************************
 * @file    dali_phy.c
 * @brief   DALI physical / data link layer (IEC 62386-101).
 *
 *          Manchester coding (bus levels): logical '1' = LOW then HIGH,
 *          logical '0' = HIGH then LOW, start bit = '1', MSB first.
 *          Te (half bit) = 416.67 us.
 *
 *          Decoder: h counts the half bits elapsed since the start edge.
 *          A short interval (1 Te) adds 1, a long one (2 Te) adds 2. Edges at
 *          odd h are mid-bit edges and carry the bit value (level after the
 *          edge); edges at even h are bit boundaries. A long interval always
 *          ends on a mid-bit edge.
 ******************************************************************************
 */
#include "dali_phy.h"
#include "bsp.h"
#include "app_config.h"

/* ---- timing (microseconds) ------------------------------------------------ */
#define TE_US                  (417U)   /* nominal half bit (416.67 us)       */
#define RX_HALF_MIN_US         (320U)   /* spec 333.3 .. 500 us + margin       */
#define RX_HALF_MAX_US         (520U)
#define RX_FULL_MIN_US         (650U)   /* spec 666.7 .. 1000 us + margin      */
#define RX_FULL_MAX_US         (1040U)
#define RX_SPIKE_US            (100U)   /* shorter first pulse = noise         */
#define RX_STOP_US             (1200U)  /* idle time marking the frame end     */
#define RX_MAX_BITS            (32U)

#define TX_SETTLING_US         (7000U)  /* forward -> backward, spec 5.5..10.5 ms */
#define TX_LATEST_US           (9500U)  /* latest acceptable start            */
#define TX_MIN_LEAD_US         (60U)
#define TX_HALF_BITS           (18U)    /* start bit + 8 data bits            */
#define TX_STOP_US             (4U * TE_US) /* 2 stop bits of idle            */

typedef enum
{
  RX_IDLE = 0,
  RX_FRAME,
  RX_ERROR
} RxState_t;

/* RX context (ISR owned) */
static volatile RxState_t s_rxState = RX_IDLE;
static uint16_t s_lastEdge;
static uint8_t  s_half;
static uint32_t s_rxData;
static uint8_t  s_rxBits;
static volatile uint16_t s_frameEnd;

/* Mailbox ISR -> application */
static volatile bool s_frameReady = false;
static DALI_Frame_t s_frame;

/* TX context */
static volatile bool s_txActive = false;
static uint32_t s_txPattern;   /* bit i = 1: bus active (LOW) during half bit i */
static uint8_t  s_txIdx;

/**
 * @brief   Samples the DALI bus level through MCU_RXD.
 * @details Applies DALI_CFG_RX_INVERTED to convert the pin level to the bus level.
 * @par     Parameters
 *          None.
 * @return  true if the bus is active (LOW), false if idle (HIGH).
 * @note    Context: any.
 */
static bool bus_is_low(void)
{
  const bool pinHigh = ((BSP_DALI_RX_PORT->IDR & BSP_PIN_MASK(BSP_DALI_RX_PIN)) != 0U);
#if (DALI_CFG_RX_INVERTED != 0)
  return pinHigh;
#else
  return !pinHigh;
#endif
}

/**
 * @brief   Drives the DALI bus through MCU_TXD.
 * @details Applies DALI_CFG_TX_ACTIVE_HIGH to convert the bus state to the pin level.
 * @param[in] active  true: pull the bus LOW (active), false: release it (idle HIGH).
 * @par     Return value
 *          None (void).
 * @note    Context: any. Atomic BSRR / BRR write.
 */
static void tx_bus_active(bool active)
{
#if (DALI_CFG_TX_ACTIVE_HIGH != 0)
  const bool pinHigh = active;
#else
  const bool pinHigh = !active;
#endif
  if (pinHigh)
  {
    BSP_DALI_TX_PORT->BSRR = BSP_PIN_MASK(BSP_DALI_TX_PIN);
  }
  else
  {
    BSP_DALI_TX_PORT->BRR = BSP_PIN_MASK(BSP_DALI_TX_PIN);
  }
}

/**
 * @brief   Arms the TIM3 CC1 compare used as receive stop / quiet-bus timeout.
 * @details Clears a pending CC1 flag and enables the CC1 interrupt.
 * @param[in] at  Absolute TIM3 time (us, 16-bit wrap) of the timeout.
 * @par     Return value
 *          None (void).
 * @note    Context: DALI PHY interrupts / initialisation.
 */
static void rx_arm_timeout(uint16_t at)
{
  TIM3->CCR1 = at;
  TIM3->SR = ~TIM_SR_CC1IF;
  TIM3->DIER |= TIM_DIER_CC1IE;
}

/**
 * @brief   Stores a completed (or failed) frame in the one-slot mailbox.
 * @param[in] data  Received bits, MSB first, right aligned.
 * @param[in] bits  Number of data bits, 0 for a framing / timing error.
 * @par     Return value
 *          None (void).
 * @note    Context: DALI PHY interrupts. Overwrites a non consumed frame.
 */
static void rx_publish(uint32_t data, uint8_t bits)
{
  s_frame.data = data;
  s_frame.bits = bits;
  s_frameReady = true;
}

/* Public API (full description in dali_phy.h): DALI transceiver initialisation. */
void DALI_PHY_Init(void)
{
  tx_bus_active(false);

  /* TIM3: 1 MHz free running time base, CC1 = RX timeout, CC2 = TX */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CR1 = 0U;
  TIM3->PSC = (uint16_t)((BSP_SYSCLK_HZ / 1000000UL) - 1UL);
  TIM3->ARR = 0xFFFFU;
  TIM3->CCMR1 = 0U;   /* frozen output compare, no pin */
  TIM3->DIER = 0U;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR = 0U;
  TIM3->CR1 = TIM_CR1_CEN;
  NVIC_SetPriority(TIM3_IRQn, BSP_PRIO_DALI_PHY);
  NVIC_EnableIRQ(TIM3_IRQn);

  /* EXTI1 <- PF1, both edges */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI1) | SYSCFG_EXTICR1_EXTI1_PF;
  EXTI->RTSR |= EXTI_RTSR_TR1;
  EXTI->FTSR |= EXTI_FTSR_TR1;
  EXTI->PR = EXTI_PR_PR1;
  EXTI->IMR |= EXTI_IMR_MR1;
  NVIC_SetPriority(EXTI0_1_IRQn, BSP_PRIO_DALI_PHY);
  NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* Start in error state: a frame is accepted only after a valid idle time */
  s_rxState = RX_ERROR;
  rx_arm_timeout((uint16_t)((uint16_t)TIM3->CNT + RX_STOP_US));
}

/* Public API (full description in dali_phy.h): Manchester decoder, one call per RX edge. */
void DALI_PHY_RxEdgeIrq(void)
{
  const uint16_t now = (uint16_t)TIM3->CNT;
  const bool low = bus_is_low();

  if (s_txActive)
  {
    /* own transmission echoed by the interface: ignore */
  }
  else if (s_rxState == RX_IDLE)
  {
    if (low)
    {
      s_rxState = RX_FRAME;
      s_half = 0U;
      s_rxData = 0U;
      s_rxBits = 0U;
      s_lastEdge = now;
      rx_arm_timeout((uint16_t)(now + RX_STOP_US));
    }
  }
  else if (s_rxState == RX_FRAME)
  {
    const uint16_t dt = (uint16_t)(now - s_lastEdge);
    bool ok = true;

    s_lastEdge = now;
    if ((dt >= RX_HALF_MIN_US) && (dt <= RX_HALF_MAX_US))
    {
      s_half++;
    }
    else if ((dt >= RX_FULL_MIN_US) && (dt <= RX_FULL_MAX_US))
    {
      s_half = (uint8_t)(s_half + 2U);
      ok = ((s_half & 1U) != 0U);
    }
    else
    {
      ok = false;
    }

    if (!ok)
    {
      if ((s_half == 0U) && (dt < RX_SPIKE_US))
      {
        s_rxState = RX_IDLE;          /* spike on the idle bus: ignore */
      }
      else
      {
        s_rxState = RX_ERROR;
        rx_publish(0U, 0U);
      }
    }
    else if ((s_half & 1U) != 0U)
    {
      const uint32_t bit = low ? 0U : 1U;
      if (s_half == 1U)
      {
        if (bit != 1U)
        {
          s_rxState = RX_ERROR;       /* start bit must be '1' */
          rx_publish(0U, 0U);
        }
      }
      else if (s_rxBits >= RX_MAX_BITS)
      {
        s_rxState = RX_ERROR;
        rx_publish(0U, 0U);
      }
      else
      {
        s_rxData = (s_rxData << 1U) | bit;
        s_rxBits++;
      }
    }
    else
    {
      /* bit boundary edge: no data */
    }
    rx_arm_timeout((uint16_t)(now + RX_STOP_US));
  }
  else
  {
    /* error state: wait for a quiet bus */
    rx_arm_timeout((uint16_t)(now + RX_STOP_US));
  }
}

/**
 * @brief   Transmit state machine step, called on every TIM3 CC2 match.
 * @details Indexes 0..17: outputs half bit n of the precomputed pattern and
 *          schedules the next one 417 / 416 us later (average 416.5 us).
 *          Index 18: releases the bus and waits the 2 stop bits.
 *          Afterwards: disables CC2, ends the transmission, re-enables RX.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: TIM3 interrupt only.
 */
static void tx_step(void)
{
  if (s_txIdx < TX_HALF_BITS)
  {
    tx_bus_active(((s_txPattern >> s_txIdx) & 1U) != 0U);
    /* 417 / 416 us alternated: average 416.5 us */
    TIM3->CCR2 = (uint16_t)(TIM3->CCR2 + (((s_txIdx & 1U) == 0U) ? TE_US : (TE_US - 1U)));
    s_txIdx++;
  }
  else if (s_txIdx == TX_HALF_BITS)
  {
    tx_bus_active(false);
    TIM3->CCR2 = (uint16_t)(TIM3->CCR2 + TX_STOP_US);
    s_txIdx++;
  }
  else
  {
    TIM3->DIER &= ~TIM_DIER_CC2IE;
    s_txActive = false;
    s_rxState = RX_IDLE;
  }
}

/* Public API (full description in dali_phy.h): RX timeout (CC1) and TX timing (CC2). */
void DALI_PHY_TimerIrq(void)
{
  const uint32_t sr = TIM3->SR;

  if (((sr & TIM_SR_CC1IF) != 0U) && ((TIM3->DIER & TIM_DIER_CC1IE) != 0U))
  {
    TIM3->SR = ~TIM_SR_CC1IF;
    TIM3->DIER &= ~TIM_DIER_CC1IE;
    if (bus_is_low())
    {
      /* bus stuck low (or collision): wait until it is released */
      if (s_rxState == RX_FRAME)
      {
        rx_publish(0U, 0U);
      }
      s_rxState = RX_ERROR;
      rx_arm_timeout((uint16_t)((uint16_t)TIM3->CNT + RX_STOP_US));
    }
    else if (s_rxState == RX_FRAME)
    {
      /* end of the last bit: half a bit after a final mid-bit edge */
      s_frameEnd = (uint16_t)(s_lastEdge + (((s_half & 1U) != 0U) ? TE_US : 0U));
      s_rxState = RX_IDLE;
      rx_publish(s_rxData, s_rxBits);
    }
    else
    {
      s_rxState = RX_IDLE;
    }
  }

  if (((sr & TIM_SR_CC2IF) != 0U) && ((TIM3->DIER & TIM_DIER_CC2IE) != 0U))
  {
    TIM3->SR = ~TIM_SR_CC2IF;
    tx_step();
  }
}

/* Public API (full description in dali_phy.h): fetches the last received frame. */
bool DALI_PHY_GetFrame(DALI_Frame_t *frame)
{
  bool ready = false;

  if (s_frameReady)
  {
    const uint32_t pm = BSP_EnterCritical();
    *frame = s_frame;
    s_frameReady = false;
    BSP_ExitCritical(pm);
    ready = true;
  }
  return ready;
}

/* Public API (full description in dali_phy.h): schedules a backward frame. */
void DALI_PHY_SendBackward(uint8_t value)
{
  const uint16_t now = (uint16_t)TIM3->CNT;
  const uint16_t elapsed = (uint16_t)(now - s_frameEnd);
  uint32_t pattern = 1U;          /* start bit: active then idle */
  uint32_t i;

  if ((!s_txActive) && (elapsed <= TX_LATEST_US))
  {
    for (i = 0U; i < 8U; i++)
    {
      const uint32_t bit = ((uint32_t)value >> (7U - i)) & 1U;
      /* '1': active then idle, '0': idle then active */
      const uint32_t halves = (bit != 0U) ? 1U : 2U;
      pattern |= halves << (2U + (2U * i));
    }
    s_txPattern = pattern;
    s_txIdx = 0U;
    s_txActive = true;
    TIM3->CCR2 = (elapsed < (TX_SETTLING_US - TX_MIN_LEAD_US)) ?
                 (uint16_t)(s_frameEnd + TX_SETTLING_US) :
                 (uint16_t)(now + TX_MIN_LEAD_US);
    TIM3->SR = ~TIM_SR_CC2IF;
    TIM3->DIER |= TIM_DIER_CC2IE;
  }
}

/* Public API (full description in dali_phy.h): bus level sample. */
bool DALI_PHY_IsBusIdle(void)
{
  return !bus_is_low();
}

/* Public API (full description in dali_phy.h): 1 MHz free running time base. */
uint16_t DALI_PHY_GetTimeUs(void)
{
  return (uint16_t)TIM3->CNT;
}

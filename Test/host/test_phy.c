/**
 * @file    test_phy.c
 * @brief   Host test of the DALI Manchester decoder / encoder (dali_phy.c).
 *          Edges are injected by writing the fake GPIOF->IDR and TIM3->CNT.
 */
#include <stdio.h>
#include "mock_hw.h"
#include "bsp.h"
#include "dali_phy.h"

int g_failures = 0;
int g_checks = 0;

static uint16_t s_t = 1000U;      /* simulated microsecond time */

static void bus_level(bool high)
{
  /* default polarity: RXD follows the bus level */
  if (high) { mockGPIOF.IDR |= (1UL << 1); } else { mockGPIOF.IDR &= ~(1UL << 1); }
}

static void edge_at(uint16_t t, bool high)
{
  mockTIM3.CNT = t;
  bus_level(high);
  DALI_PHY_RxEdgeIrq();
}

static void fire_cc1(void)
{
  if ((mockTIM3.DIER & TIM_DIER_CC1IE) != 0U)
  {
    mockTIM3.CNT = mockTIM3.CCR1;
    mockTIM3.SR = TIM_SR_CC1IF;
    DALI_PHY_TimerIrq();
  }
}

/* Sends 'bits' bits of 'data' with half bit durations te0 / te1 (asymmetric). */
static void send_frame(uint32_t data, uint8_t bits, uint16_t te0, uint16_t te1)
{
  bool levels[2U * 33U];   /* true = bus high */
  uint32_t n = 0U;
  uint32_t i;
  bool cur = true;
  uint16_t t = s_t;

  levels[n++] = false;     /* start bit '1' */
  levels[n++] = true;
  for (i = 0U; i < bits; i++)
  {
    const bool b = ((data >> (bits - 1U - i)) & 1U) != 0U;
    levels[n++] = !b;
    levels[n++] = b;
  }
  for (i = 0U; i < n; i++)
  {
    if (levels[i] != cur)
    {
      edge_at(t, levels[i]);
      cur = levels[i];
    }
    t = (uint16_t)(t + (((i & 1U) == 0U) ? te0 : te1));
  }
  if (!cur)
  {
    edge_at(t, true);      /* back to idle */
  }
  fire_cc1();              /* stop condition */
  s_t = (uint16_t)(t + 20000U);
}

static void test_rx(void)
{
  DALI_Frame_t f;

  printf("RX nominal 16 bit\n");
  send_frame(0xFE80U, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.bits, 16);
  CHECK_EQ(f.data, 0xFE80);

  printf("RX all zeros / all ones\n");
  send_frame(0x0000U, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.bits, 16);
  CHECK_EQ(f.data, 0x0000);
  send_frame(0xFFFFU, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.data, 0xFFFF);

  printf("RX timing limits (-10 %% / +10 %%, asymmetric)\n");
  send_frame(0xA5C3U, 16U, 375U, 375U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.data, 0xA5C3);
  send_frame(0x5A3CU, 16U, 458U, 458U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.data, 0x5A3C);
  send_frame(0x1234U, 16U, 350U, 480U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.data, 0x1234);

  printf("RX 24 bit control device frame\n");
  send_frame(0xFFFE12UL, 24U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.bits, 24);
  CHECK_EQ(f.data, 0xFFFE12);

  printf("RX timing violation -> error frame\n");
  send_frame(0x1234U, 16U, 250U, 250U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.bits, 0);
  fire_cc1();
  /* decoder must recover */
  send_frame(0x0102U, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  CHECK_EQ(f.bits, 16);
  CHECK_EQ(f.data, 0x0102);

  printf("RX spike on idle bus ignored\n");
  edge_at(s_t, false);
  edge_at((uint16_t)(s_t + 30U), true);
  s_t = (uint16_t)(s_t + 5000U);
  fire_cc1();
  CHECK(!DALI_PHY_GetFrame(&f));
}

static void test_tx(void)
{
  const uint8_t value = 0xA5U;
  bool active[40];
  uint16_t times[40];
  uint32_t n = 0U;
  uint32_t guard = 0U;
  uint16_t frameEnd;
  DALI_Frame_t f;
  uint32_t i;

  printf("TX backward frame 0xA5, settling time\n");
  send_frame(0xFF90U, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  /* last bit of 0x90 is '0': frame ends with the final rising edge */
  frameEnd = (uint16_t)(s_t - 20000U);
  mockTIM3.CNT = (uint16_t)(frameEnd + 2000U);
  DALI_PHY_SendBackward(value);
  CHECK((mockTIM3.DIER & TIM_DIER_CC2IE) != 0U);
  CHECK_EQ((uint16_t)(mockTIM3.CCR2 - frameEnd), 7000);

  while (((mockTIM3.DIER & TIM_DIER_CC2IE) != 0U) && (guard < 100U))
  {
    mockGPIOF.BSRR = 0U;
    mockGPIOF.BRR = 0U;
    mockTIM3.CNT = mockTIM3.CCR2;
    mockTIM3.SR = TIM_SR_CC2IF;
    times[n] = (uint16_t)mockTIM3.CNT;
    DALI_PHY_TimerIrq();
    if ((mockGPIOF.BSRR | mockGPIOF.BRR) != 0U)
    {
      /* default polarity: TX pin HIGH = bus active (low) */
      active[n] = (mockGPIOF.BSRR & 1U) != 0U;
      n++;
    }
    guard++;
  }
  CHECK_EQ(n, 19);    /* 18 half bits + release */
  CHECK(active[0] && !active[1]);   /* start bit */
  for (i = 0U; i < 8U; i++)
  {
    const bool bit = ((value >> (7U - i)) & 1U) != 0U;
    /* '1' = active (low) then idle (high) */
    CHECK_EQ(active[2U + (2U * i)], bit);
    CHECK_EQ(active[3U + (2U * i)], !bit);
  }
  CHECK(!active[18]);
  for (i = 1U; i < 18U; i++)
  {
    const uint16_t d = (uint16_t)(times[i] - times[i - 1U]);
    CHECK((d == 416U) || (d == 417U));
  }

  printf("TX too late -> dropped\n");
  send_frame(0xFF90U, 16U, 417U, 416U);
  CHECK(DALI_PHY_GetFrame(&f));
  mockTIM3.CNT = (uint16_t)(s_t - 20000U + 11000U);
  DALI_PHY_SendBackward(0x55U);
  CHECK((mockTIM3.DIER & TIM_DIER_CC2IE) == 0U);
}

int main(void)
{
  Mock_Reset();
  bus_level(true);
  DALI_PHY_Init();
  mockTIM3.CNT = 2000U;
  fire_cc1();          /* initial idle detection */

  test_rx();
  test_tx();

  printf("\ntest_phy: %d checks, %d failures\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}

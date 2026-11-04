/**
 ******************************************************************************
 * @file    shiftreg.c
 * @brief   SN74HCS596 bit-banged driver.
 *
 *          Timing at 48 MHz: one GPIO write takes >= 2 bus cycles (~42 ns);
 *          the extra NOPs keep every high/low phase above 100 ns, well over
 *          the SN74HCS596 minimum pulse width (~20 ns at 3.3 V).
 ******************************************************************************
 */
#include "shiftreg.h"
#include "bsp.h"

/**
 * @brief   Short delay (4 NOP, ~85 ns at 48 MHz) between shift register edges.
 * @details Together with the GPIO write latency every phase lasts > 100 ns,
 *          far above the SN74HCS596 minimum pulse width (~20 ns at 3.3 V).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: any.
 */
static void short_delay(void)
{
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

/* Public API (full description in shiftreg.h): shifts and latches one byte (bit n -> Qn). */
void ShiftReg_Write(uint8_t value)
{
  uint32_t i;

  BSP_SR_STCP_PORT->BRR = BSP_PIN_MASK(BSP_SR_STCP_PIN);
  /* MSB first: after 8 clocks bit 7 is in Q7 and bit 0 in Q0 */
  for (i = 0U; i < 8U; i++)
  {
    const uint32_t bit = ((uint32_t)value >> (7U - i)) & 1U;
    if (bit != 0U)
    {
      BSP_SR_DS_PORT->BSRR = BSP_PIN_MASK(BSP_SR_DS_PIN);
    }
    else
    {
      BSP_SR_DS_PORT->BRR = BSP_PIN_MASK(BSP_SR_DS_PIN);
    }
    short_delay();
    BSP_SR_SHCP_PORT->BSRR = BSP_PIN_MASK(BSP_SR_SHCP_PIN);
    short_delay();
    BSP_SR_SHCP_PORT->BRR = BSP_PIN_MASK(BSP_SR_SHCP_PIN);
  }
  short_delay();
  /* latch to the output register */
  BSP_SR_STCP_PORT->BSRR = BSP_PIN_MASK(BSP_SR_STCP_PIN);
  short_delay();
  BSP_SR_STCP_PORT->BRR = BSP_PIN_MASK(BSP_SR_STCP_PIN);
  BSP_SR_DS_PORT->BRR = BSP_PIN_MASK(BSP_SR_DS_PIN);
}

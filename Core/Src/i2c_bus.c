/**
 ******************************************************************************
 * @file    i2c_bus.c
 * @brief   Blocking I2C1 master driver (register level).
 ******************************************************************************
 */
#include "i2c_bus.h"
#include <stddef.h>
#include "bsp.h"

/*
 * TIMINGR for I2CCLK = 8 MHz (HSI), standard mode 100 kHz (RM0360 tab. 83).
 * 100 kHz is used because of the 10 kOhm pull-ups (R81 / R84) and the extra
 * capacitance of the NFC board connector.
 */
#define I2C_TIMINGR_100KHZ     (0x2000090EUL)
#define I2C_TIMEOUT_MS         (5U)

/**
 * @brief   Waits until an I2C ISR flag is set, a NACK is received or the
 * @details transfer timeout expires.
 * @param[in] flag     ISR flag mask to wait for (TXIS, RXNE, TC, STOPF).
 * @param[in] startMs  Start time of the whole transfer (BSP_GetTickMs()).
 * @return  true when the flag is set, false on NACK or timeout (I2C_TIMEOUT_MS).
 * @note    Context: main loop only.
 */
static bool wait_flag(uint32_t flag, uint32_t startMs)
{
  bool ok = true;
  while ((I2C1->ISR & flag) == 0U)
  {
    if ((I2C1->ISR & I2C_ISR_NACKF) != 0U)
    {
      ok = false;
      break;
    }
    if ((BSP_GetTickMs() - startMs) > I2C_TIMEOUT_MS)
    {
      ok = false;
      break;
    }
  }
  return ok;
}

/**
 * @brief   Software reset of I2C1 (PE cleared then set again).
 * @details Clears the internal state machine and the status flags after a bus
 *          error or a timeout. The configuration registers are preserved.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only. PE must stay low for >= 3 APB cycles.
 */
static void peripheral_reset(void)
{
  I2C1->CR1 &= ~I2C_CR1_PE;
  __NOP();
  __NOP();
  __NOP();
  I2C1->CR1 |= I2C_CR1_PE;
}

/* Public API (full description in i2c_bus.h): I2C1 master at 100 kHz. */
void I2C_BusInit(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
  I2C1->CR1 = 0U;
  I2C1->TIMINGR = I2C_TIMINGR_100KHZ;
  I2C1->CR1 = I2C_CR1_PE;
}

/**
 * @brief   Terminates a failed transfer and classifies the error.
 * @details On NACK a STOP is generated (if not automatic) and awaited; on any
 *          other error the peripheral is reset. The error flags are cleared.
 * @param[in] startMs  Start time of the transfer, bounds the STOP wait.
 * @return  I2C_BUS_NACK if the slave did not acknowledge, I2C_BUS_TIMEOUT otherwise.
 * @note    Context: main loop only.
 */
static I2C_BusStatus_t finish_error(uint32_t startMs)
{
  I2C_BusStatus_t st = I2C_BUS_TIMEOUT;

  if ((I2C1->ISR & I2C_ISR_NACKF) != 0U)
  {
    st = I2C_BUS_NACK;
    /* Hardware generates STOP after NACK only in AUTOEND mode */
    if ((I2C1->CR2 & I2C_CR2_AUTOEND) == 0U)
    {
      I2C1->CR2 |= I2C_CR2_STOP;
    }
    while (((I2C1->ISR & I2C_ISR_STOPF) == 0U) &&
           ((BSP_GetTickMs() - startMs) <= (2U * I2C_TIMEOUT_MS)))
    {
      /* wait stop */
    }
  }
  I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
  if (st != I2C_BUS_NACK)
  {
    peripheral_reset();
  }
  return st;
}

/* Public API (full description in i2c_bus.h): write / read / write + read transfer. */
I2C_BusStatus_t I2C_BusTransfer(uint8_t addr7,
                                const uint8_t *wdata, uint8_t wlen,
                                uint8_t *rdata, uint8_t rlen)
{
  const uint32_t startMs = BSP_GetTickMs();
  const uint32_t sadd = ((uint32_t)addr7 << 1U) & I2C_CR2_SADD;
  I2C_BusStatus_t st = I2C_BUS_OK;
  uint8_t i;

  if (((wlen != 0U) && (wdata == NULL)) || ((rlen != 0U) && (rdata == NULL)) ||
      ((wlen == 0U) && (rlen == 0U)))
  {
    st = I2C_BUS_PARAM;
  }
  else if ((I2C1->ISR & I2C_ISR_BUSY) != 0U)
  {
    peripheral_reset();
    st = I2C_BUS_TIMEOUT;
  }
  else
  {
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;

    if (wlen != 0U)
    {
      uint32_t cr2 = sadd | ((uint32_t)wlen << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
      if (rlen == 0U)
      {
        cr2 |= I2C_CR2_AUTOEND;
      }
      I2C1->CR2 = cr2;
      for (i = 0U; (i < wlen) && (st == I2C_BUS_OK); i++)
      {
        if (!wait_flag(I2C_ISR_TXIS, startMs))
        {
          st = finish_error(startMs);
        }
        else
        {
          I2C1->TXDR = wdata[i];
        }
      }
      if (st == I2C_BUS_OK)
      {
        const uint32_t endFlag = (rlen == 0U) ? I2C_ISR_STOPF : I2C_ISR_TC;
        if (!wait_flag(endFlag, startMs))
        {
          st = finish_error(startMs);
        }
      }
    }

    if ((st == I2C_BUS_OK) && (rlen != 0U))
    {
      I2C1->CR2 = sadd | I2C_CR2_RD_WRN | ((uint32_t)rlen << I2C_CR2_NBYTES_Pos) |
                  I2C_CR2_AUTOEND | I2C_CR2_START;
      for (i = 0U; (i < rlen) && (st == I2C_BUS_OK); i++)
      {
        if (!wait_flag(I2C_ISR_RXNE, startMs))
        {
          st = finish_error(startMs);
        }
        else
        {
          rdata[i] = (uint8_t)I2C1->RXDR;
        }
      }
      if ((st == I2C_BUS_OK) && (!wait_flag(I2C_ISR_STOPF, startMs)))
      {
        st = finish_error(startMs);
      }
    }
    I2C1->ICR = I2C_ICR_STOPCF;
  }
  return st;
}

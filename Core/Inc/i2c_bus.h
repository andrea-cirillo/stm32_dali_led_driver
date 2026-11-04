/**
 ******************************************************************************
 * @file    i2c_bus.h
 * @brief   Blocking I2C1 master driver (register level, polling, timeouts).
 *          Shared by the M24C04 EEPROM and the NTAG 5 link. Only to be used
 *          from the main loop context (not reentrant, not interrupt safe).
 ******************************************************************************
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>

/** Result of an I2C transfer. */
typedef enum
{
  I2C_BUS_OK = 0,
  I2C_BUS_NACK,      /**< address or data not acknowledged (busy / absent device) */
  I2C_BUS_TIMEOUT,   /**< bus stuck or peripheral error, peripheral reset          */
  I2C_BUS_PARAM      /**< invalid arguments, nothing transferred                  */
} I2C_BusStatus_t;

/**
 * @brief   Initialises I2C1 as master at 100 kHz.
 * @details Enables the I2C1 clock and programs TIMINGR for an 8 MHz kernel
 *          clock (HSI). The GPIOs (PA9/PA10, AF4, open drain) are configured
 *          by BSP_Init().
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_Init() already called.
 * @note    Context: main, once.
 */
void I2C_BusInit(void);

/**
 * @brief   Performs a write, a read, or a combined write + read transfer.
 * @details - wlen > 0, rlen = 0: START, address+W, wlen bytes, STOP.
 *          - wlen = 0, rlen > 0: START, address+R, rlen bytes, STOP.
 *          - both > 0: write phase, repeated START, read phase, STOP
 *            (typical "set address pointer then read" access).
 *          Every wait is bounded by a 5 ms timeout. On NACK a STOP is sent;
 *          on timeout / bus error the peripheral is reset.
 * @param[in]  addr7  7-bit slave address (not shifted).
 * @param[in]  wdata  Bytes to write; may be NULL only if wlen = 0.
 * @param[in]  wlen   Number of bytes to write (0..255).
 * @param[out] rdata  Buffer for the read bytes; may be NULL only if rlen = 0.
 * @param[in]  rlen   Number of bytes to read (0..255).
 * @return  I2C_BUS_OK on success, otherwise the error cause
 *          (see I2C_BusStatus_t). On error the content of rdata is undefined.
 * @note    Context: main loop only. Blocking: about 0.1 ms per byte.
 */
I2C_BusStatus_t I2C_BusTransfer(uint8_t addr7,
                                const uint8_t *wdata, uint8_t wlen,
                                uint8_t *rdata, uint8_t rlen);

#endif /* I2C_BUS_H */

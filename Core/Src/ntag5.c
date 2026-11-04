/**
 ******************************************************************************
 * @file    ntag5.c
 * @brief   Minimal NTAG 5 link I2C driver.
 *
 *          While the RF interface owns the memory (arbiter in normal mode) the
 *          NTAG 5 does not acknowledge I2C accesses: the caller simply retries
 *          later, nothing is blocking.
 ******************************************************************************
 */
#include "ntag5.h"
#include <stddef.h>
#include "i2c_bus.h"
#include "app_config.h"

/* Public API (full description in ntag5.h): reads NTAG 5 memory blocks. */
bool NTAG5_ReadBlocks(uint16_t block_addr, uint8_t *buf, uint8_t len)
{
  bool ok = false;

  if ((buf != NULL) && (len != 0U) && ((len % NTAG5_BLOCK_SIZE) == 0U))
  {
    const uint8_t tx[2] = { (uint8_t)(block_addr >> 8U), (uint8_t)block_addr };
    ok = (I2C_BusTransfer(CFG_NTAG5_I2C_ADDR, tx, 2U, buf, len) == I2C_BUS_OK);
  }
  return ok;
}

/* Public API (full description in ntag5.h): masked session register write. */
bool NTAG5_WriteReg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value)
{
  const uint8_t tx[5] = { (uint8_t)(reg_addr >> 8U), (uint8_t)reg_addr, reg_byte, mask, value };
  return (I2C_BusTransfer(CFG_NTAG5_I2C_ADDR, tx, 5U, NULL, 0U) == I2C_BUS_OK);
}

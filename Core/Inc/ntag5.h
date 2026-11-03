/**
 ******************************************************************************
 * @file    ntag5.h
 * @brief   Minimal bare-metal I2C driver for the NXP NTAG 5 link (NTP5332,
 *          OM2NTP5332 demo board). Ported from stm32_nfc_ntag5 (only the
 *          functions needed by this product are kept).
 *
 *          I2C memory map (block addresses, 1 block = 4 bytes):
 *            0000h..01FFh  user EEPROM
 *            10A0h..10AFh  session registers
 ******************************************************************************
 */
#ifndef NTAG5_H
#define NTAG5_H

#include <stdint.h>
#include <stdbool.h>

#define NTAG5_BLOCK_SIZE           (4U)
#define NTAG5_REG_ED_CONFIG        (0x10A8U)
#define NTAG5_ED_MASK              (0x0FU)
#define NTAG5_ED_WRITE_CMD         (0x08U)  /* ED active on every RF WRITE */

/** @brief  Reads len bytes (multiple of 4) starting at block_addr. */
bool NTAG5_ReadBlocks(uint16_t block_addr, uint8_t *buf, uint8_t len);

/** @brief  Writes one byte of a session register (masked). */
bool NTAG5_WriteReg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value);

#endif /* NTAG5_H */

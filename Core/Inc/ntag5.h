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

/**
 * @brief   Reads consecutive 4-byte blocks of the NTAG 5 memory.
 * @details I2C sequence: write the 2-byte block address (MSB first), then
 *          repeated START and read 'len' bytes. While the RF side owns the
 *          memory (arbiter in normal mode) the tag does not acknowledge: the
 *          call fails and the caller simply retries later.
 * @param[in]  block_addr  First block address (e.g. 0x0000..0x01FF user memory).
 * @param[out] buf         Destination buffer of at least len bytes.
 * @param[in]  len         Number of bytes, multiple of 4 (4..252).
 * @return  true on success, false on invalid arguments, NACK or I2C error.
 * @note    Context: main loop only (blocking I2C).
 */
bool NTAG5_ReadBlocks(uint16_t block_addr, uint8_t *buf, uint8_t len);

/**
 * @brief   Writes one byte of a session / configuration register (masked).
 * @details I2C frame: address MSB, address LSB, register byte index, mask,
 *          data. Only the bits set in 'mask' are modified.
 * @param[in] reg_addr  Register block address (e.g. NTAG5_REG_ED_CONFIG).
 * @param[in] reg_byte  Byte index inside the 4-byte register block (0..3).
 * @param[in] mask      Bits to be modified.
 * @param[in] value     New value of the masked bits.
 * @return  true if the frame was acknowledged, false otherwise (tag absent,
 *          RF arbitration or I2C error).
 * @note    Context: main loop only. Session registers are volatile: they
 *          must be written again after every tag power-up.
 */
bool NTAG5_WriteReg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value);

#endif /* NTAG5_H */

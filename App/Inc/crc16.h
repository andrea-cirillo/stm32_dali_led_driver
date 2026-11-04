/**
 ******************************************************************************
 * @file    crc16.h
 * @brief   CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection,
 *          no final XOR). Check value for "123456789" = 0x29B1.
 ******************************************************************************
 */
#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

/**
 * @brief   Computes the CRC-16/CCITT-FALSE of a buffer.
 * @details Bitwise implementation (no table) to save flash. Used for the
 *          EEPROM records, the NFC variant record and the identification
 *          number derivation.
 * @param[in] data  Input bytes (may be NULL only if len = 0).
 * @param[in] len   Number of bytes.
 * @return  16-bit CRC (0xFFFF for an empty buffer).
 * @note    Context: any, reentrant.
 */
uint16_t CRC16_Compute(const uint8_t *data, uint32_t len);

#endif /* CRC16_H */

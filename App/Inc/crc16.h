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

uint16_t CRC16_Compute(const uint8_t *data, uint32_t len);

#endif /* CRC16_H */

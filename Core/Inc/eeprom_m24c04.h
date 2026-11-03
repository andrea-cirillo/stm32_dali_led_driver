/**
 ******************************************************************************
 * @file    eeprom_m24c04.h
 * @brief   ST M24C04 (4 Kbit, 512 bytes, 16-byte pages) driver over I2C1.
 *          E1 = E2 = GND: device select 0x50 (block 0) / 0x51 (block 1).
 ******************************************************************************
 */
#ifndef EEPROM_M24C04_H
#define EEPROM_M24C04_H

#include <stdint.h>
#include <stdbool.h>

#define M24C04_SIZE            (512U)
#define M24C04_PAGE_SIZE       (16U)

bool M24C04_Read(uint16_t addr, uint8_t *data, uint16_t len);

/** @brief  Page-aware write with ACK polling of the internal write cycle. */
bool M24C04_Write(uint16_t addr, const uint8_t *data, uint16_t len);

#endif /* EEPROM_M24C04_H */

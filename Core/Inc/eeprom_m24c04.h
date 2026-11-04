/**
 ******************************************************************************
 * @file    eeprom_m24c04.h
 * @brief   ST M24C04 (4 Kbit, 512 bytes, 16-byte pages) driver over I2C1.
 *          E1 = E2 = GND: device select 0x50 (block 0) / 0x51 (block 1),
 *          the address bit A8 selects the 256-byte block.
 ******************************************************************************
 */
#ifndef EEPROM_M24C04_H
#define EEPROM_M24C04_H

#include <stdint.h>
#include <stdbool.h>

#define M24C04_SIZE            (512U)
#define M24C04_PAGE_SIZE       (16U)

/**
 * @brief   Reads a contiguous area of the EEPROM.
 * @details The request is split at the 256-byte block boundary and in
 *          chunks of at most 128 bytes; each chunk is retried up to 3 times.
 * @param[in]  addr  First byte address (0..511).
 * @param[out] data  Destination buffer of at least len bytes.
 * @param[in]  len   Number of bytes (1..512, addr + len <= 512).
 * @return  true if all the bytes were read, false on invalid arguments or
 *          on an I2C error (the buffer content is then undefined).
 * @note    Context: main loop only (blocking I2C).
 */
bool M24C04_Read(uint16_t addr, uint8_t *data, uint16_t len);

/**
 * @brief   Writes a contiguous area of the EEPROM.
 * @details The request is split at the 16-byte page boundaries. After every
 *          page the internal write cycle (tW <= 5 ms) is waited for by ACK
 *          polling (10 ms timeout). Each page is retried up to 3 times.
 * @param[in] addr  First byte address (0..511).
 * @param[in] data  Source buffer of at least len bytes.
 * @param[in] len   Number of bytes (1..512, addr + len <= 512).
 * @return  true if all pages were written and acknowledged, false on
 *          invalid arguments or I2C error (the area may be partially written).
 * @note    Context: main loop only. Blocking: ~6 ms per page.
 *          Endurance: 4 million cycles per byte (25 degC).
 */
bool M24C04_Write(uint16_t addr, const uint8_t *data, uint16_t len);

#endif /* EEPROM_M24C04_H */

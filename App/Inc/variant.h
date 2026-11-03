/**
 ******************************************************************************
 * @file    variant.h
 * @brief   Lamp variant management: current limitation resistor selection
 *          (SN74HCS596 Q0..Q5, see Docs/Schematics/Table_RvsQx.pdf), storage
 *          in the M24C04 EEPROM and configuration through the NTAG 5 tag.
 *
 *          Variant record (8 bytes, used both in the NTAG 5 user memory at
 *          block CFG_NFC_VARIANT_BLOCK and in the EEPROM):
 *            byte 0..3  identification code "DLVR" (0x44 0x4C 0x56 0x52)
 *            byte 4     record format version (0x01)
 *            byte 5     variant index (0 .. CFG_VARIANT_COUNT - 1)
 *            byte 6..7  CRC-16/CCITT-FALSE of bytes 0..5, MSB first
 *
 *          The variant is written to the EEPROM as soon as a new valid record
 *          is found on the tag, but the resistor network is switched only at
 *          boot (Variant_ApplyAtBoot()).
 ******************************************************************************
 */
#ifndef VARIANT_H
#define VARIANT_H

#include <stdint.h>
#include <stdbool.h>

#define VARIANT_RECORD_SIZE    (8U)

/**
 * @brief  Boot sequence (power stage must still be OFF): reads the tag and
 *         the EEPROM, stores a new tag variant, writes the shift register.
 */
void Variant_ApplyAtBoot(void);

/** @brief  Variant applied at boot. */
uint8_t Variant_GetActive(void);

/** @brief  Variant stored in EEPROM (applied at next boot). */
uint8_t Variant_GetStored(void);

/** @brief  Main loop service: NFC event / periodic tag check. */
void Variant_Service(void);

/** @brief  NTAG 5 ED pin interrupt (EXTI0). */
void Variant_NfcEventIrq(void);

/** @brief  Record codec (exported for unit tests / EOL tools). */
bool Variant_DecodeRecord(const uint8_t rec[VARIANT_RECORD_SIZE], uint8_t *variant);
void Variant_EncodeRecord(uint8_t variant, uint8_t rec[VARIANT_RECORD_SIZE]);

#endif /* VARIANT_H */

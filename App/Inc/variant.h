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
 * @brief   Selects and applies the lamp variant at boot.
 * @details 1. Writes the default (lowest current) variant to the shift
 *             register, so the outputs are no longer random.
 *          2. Loads the stored variant from EEPROM copy A, else copy B.
 *          3. Reads the NTAG 5 record (3 attempts): if valid and different
 *             from the stored one, it is written to the EEPROM.
 *          4. Writes the resulting variant (default if none is valid) to
 *             the shift register. This is the only point where the current
 *             limitation network is switched.
 *          5. Configures the NTAG 5 ED pin on RF write commands and enables
 *             EXTI0 (PA0, falling edge).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     I2C_BusInit() called; power stage still OFF.
 * @note    Context: main, once. Blocking (a few ms, more if the tag NACKs).
 */
void Variant_ApplyAtBoot(void);

/**
 * @brief   Returns the variant applied at boot.
 * @par     Parameters
 *          None.
 * @return  Variant index 0 .. CFG_VARIANT_COUNT - 1.
 * @note    Context: any.
 */
uint8_t Variant_GetActive(void);

/**
 * @brief   Returns the variant stored in the EEPROM (applied at next boot).
 * @par     Parameters
 *          None.
 * @return  Variant index 0 .. CFG_VARIANT_COUNT - 1 (default if none valid).
 * @note    Context: any.
 */
uint8_t Variant_GetStored(void);

/**
 * @brief   NFC configuration task, to be called continuously by the main loop.
 * @details Reads the tag CFG_NFC_EVENT_SETTLE_MS after the last ED event,
 *          and anyway every CFG_NFC_POLL_PERIOD_MS. A valid record with a
 *          new variant is stored in the EEPROM. The shift register is NOT
 *          changed: the new variant becomes active at the next power-up.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only.
 */
void Variant_Service(void);

/**
 * @brief   NTAG 5 ED pin event.
 * @details Records the event time; the tag is read later by
 *          Variant_Service() when the RF activity has settled.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: EXTI0 interrupt (via APP_NfcEventIrq()).
 */
void Variant_NfcEventIrq(void);

/**
 * @brief   Validates and decodes a variant record.
 * @param[in]  rec      8-byte record.
 * @param[out] variant  Decoded variant index; written only on success.
 * @return  true if identification code, format version, range
 *          (< CFG_VARIANT_COUNT) and CRC are all correct, false otherwise.
 * @note    Context: any, reentrant.
 */
bool Variant_DecodeRecord(const uint8_t rec[VARIANT_RECORD_SIZE], uint8_t *variant);

/**
 * @brief   Builds a variant record (identification code, version, CRC).
 * @param[in]  variant  Variant index to encode (not range checked).
 * @param[out] rec      8-byte destination buffer.
 * @par     Return value
 *          None (void).
 * @note    Context: any, reentrant. Can be used by EOL tools / tests.
 */
void Variant_EncodeRecord(uint8_t variant, uint8_t rec[VARIANT_RECORD_SIZE]);

#endif /* VARIANT_H */

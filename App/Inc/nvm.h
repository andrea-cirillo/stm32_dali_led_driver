/**
 ******************************************************************************
 * @file    nvm.h
 * @brief   Non-volatile storage layout in the M24C04 EEPROM (512 bytes).
 *
 *   0x000..0x03F  DALI persistent variables, copy A  (seq + data + CRC16)
 *   0x040..0x07F  DALI persistent variables, copy B
 *   0x080..0x0F4  Memory bank 1 (IEC 62386-251) locations 0x03..0x77
 *   0x0F6..0x0F7  Memory bank 1 "initialised" marker
 *   0x100..0x107  Lamp variant record, copy A  (same format as the NFC block)
 *   0x110..0x117  Lamp variant record, copy B
 *   0x120..0x1DF  Operating counters, 4 rotating slots of 48 bytes
 *                 (seq + data + CRC16, newest valid slot wins)
 *
 *   Record format: [seq LSB][seq MSB][data ...][CRC16 MSB][CRC16 LSB], the
 *   CRC covering seq + data. Every record is written data first and CRC
 *   last into the oldest slot: an interrupted write leaves an invalid record
 *   and the previous copy is still used.
 ******************************************************************************
 */
#ifndef NVM_H
#define NVM_H

#include <stdint.h>
#include <stdbool.h>

#define NVM_MB1_SIZE           (0x75U)  /* locations 0x03..0x77 */

/** DALI persistent variables (byte members only: no padding). */
typedef struct
{
  uint8_t shortAddress;       /**< 0..63, 0xFF = MASK (no address)       */
  uint8_t powerOnLevel;       /**< 0..254, 0xFF = last light level       */
  uint8_t systemFailureLevel; /**< 0..254, 0xFF = no change              */
  uint8_t minLevel;           /**< PHM..maxLevel                         */
  uint8_t maxLevel;           /**< minLevel..254                         */
  uint8_t fadeRate;           /**< 1..15                                 */
  uint8_t fadeTime;           /**< 0..15                                 */
  uint8_t extFadeTime;        /**< multiplier << 4 | base                */
  uint8_t operatingMode;      /**< 0 = standard                          */
  uint8_t lastLightLevel;     /**< level restored when powerOnLevel=MASK */
  uint8_t dimmingCurve;       /**< IEC 62386-207: 0 log, 1 linear        */
  uint8_t fastFadeTime;       /**< IEC 62386-207: 0..27 (x 25 ms)        */
  uint8_t randomAddress[3];   /**< H, M, L                               */
  uint8_t groups[2];          /**< groups 0-7, 8-15                      */
  uint8_t scene[16];          /**< 0..254, 0xFF = MASK                   */
} NVM_Gear_t;

/** Operating counters (IEC 62386-252 / -253). */
typedef struct
{
  uint32_t gearOperatingTimeS;   /**< control gear operating time [s]     */
  uint32_t gearStartCounter;     /**< control gear power-ups              */
  uint32_t lsOnTimeS;            /**< light source on time [s]            */
  uint32_t lsOnTimeResS;         /**< resettable light source on time [s] */
  uint32_t lsStartCounter;       /**< light source switch-on events       */
  uint32_t lsStartCounterRes;    /**< resettable light source starts      */
  uint32_t activeEnergyWh;       /**< estimated active energy [Wh]        */
  uint8_t  gearFailureCounter;   /**< control gear overall failures       */
  uint8_t  lsFailureCounter;     /**< light source overall failures       */
  uint8_t  lsShortCounter;       /**< light source short circuits         */
  uint8_t  lsOpenCounter;        /**< light source open circuits          */
} NVM_Counters_t;

/**
 * @brief   Loads the newest valid copy of the DALI persistent variables.
 * @param[out] gear  Destination structure; written only if a valid copy
 *                   exists.
 * @return  true if a valid record (CRC OK) was found, false if both copies
 *          are blank or corrupted (the caller must apply the defaults).
 * @note    Context: main loop only (blocking I2C).
 */
bool NVM_LoadGear(NVM_Gear_t *gear);

/**
 * @brief   Saves the DALI persistent variables into the oldest copy.
 * @param[in] gear  Variables to store.
 * @return  true on success, false on EEPROM error (the previous copy
 *          remains valid).
 * @note    Context: main loop only. Duration ~20 ms.
 */
bool NVM_SaveGear(const NVM_Gear_t *gear);

/**
 * @brief   Loads the newest valid operating counters record.
 * @param[out] cnt  Destination structure; written only if a valid slot exists.
 * @return  true if a valid record was found, false otherwise.
 * @note    Context: main loop only.
 */
bool NVM_LoadCounters(NVM_Counters_t *cnt);

/**
 * @brief   Saves the operating counters into the next rotating slot.
 * @param[in] cnt  Counters to store.
 * @return  true on success, false on EEPROM error.
 * @note    Context: main loop only. Duration ~20 ms. With 4 slots and a
 *          15 minute period each byte is written once per hour.
 */
bool NVM_SaveCounters(const NVM_Counters_t *cnt);

/**
 * @brief   Loads the memory bank 1 content (locations 0x03..0x77).
 * @param[out] data  Destination buffer, NVM_MB1_SIZE bytes.
 * @return  true if the area was initialised before (marker present) and
 *          read correctly, false otherwise (caller sets the defaults).
 * @note    Context: main loop only.
 */
bool NVM_LoadMb1(uint8_t data[NVM_MB1_SIZE]);

/**
 * @brief   Saves the memory bank 1 content and its "initialised" marker.
 * @param[in] data  Source buffer, NVM_MB1_SIZE bytes.
 * @return  true on success, false on EEPROM error.
 * @note    Context: main loop only. Duration ~50 ms (8 pages).
 */
bool NVM_SaveMb1(const uint8_t data[NVM_MB1_SIZE]);

/**
 * @brief   Reads one copy of the lamp variant record (raw, not validated).
 * @param[out] rec   Destination buffer, 8 bytes.
 * @param[in]  copy  0 = copy A, 1 = copy B (only bit 0 is used).
 * @return  true if the bytes were read, false on EEPROM error. Validation
 *          is done by Variant_DecodeRecord().
 * @note    Context: main loop only.
 */
bool NVM_LoadVariantRecord(uint8_t rec[8], uint8_t copy);

/**
 * @brief   Writes the lamp variant record into both copies (A then B).
 * @param[in] rec  Record to store, 8 bytes.
 * @return  true if both copies were written, false otherwise.
 * @note    Context: main loop only.
 */
bool NVM_SaveVariantRecord(const uint8_t rec[8]);

#endif /* NVM_H */

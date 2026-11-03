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
 *   Every multi-byte record is written data first and CRC last: an
 *   interrupted write leaves an invalid record and the previous copy is used.
 ******************************************************************************
 */
#ifndef NVM_H
#define NVM_H

#include <stdint.h>
#include <stdbool.h>

#define NVM_MB1_SIZE           (0x75U)  /* locations 0x03..0x77 */

/* DALI persistent variables (byte members only: no padding) */
typedef struct
{
  uint8_t shortAddress;
  uint8_t powerOnLevel;
  uint8_t systemFailureLevel;
  uint8_t minLevel;
  uint8_t maxLevel;
  uint8_t fadeRate;
  uint8_t fadeTime;
  uint8_t extFadeTime;        /* multiplier << 4 | base               */
  uint8_t operatingMode;
  uint8_t lastLightLevel;
  uint8_t dimmingCurve;       /* IEC 62386-207                         */
  uint8_t fastFadeTime;       /* IEC 62386-207                         */
  uint8_t randomAddress[3];   /* H, M, L                               */
  uint8_t groups[2];          /* groups 0-7, 8-15                      */
  uint8_t scene[16];
} NVM_Gear_t;

/* Operating counters (IEC 62386-252 / -253) */
typedef struct
{
  uint32_t gearOperatingTimeS;
  uint32_t gearStartCounter;
  uint32_t lsOnTimeS;
  uint32_t lsOnTimeResS;
  uint32_t lsStartCounter;
  uint32_t lsStartCounterRes;
  uint32_t activeEnergyWh;
  uint8_t  gearFailureCounter;
  uint8_t  lsFailureCounter;
  uint8_t  lsShortCounter;
  uint8_t  lsOpenCounter;
} NVM_Counters_t;

bool NVM_LoadGear(NVM_Gear_t *gear);
bool NVM_SaveGear(const NVM_Gear_t *gear);

bool NVM_LoadCounters(NVM_Counters_t *cnt);
bool NVM_SaveCounters(const NVM_Counters_t *cnt);

/** @brief  Loads MB1 content; false if never initialised (caller sets defaults). */
bool NVM_LoadMb1(uint8_t data[NVM_MB1_SIZE]);
bool NVM_SaveMb1(const uint8_t data[NVM_MB1_SIZE]);

bool NVM_LoadVariantRecord(uint8_t rec[8], uint8_t copy);
bool NVM_SaveVariantRecord(const uint8_t rec[8]);

#endif /* NVM_H */

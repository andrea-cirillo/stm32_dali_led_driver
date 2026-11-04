/**
 ******************************************************************************
 * @file    dali_membank.h
 * @brief   DALI memory banks:
 *            bank 0    IEC 62386-102 identification (ROM)
 *            bank 1    IEC 62386-251 luminaire information (lockable NVM)
 *            bank 202  IEC 62386-252 active energy / power (latchable)
 *            bank 205  IEC 62386-253 control gear diagnostics (lock / latch)
 *            bank 206  IEC 62386-253 light source diagnostics (lock / latch)
 *
 *          Called from the DALI command handler (SysTick context), except
 *          MB_Init() and MB_Service() (main loop).
 ******************************************************************************
 */
#ifndef DALI_MEMBANK_H
#define DALI_MEMBANK_H

#include <stdint.h>
#include <stdbool.h>

#define MB_LAST_BANK           (206U)

/**
 * @brief   Initialises the memory banks.
 * @details Derives the 64-bit identification number of bank 0 from the
 *          STM32 UID (or uses CFG_MB0_ID_NUMBER), loads bank 1 from the
 *          EEPROM and, if it was never initialised, writes the factory
 *          defaults of app_config.h. Lock / latch bytes start at 0xFF.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once.
 */
void MB_Init(void);

/**
 * @brief   Persists memory bank 1 after a write burst.
 * @details When bank 1 was modified and no further write arrived for
 *          1 s, a snapshot of the RAM mirror is written to the EEPROM (one
 *          write for a whole burst limits wear).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only.
 */
void MB_Service(void);

/**
 * @brief   Tells whether a memory bank exists.
 * @param[in] bank  Memory bank number (DTR1).
 * @return  true for banks 0, 1, 202, 205, 206.
 * @note    Context: SysTick.
 */
bool MB_IsImplemented(uint8_t bank);

/**
 * @brief   Returns the last accessible location of a bank.
 * @param[in] bank  Implemented memory bank number.
 * @return  0x1A (bank 0), 0x77 (1), 0x0F (202), 0x1C (205), 0x20 (206).
 *          Unspecified for banks that are not implemented.
 * @note    Context: SysTick.
 */
uint8_t MB_LastLocation(uint8_t bank);

/**
 * @brief   Reads one memory bank location (READ MEMORY LOCATION).
 * @details Banks 202/205/206 return the latched snapshot when their latch
 *          byte (location 0x02) is 0xAA, otherwise live values. Values not
 *          implemented read as MASK (0xFF bytes); reserved locations
 *          (e.g. 0x01) give no answer.
 * @param[in]  bank   Memory bank number (DTR1).
 * @param[in]  loc    Location (DTR0).
 * @param[out] value  Location content; written only when true is returned.
 * @return  true if the location is implemented (the gear answers),
 *          false otherwise (no answer).
 * @note    Context: SysTick.
 */
bool MB_Read(uint8_t bank, uint8_t loc, uint8_t *value);

/**
 * @brief   Writes one memory bank location (WRITE MEMORY LOCATION).
 * @details Writable locations:
 *          - 0x02 of banks 1/202/205/206 (lock / latch byte): 0x55
 *            unlocks, 0xAA latches a snapshot (202/205/206), other values
 *            release the latch;
 *          - bank 1 locations 0x03..0x77, only while unlocked (persisted
 *            later by MB_Service());
 *          - bank 206 resettable counters (0x04..0x06, 0x0A..0x0D).
 * @param[in] bank   Memory bank number (DTR1).
 * @param[in] loc    Location (DTR0).
 * @param[in] value  Byte to write.
 * @return  true if written (the gear answers with the value), false if the
 *          location is read-only, locked or not implemented.
 * @note    Context: SysTick. writeEnableState is checked by the caller.
 */
bool MB_Write(uint8_t bank, uint8_t loc, uint8_t value);

/**
 * @brief   Resets memory bank content (RESET MEMORY BANK).
 * @details bank 0: all resettable banks; bank 1: factory defaults (only if
 *          unlocked); bank 206: resettable light source counters (only if
 *          unlocked). Other banks have no resettable content.
 * @param[in] bank  DTR0 value of the command.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick.
 */
void MB_Reset(uint8_t bank);

#endif /* DALI_MEMBANK_H */

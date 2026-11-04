/**
 ******************************************************************************
 * @file    opdata.h
 * @brief   Operating data: control gear / light source operating time and
 *          start counters, failure counters (IEC 62386-253) and estimated
 *          active energy (IEC 62386-252). Persisted every
 *          CFG_COUNTERS_SAVE_PERIOD_MS and after every fault.
 ******************************************************************************
 */
#ifndef OPDATA_H
#define OPDATA_H

#include <stdint.h>
#include "nvm.h"

#define OPDATA_CNT_LS_STARTS_RES   (0U)   /**< resettable light source start counter */
#define OPDATA_CNT_LS_ONTIME_RES   (1U)   /**< resettable light source on time       */

/**
 * @brief   Loads the counters and records a new control gear start.
 * @details Loads the newest valid counters record from the EEPROM (all
 *          counters 0 if none), increments the control gear start counter
 *          and saves it immediately.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     I2C_BusInit() and Light_Init() already called.
 * @note    Context: main, once.
 */
void OpData_Init(void);

/**
 * @brief   Operating data task, to be called continuously by the main loop.
 * @details Every second: control gear operating time +1 s, light source on
 *          time +1 s while lit, light source start counters + number of new
 *          switch-on events, active energy += estimated input power (1 Wh
 *          = 3 600 000 mWs). Saves the counters every
 *          CFG_COUNTERS_SAVE_PERIOD_MS or when a save was requested.
 *          All counters saturate at their maximum value (0x..FD).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only (EEPROM write ~20 ms when saving).
 */
void OpData_Service(void);

/**
 * @brief   Records a confirmed light source fault.
 * @details Increments the overall light source failure counter and the
 *          short / open circuit counters according to the bits, then
 *          requests an immediate save.
 * @param[in] failureBits  MON_FAIL_xxx bits of the fault.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop (led_monitor).
 */
void OpData_OnLightSourceFault(uint8_t failureBits);

/**
 * @brief   Read-only access to the live counters.
 * @par     Parameters
 *          None.
 * @return  Pointer to the counters structure (never NULL). Each field is a
 *          32-bit or 8-bit value updated atomically; for a consistent
 *          multi-field snapshot use the memory bank latch.
 * @note    Context: any (memory banks in SysTick context).
 */
const NVM_Counters_t *OpData_Get(void);

/**
 * @brief   Resets the resettable light source counters (start counter and
 *          on time of memory bank 206) and requests a save.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick (RESET MEMORY BANK).
 */
void OpData_ResetResettable(void);

/**
 * @brief   Writes one byte of a resettable counter (WRITE MEMORY LOCATION
 *          into memory bank 206).
 * @param[in] counter  OPDATA_CNT_LS_STARTS_RES or OPDATA_CNT_LS_ONTIME_RES.
 * @param[in] byteIdx  Byte position in the memory bank field, 0 = MSB.
 * @param[in] nBytes   Size of the memory bank field (3 for the start
 *                     counter, 4 for the on time).
 * @param[in] value    New byte value.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick. A save is requested.
 */
void OpData_WriteResettableByte(uint8_t counter, uint8_t byteIdx, uint8_t nBytes, uint8_t value);

#endif /* OPDATA_H */

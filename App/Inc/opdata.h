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

void OpData_Init(void);

/** @brief  Main loop service (1 s accounting, periodic save). */
void OpData_Service(void);

/** @brief  Called on a confirmed light source fault (monitor bits). */
void OpData_OnLightSourceFault(uint8_t failureBits);

/** @brief  Read-only access to the live counters (32-bit atomic fields). */
const NVM_Counters_t *OpData_Get(void);

/** @brief  Resets the resettable light source counters (MB206). */
void OpData_ResetResettable(void);

/**
 * @brief  Byte write into a resettable counter (MB206 locations 0x04..0x06
 *         start counter, 0x0A..0x0D on time). byteIdx 0 = MSB.
 */
void OpData_WriteResettableByte(uint8_t counter, uint8_t byteIdx, uint8_t nBytes, uint8_t value);

#define OPDATA_CNT_LS_STARTS_RES   (0U)
#define OPDATA_CNT_LS_ONTIME_RES   (1U)

#endif /* OPDATA_H */

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

void MB_Init(void);

/** @brief  Persists memory bank 1 after a write burst (main loop). */
void MB_Service(void);

bool MB_IsImplemented(uint8_t bank);

/** @brief  Last accessible location of an implemented bank. */
uint8_t MB_LastLocation(uint8_t bank);

/** @brief  false = location not implemented (no answer). */
bool MB_Read(uint8_t bank, uint8_t loc, uint8_t *value);

/** @brief  true = written (the gear answers with the value). */
bool MB_Write(uint8_t bank, uint8_t loc, uint8_t value);

/** @brief  RESET MEMORY BANK: bank 0 = all resettable banks. */
void MB_Reset(uint8_t bank);

#endif /* DALI_MEMBANK_H */

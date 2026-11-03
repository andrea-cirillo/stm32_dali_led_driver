/**
 ******************************************************************************
 * @file    dali_gear.h
 * @brief   DALI-2 control gear application layer:
 *            IEC 62386-102 ed.2  general requirements for control gear
 *            IEC 62386-207 ed.2  LED modules (device type 6)
 *            IEC 62386-251/252/253 memory bank extensions (device types
 *            50 / 51 / 52) through dali_membank.
 *
 *          Gear_Tick1ms() runs in the SysTick context and owns all the DALI
 *          variables; the main loop only persists them (Gear_NvmService()).
 ******************************************************************************
 */
#ifndef DALI_GEAR_H
#define DALI_GEAR_H

#include <stdint.h>

/** @brief  Loads the persistent variables (main loop, before SysTick use). */
void Gear_Init(void);

/** @brief  Applies the power-on level (call once after Gear_Init()). */
void Gear_PowerOn(void);

/** @brief  1 ms tick: frame processing, timers, fade (SysTick context). */
void Gear_Tick1ms(void);

/** @brief  Deferred EEPROM write of the persistent variables (main loop). */
void Gear_NvmService(void);

#endif /* DALI_GEAR_H */

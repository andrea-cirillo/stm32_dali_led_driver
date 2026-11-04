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

/**
 * @brief   Loads the DALI persistent variables.
 * @details Seeds the random generator with the device UID, loads the
 *          variables from the EEPROM (factory / reset values if none are
 *          valid, with an immediate save request), sanitises out of range
 *          values (min / max level, short address) and selects the stored
 *          dimming curve.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     I2C_BusInit() and Light_Init() already called.
 * @note    Context: main, once, before the SysTick processing is enabled.
 */
void Gear_Init(void);

/**
 * @brief   Applies the power-on behaviour of IEC 62386-102.
 * @details Sets the light output, without fade, to powerOnLevel, or to
 *          lastLightLevel when powerOnLevel is MASK, clamped to
 *          [minLevel, maxLevel]. Marks "power cycle seen" (QUERY POWER
 *          FAILURE answers YES until the first arc power command).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once, after Gear_Init(). Internally protected
 *          against the SysTick tick.
 */
void Gear_PowerOn(void);

/**
 * @brief   1 ms tick of the control gear.
 * @details - DALI system failure: bus low for more than 500 ms applies the
 *            systemFailureLevel (if not MASK).
 *          - Processes the frame received by the PHY, if any (send-twice
 *            check, ENABLE DEVICE TYPE, addressing, command execution and
 *            backward frame).
 *          - Timers: initialisation (15 min), DAPC sequence (200 ms),
 *            identification (10 s).
 *          - Runs the light output tick and tracks lastLightLevel.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick only (via APP_Tick1ms()). Execution < 50 us.
 */
void Gear_Tick1ms(void);

/**
 * @brief   Deferred EEPROM write of the DALI persistent variables.
 * @details Saves a snapshot of the variables CFG_NVM_SAVE_DELAY_MS after
 *          the last change, or immediately after SAVE PERSISTENT VARIABLES.
 *          The snapshot is taken in a short critical section; the slow
 *          EEPROM write runs outside it, so the DALI timing is not affected.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop only.
 */
void Gear_NvmService(void);

#endif /* DALI_GEAR_H */

/**
 ******************************************************************************
 * @file    light_ctrl.h
 * @brief   Light output control: fade engine, dimming curve (IEC 62386-102
 *          logarithmic / IEC 62386-207 linear), PWM_OUT calibration, power
 *          stage switching, identification and fault latch.
 *
 *          Levels are handled internally in Q16 fixed point (level << 16) so
 *          that fades are smooth and exact for every fade time.
 *
 *          Context: all functions run in the SysTick context, except
 *          Light_SetFaultLatch() and the getters (main loop, atomic access).
 ******************************************************************************
 */
#ifndef LIGHT_CTRL_H
#define LIGHT_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#define LIGHT_CURVE_LOGARITHMIC  (0U)
#define LIGHT_CURVE_LINEAR       (1U)

/**
 * @brief   Initialises the light output in the OFF state.
 * @details Starts the PWM at maximum duty (minimum current), resets the
 *          fade engine to level 0 and keeps the power stage OFF.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main, once, before the SysTick tick is enabled.
 */
void Light_Init(void);

/**
 * @brief   1 ms tick of the light output.
 * @details Moves the actual level one step towards the target, completes a
 *          fade to off, advances the identification blinking, then updates
 *          the outputs: power stage (OFF pin) and PWM_OUT compare value
 *          computed through the dimming curve and the calibration vector.
 *          Also maintains the lamp-on time and start counters.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick (called by Gear_Tick1ms()).
 */
void Light_Tick1ms(void);

/**
 * @brief   Starts a timed fade (or an immediate change) to a target level.
 * @details IEC 62386-102 behaviour:
 *          - fadeMs = 0: the level changes immediately.
 *          - lamp off and target > 0: the output jumps to minLevel, then
 *            fades from there.
 *          - target = 0 with fadeMs > 0: fades down to minLevel, then
 *            switches off at the end of the fade.
 *          The step per ms is chosen so that the whole fade lasts fadeMs
 *          whatever the distance.
 * @param[in] target    Arc power level 0..254 (0 = off), already clamped by
 *                      the caller to [minLevel, maxLevel].
 * @param[in] fadeMs    Fade duration in milliseconds (0 = no fade).
 * @param[in] minLevel  Current minLevel (1..254), used as start / end point.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick (or main inside a critical section).
 */
void Light_FadeTo(uint8_t target, uint32_t fadeMs, uint8_t minLevel);

/**
 * @brief   Starts a fade at a constant rate (UP, DOWN, CONTINUOUS UP/DOWN).
 * @param[in] target        Level at which the fade stops (1..254).
 * @param[in] stepQ16PerMs  Speed in Q16 levels per millisecond
 *                          (fade rate F(n) * 65536 / 1000); 0 is treated as 1.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick. The lamp must already be on.
 */
void Light_FadeAtRate(uint8_t target, uint32_t stepQ16PerMs);

/**
 * @brief   Stops a running fade at the nearest integer level
 *          (DAPC MASK command).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick.
 */
void Light_StopFade(void);

/**
 * @brief   Returns the DALI actualLevel.
 * @par     Parameters
 *          None.
 * @return  Actual level 0..254 (rounded during a fade).
 * @note    Context: SysTick.
 */
uint8_t Light_GetActualLevel(void);

/**
 * @brief   Tells whether a fade is running (QUERY STATUS bit 4).
 * @par     Parameters
 *          None.
 * @return  true while the level moves or a fade to off is pending.
 * @note    Context: SysTick.
 */
bool Light_IsFading(void);

/**
 * @brief   Selects the dimming curve (IEC 62386-207 SELECT DIMMING CURVE).
 * @param[in] curve  LIGHT_CURVE_LOGARITHMIC (standard, 0.1 %..100 %) or
 *                   LIGHT_CURVE_LINEAR (level / 254). Other values select
 *                   the logarithmic curve.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick / main before the tick runs.
 */
void Light_SetCurve(uint8_t curve);

/**
 * @brief   Starts or stops the identification (IDENTIFY DEVICE).
 * @details While active the output alternates every 500 ms between level
 *          254 and minLevel, without switching the power stage off (no
 *          converter restarts) and without changing actualLevel.
 * @param[in] enable    true = start, false = stop.
 * @param[in] minLevel  Low level of the blinking (0 is treated as 1).
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick. The 10 s duration is timed by dali_gear.
 */
void Light_SetIdentify(bool enable, uint8_t minLevel);

/**
 * @brief   Tells whether the identification is active.
 * @par     Parameters
 *          None.
 * @return  true while identifying.
 * @note    Context: SysTick.
 */
bool Light_IsIdentifying(void);

/**
 * @brief   Latches the power stage OFF until the next reset.
 * @details Sets the latch flag and drives OFF HIGH immediately. From then
 *          on the output stage ignores every level request (the DALI
 *          variables keep being updated).
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main loop (led_monitor). Safe against the SysTick tick.
 */
void Light_SetFaultLatch(void);

/**
 * @brief   Tells whether a fault latched the power stage OFF.
 * @par     Parameters
 *          None.
 * @return  true after Light_SetFaultLatch(), until reset.
 * @note    Context: any.
 */
bool Light_IsFaultLatched(void);

/**
 * @brief   Tells whether the lamp is physically lit.
 * @par     Parameters
 *          None.
 * @return  true when the power stage is enabled with a level > 0.
 * @note    Context: any (QUERY LAMP POWER ON, counters, estimates).
 */
bool Light_IsLampOn(void);

/**
 * @brief   Time since the power stage was last enabled.
 * @par     Parameters
 *          None.
 * @return  Milliseconds since switch-on, 0 while off (saturates at 2^32-1).
 * @note    Context: any. Used for the fault detection blanking time.
 */
uint32_t Light_GetOnTimeMs(void);

/**
 * @brief   Number of off -> on transitions of the power stage since reset.
 * @par     Parameters
 *          None.
 * @return  Free running event counter (the caller computes differences).
 * @note    Context: any. Used for the light source start counters.
 */
uint32_t Light_GetStartEvents(void);

/**
 * @brief   Relative output current currently requested.
 * @par     Parameters
 *          None.
 * @return  Q16 fraction of the nominal current (65535 = 100 %), 0 when off.
 * @note    Context: any. Used for the 252/253 estimates.
 */
uint16_t Light_GetRelativeCurrentQ16(void);

#endif /* LIGHT_CTRL_H */

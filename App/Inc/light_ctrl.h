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

void Light_Init(void);

/** @brief  1 ms tick: fade progress and output update (SysTick context). */
void Light_Tick1ms(void);

/**
 * @brief  Starts a fade to 'target' (0 = off) lasting fadeMs (0 = immediate).
 *         From off the output jumps to minLevel first; a fade to off ends at
 *         minLevel and then switches off (IEC 62386-102 behaviour).
 */
void Light_FadeTo(uint8_t target, uint32_t fadeMs, uint8_t minLevel);

/** @brief  Fades towards 'target' at a constant rate (Q16 levels per ms). */
void Light_FadeAtRate(uint8_t target, uint32_t stepQ16PerMs);

/** @brief  Stops a running fade at the current level. */
void Light_StopFade(void);

uint8_t Light_GetActualLevel(void);
bool Light_IsFading(void);

void Light_SetCurve(uint8_t curve);

/** @brief  Identification: alternates max / minLevel every 500 ms. */
void Light_SetIdentify(bool enable, uint8_t minLevel);
bool Light_IsIdentifying(void);

/** @brief  Latches the power stage OFF until the next reset (main loop). */
void Light_SetFaultLatch(void);
bool Light_IsFaultLatched(void);

/** @brief  true when the power stage is enabled and the lamp is lit. */
bool Light_IsLampOn(void);

/** @brief  Milliseconds since the power stage was enabled (0 when off). */
uint32_t Light_GetOnTimeMs(void);

/** @brief  Number of off -> on transitions since reset. */
uint32_t Light_GetStartEvents(void);

/** @brief  Relative output current, Q16 (65535 = 100 %), 0 when off. */
uint16_t Light_GetRelativeCurrentQ16(void);

#endif /* LIGHT_CTRL_H */

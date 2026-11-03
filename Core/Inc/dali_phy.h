/**
 ******************************************************************************
 * @file    dali_phy.h
 * @brief   DALI physical / data link layer (IEC 62386-101) for control gear.
 *
 *          RX: MCU_RXD (PF1) edges on EXTI1, timestamps from TIM3 (1 MHz,
 *              free running). Manchester decoding inside the edge ISR, frame
 *              end detected with TIM3 CC1 (no edge for RX_STOP_US).
 *          TX: MCU_TXD (PF0) driven by TIM3 CC2 every half bit. Backward
 *              frames start DALI_PHY_SETTLING_US after the forward frame end.
 *
 *          Polarity is selected at build time, see app_config.h.
 ******************************************************************************
 */
#ifndef DALI_PHY_H
#define DALI_PHY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint32_t data;   /* received bits, MSB first, right aligned           */
  uint8_t  bits;   /* number of data bits, 0 = framing / timing error  */
} DALI_Frame_t;

void DALI_PHY_Init(void);

/** @brief  Fetches the last received frame (true if a new one is available). */
bool DALI_PHY_GetFrame(DALI_Frame_t *frame);

/**
 * @brief  Schedules an 8-bit backward frame as answer to the last forward
 *         frame. Silently dropped if called too late for the 10.5 ms window.
 */
void DALI_PHY_SendBackward(uint8_t value);

/** @brief  true while the bus is in the idle (high) state. */
bool DALI_PHY_IsBusIdle(void);

/** @brief  Free running microsecond counter (used as entropy source). */
uint16_t DALI_PHY_GetTimeUs(void);

/* Interrupt entry points (called from stm32f0xx_it.c) */
void DALI_PHY_RxEdgeIrq(void);
void DALI_PHY_TimerIrq(void);

#endif /* DALI_PHY_H */

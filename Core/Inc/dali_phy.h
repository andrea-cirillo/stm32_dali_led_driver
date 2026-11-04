/**
 ******************************************************************************
 * @file    dali_phy.h
 * @brief   DALI physical / data link layer (IEC 62386-101) for control gear.
 *
 *          RX: MCU_RXD (PF1) edges on EXTI1, timestamps from TIM3 (1 MHz,
 *              free running). Manchester decoding inside the edge ISR, frame
 *              end detected with TIM3 CC1 (no edge for RX_STOP_US).
 *          TX: MCU_TXD (PF0) driven by TIM3 CC2 every half bit. Backward
 *              frames start TX_SETTLING_US after the forward frame end.
 *
 *          Polarity is selected at build time, see app_config.h.
 ******************************************************************************
 */
#ifndef DALI_PHY_H
#define DALI_PHY_H

#include <stdint.h>
#include <stdbool.h>

/** Frame delivered by the receiver. */
typedef struct
{
  uint32_t data;   /**< received bits, MSB first, right aligned           */
  uint8_t  bits;   /**< number of data bits, 0 = framing / timing error  */
} DALI_Frame_t;

/**
 * @brief   Initialises the DALI transceiver.
 * @details Releases the bus, starts TIM3 as 1 MHz free running time base
 *          (CC1 = receive stop timeout, CC2 = transmit half-bit timing),
 *          maps EXTI1 to PF1 on both edges and enables the EXTI0_1 and TIM3
 *          interrupts at priority BSP_PRIO_DALI_PHY. The receiver starts in
 *          the error state and accepts frames only after a quiet bus time.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @pre     BSP_Init() already called.
 * @note    Context: main, once.
 */
void DALI_PHY_Init(void);

/**
 * @brief   Fetches the last frame decoded by the receiver.
 * @details One-slot mailbox: a new frame overwrites a non consumed one
 *          (DALI frames are at least ~22 ms apart and the mailbox is polled
 *          every 1 ms, so no frame is lost in practice).
 * @param[out] frame  Copy of the frame. frame->bits = 16 for a control gear
 *                    forward frame, 24 for a control device frame, 8 for a
 *                    backward frame of another gear, 0 for an invalid frame.
 * @return  true if a new frame (valid or not) was available, false otherwise.
 * @note    Context: SysTick (dali_gear).
 */
bool DALI_PHY_GetFrame(DALI_Frame_t *frame);

/**
 * @brief   Schedules an 8-bit backward frame as answer to the last frame.
 * @details The transmission starts 7.0 ms after the end of the last
 *          received frame (allowed window 5.5..10.5 ms), or 60 us from now
 *          if that time is already close. If more than 9.5 ms have already
 *          elapsed, or a transmission is in progress, the answer is dropped.
 *          The frame is: start bit, 8 data bits MSB first, 2 stop bits idle.
 * @param[in] value  Byte to send (e.g. 0xFF = YES).
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick, within the processing of the received frame.
 */
void DALI_PHY_SendBackward(uint8_t value);

/**
 * @brief   Samples the bus level.
 * @par     Parameters
 *          None.
 * @return  true if the bus is idle (HIGH), false if it is active (LOW).
 * @note    Context: any. Used every 1 ms for the 500 ms system failure check.
 */
bool DALI_PHY_IsBusIdle(void);

/**
 * @brief   Returns the free running 1 MHz time base.
 * @par     Parameters
 *          None.
 * @return  TIM3 counter in microseconds (16 bit, wraps every 65.5 ms).
 * @note    Context: any. Used as an entropy source for RANDOMISE.
 */
uint16_t DALI_PHY_GetTimeUs(void);

/**
 * @brief   RX edge interrupt handler (EXTI line 1, PF1).
 * @details Time stamps the edge and runs the Manchester decoder: checks the
 *          interval (1 or 2 half bits), extracts the bit value at mid-bit
 *          edges and re-arms the stop timeout. Edges during an own
 *          transmission are ignored.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: EXTI0_1_IRQHandler only (pending flag already cleared).
 */
void DALI_PHY_RxEdgeIrq(void);

/**
 * @brief   TIM3 interrupt handler.
 * @details CC1: no edge for RX_STOP_US. A frame in progress is completed
 *          (bus idle) or reported as error (bus stuck low).
 *          CC2: next transmit half bit, release of the bus, end of the stop
 *          condition.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: TIM3_IRQHandler only.
 */
void DALI_PHY_TimerIrq(void);

#endif /* DALI_PHY_H */

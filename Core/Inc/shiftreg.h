/**
 ******************************************************************************
 * @file    shiftreg.h
 * @brief   TI SN74HCS596 (open-drain 8-bit shift register) driver.
 *          DS = PB1 (CurrentLimit_DS), SHCP = PA7, STCP = PA6.
 *          OE is tied low and MR tied high on the board.
 ******************************************************************************
 */
#ifndef SHIFTREG_H
#define SHIFTREG_H

#include <stdint.h>

/**
 * @brief   Shifts a byte into the SN74HCS596 and latches it to the outputs.
 * @details The byte is sent MSB first on DS, one SHCP rising edge per bit,
 *          then a single STCP pulse transfers the shift register to the
 *          output register. After the call bit n of 'value' drives output Qn:
 *          '1' switches the open-drain output ON, which connects the
 *          corresponding resistor of the current limitation network
 *          (Q0 R92, Q1 R98, Q2 R93, Q3 R99, Q4 R94, Q5 R100) to GND.
 *          Q6 and Q7 are not connected.
 * @param[in] value  Output pattern (bit n -> Qn).
 * @par     Return value
 *          None (void).
 * @note    Context: main. Duration ~2 us. Every phase lasts > 100 ns.
 */
void ShiftReg_Write(uint8_t value);

#endif /* SHIFTREG_H */

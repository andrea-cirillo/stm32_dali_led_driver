/**
 ******************************************************************************
 * @file    shiftreg.h
 * @brief   TI SN74HCS596 (open-drain 8-bit shift register) driver.
 *          DS = PB1 (CurrentLimit_DS), SHCP = PA7, STCP = PA6.
 *          OE is tied low and MR tied high on the board.
 *
 *          Bit n of the written value drives output Qn. A '1' switches the
 *          open-drain output ON, i.e. connects the corresponding resistor of
 *          the current limitation network (R92..R100) to GND.
 ******************************************************************************
 */
#ifndef SHIFTREG_H
#define SHIFTREG_H

#include <stdint.h>

void ShiftReg_Write(uint8_t value);

#endif /* SHIFTREG_H */

#ifndef DALI_SW_H
#define DALI_SW_H
#include "main.h"
/* DALI forward frame: address byte + data/command byte, MSB first.
 * Requires external DALI physical-layer interface; GPIO is not bus-safe. */
HAL_StatusTypeDef DALI_SendForwardFrame(uint8_t address, uint8_t data);
#endif

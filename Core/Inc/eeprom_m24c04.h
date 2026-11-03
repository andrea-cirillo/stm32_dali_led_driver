#ifndef EEPROM_M24C04_H
#define EEPROM_M24C04_H
#include "main.h"
#define M24C04_CAPACITY 512U
HAL_StatusTypeDef M24C04_Init(void);
HAL_StatusTypeDef M24C04_Read(uint16_t address, uint8_t *data, uint16_t length);
HAL_StatusTypeDef M24C04_Write(uint16_t address, const uint8_t *data, uint16_t length);
#endif

#ifndef SOFT_UART_H
#define SOFT_UART_H
#include "main.h"
#define SOFTUART_BAUD 9600U
/* Per schematic: PF0 is MCU_TXD, PF1 is MCU_RXD */
#define SOFTUART_TX_PORT GPIOF
#define SOFTUART_TX_PIN  GPIO_PIN_0
#define SOFTUART_RX_PORT GPIOF
#define SOFTUART_RX_PIN  GPIO_PIN_1
void SoftUART_Init(void);
void SoftUART_WriteByte(uint8_t byte);
void SoftUART_Write(const uint8_t *data, uint16_t len);
HAL_StatusTypeDef SoftUART_ReadByte(uint8_t *byte, uint32_t timeout_ms);
#endif

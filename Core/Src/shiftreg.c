#include "shiftreg.h"
/* Schematic mapping */
#define SR_DATA_PORT GPIOB
#define SR_DATA_PIN  GPIO_PIN_1
#define SR_SHCP_PORT GPIOA
#define SR_SHCP_PIN  GPIO_PIN_7
#define SR_STCP_PORT GPIOA
#define SR_STCP_PIN  GPIO_PIN_6
void ShiftRegister_Write(uint8_t value) {
    HAL_GPIO_WritePin(SR_STCP_PORT, SR_STCP_PIN, GPIO_PIN_RESET);
    for (int8_t i=7; i>=0; --i) {
        HAL_GPIO_WritePin(SR_DATA_PORT, SR_DATA_PIN, (value & (1U<<i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SR_SHCP_PORT, SR_SHCP_PIN, GPIO_PIN_RESET);
        __NOP(); HAL_GPIO_WritePin(SR_SHCP_PORT, SR_SHCP_PIN, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(SR_STCP_PORT, SR_STCP_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_STCP_PORT, SR_STCP_PIN, GPIO_PIN_RESET);
}

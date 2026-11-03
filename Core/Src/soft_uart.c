#include "soft_uart.h"

/* Rough software delay calibrated for STM32F030 at 48 MHz, -O0/-Og.
 * Production recommendation: replace with TIM14-based microsecond timing. */
static void delay_halfbit(void) {
    volatile uint32_t n = (SystemCoreClock / SOFTUART_BAUD / 2U) / 5U;
    while (n--) { __NOP(); }
}
static void delay_bit(void) { delay_halfbit(); delay_halfbit(); }

void SoftUART_Init(void) {
    GPIO_InitTypeDef g = {0};
    g.Pin = SOFTUART_RX_PIN; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFTUART_RX_PORT, &g);
    g.Pin = SOFTUART_TX_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFTUART_TX_PORT, &g);
    HAL_GPIO_WritePin(SOFTUART_TX_PORT, SOFTUART_TX_PIN, GPIO_PIN_SET);
}
void SoftUART_WriteByte(uint8_t b) {
    HAL_GPIO_WritePin(SOFTUART_TX_PORT, SOFTUART_TX_PIN, GPIO_PIN_RESET); /* start */
    delay_bit();
    for (uint8_t i=0; i<8; ++i) {
        HAL_GPIO_WritePin(SOFTUART_TX_PORT, SOFTUART_TX_PIN, (b & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        delay_bit(); b >>= 1;
    }
    HAL_GPIO_WritePin(SOFTUART_TX_PORT, SOFTUART_TX_PIN, GPIO_PIN_SET); /* stop */
    delay_bit();
}
void SoftUART_Write(const uint8_t *data, uint16_t len) {
    while (len--) SoftUART_WriteByte(*data++);
}
HAL_StatusTypeDef SoftUART_ReadByte(uint8_t *byte, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(SOFTUART_RX_PORT, SOFTUART_RX_PIN) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= timeout_ms) return HAL_TIMEOUT;
    }
    delay_halfbit(); /* center of start bit */
    if (HAL_GPIO_ReadPin(SOFTUART_RX_PORT, SOFTUART_RX_PIN) != GPIO_PIN_RESET) return HAL_ERROR;
    for (uint8_t i=0; i<8; ++i) {
        delay_bit();
        if (HAL_GPIO_ReadPin(SOFTUART_RX_PORT, SOFTUART_RX_PIN) == GPIO_PIN_SET) *byte |= (uint8_t)(1U << i);
        else *byte &= (uint8_t)~(1U << i);
    }
    delay_bit();
    return HAL_OK;
}

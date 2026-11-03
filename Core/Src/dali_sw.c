#include "dali_sw.h"
#include "soft_uart.h"

/* DALI interface board TXD input is driven by MCU_TXD = PF0 */
#define DALI_TX_PORT GPIOF
#define DALI_TX_PIN  GPIO_PIN_0
#define DALI_HALF_BIT_US 417U

/* Timing loop is deliberately isolated for calibration. At 48 MHz this
 * loop is only approximate; use a 1 MHz hardware timer for production. */
static void delay_us_approx(uint32_t us) {
    volatile uint32_t n = (SystemCoreClock / 1000000U / 5U) * us;
    while (n--) __NOP();
}
static void tx_level(GPIO_PinState s) { HAL_GPIO_WritePin(DALI_TX_PORT, DALI_TX_PIN, s); }
static void dali_bit(uint8_t bit) {
    /* DALI Manchester: logical 1 = high->low; logical 0 = low->high
       (polarity depends on transceiver convention; verify on scope). */
    if (bit) { tx_level(GPIO_PIN_SET); delay_us_approx(DALI_HALF_BIT_US); tx_level(GPIO_PIN_RESET); }
    else     { tx_level(GPIO_PIN_RESET); delay_us_approx(DALI_HALF_BIT_US); tx_level(GPIO_PIN_SET); }
    delay_us_approx(DALI_HALF_BIT_US);
}
HAL_StatusTypeDef DALI_SendForwardFrame(uint8_t address, uint8_t data) {
    /* Start bit = logical 1, then address/data MSB-first, followed by idle/stop.
       DALI forward frame is 1 start + 16 data bits + 2 stop bit times. */
    GPIO_InitTypeDef g = {0};
    g.Pin = DALI_TX_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DALI_TX_PORT, &g);
    tx_level(GPIO_PIN_SET);
    delay_us_approx(10000U); /* conservative bus idle before frame */
    dali_bit(1U);
    for (int8_t i=7; i>=0; --i) dali_bit((address >> i) & 1U);
    for (int8_t i=7; i>=0; --i) dali_bit((data >> i) & 1U);
    tx_level(GPIO_PIN_SET);
    delay_us_approx(2U * 833U);
    return HAL_OK;
}

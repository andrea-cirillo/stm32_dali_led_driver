#include "eeprom_m24c04.h"

/* Proposed software-I2C remap. Check these pins are free on the PCB. */
#define I2C_GPIO GPIOA
#define SCL_PIN GPIO_PIN_4
#define SDA_PIN GPIO_PIN_5
#define M24C04_BASE 0x50U /* 7-bit base; address block bit handled below */

static void wait_short(void) { for (volatile uint32_t i=0; i<18; ++i) __NOP(); }
static void scl(uint8_t v) { HAL_GPIO_WritePin(I2C_GPIO,SCL_PIN,v?GPIO_PIN_SET:GPIO_PIN_RESET); wait_short(); }
static void sda(uint8_t v) { HAL_GPIO_WritePin(I2C_GPIO,SDA_PIN,v?GPIO_PIN_SET:GPIO_PIN_RESET); wait_short(); }
static uint8_t read_sda(void) { return HAL_GPIO_ReadPin(I2C_GPIO,SDA_PIN)==GPIO_PIN_SET; }
static void sda_output(void) { GPIO_InitTypeDef g={0}; g.Pin=SDA_PIN; g.Mode=GPIO_MODE_OUTPUT_OD; g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_HIGH; HAL_GPIO_Init(I2C_GPIO,&g); }
static void sda_input(void) { GPIO_InitTypeDef g={0}; g.Pin=SDA_PIN; g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP; HAL_GPIO_Init(I2C_GPIO,&g); }
static void start_cond(void) { sda_output(); sda(1); scl(1); sda(0); scl(0); }
static void stop_cond(void) { sda_output(); sda(0); scl(1); sda(1); }
static uint8_t write_byte(uint8_t b) {
    sda_output();
    for (uint8_t m=0x80; m; m>>=1) { sda((b&m)!=0); scl(1); scl(0); }
    sda_input(); scl(1); uint8_t nack=read_sda(); scl(0); sda_output(); return nack;
}
static uint8_t read_byte(uint8_t ack) {
    uint8_t b=0; sda_input();
    for (uint8_t i=0;i<8;i++) { scl(1); b=(uint8_t)((b<<1)|read_sda()); scl(0); }
    sda_output(); sda(ack?0:1); scl(1); scl(0); sda(1); return b;
}
static uint8_t dev_addr(uint16_t a, uint8_t read) {
    /* M24C04 uses A8 as the block-select bit in the control byte. */
    return (uint8_t)((M24C04_BASE<<1) | ((a>>8)&1U) | (read&1U));
}
HAL_StatusTypeDef M24C04_Init(void) {
    GPIO_InitTypeDef g={0}; __HAL_RCC_GPIOA_CLK_ENABLE();
    g.Pin=SCL_PIN|SDA_PIN; g.Mode=GPIO_MODE_OUTPUT_OD; g.Pull=GPIO_PULLUP; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_GPIO,&g); scl(1); sda(1); return HAL_OK;
}
HAL_StatusTypeDef M24C04_Read(uint16_t address, uint8_t *data, uint16_t length) {
    if (!data || address>=M24C04_CAPACITY || length>M24C04_CAPACITY-address) return HAL_ERROR;
    while (length) {
        start_cond();
        if (write_byte(dev_addr(address,0))) { stop_cond(); return HAL_ERROR; }
        if (write_byte((uint8_t)address)) { stop_cond(); return HAL_ERROR; }
        start_cond();
        if (write_byte(dev_addr(address,1))) { stop_cond(); return HAL_ERROR; }
        while (length) { --length; *data++=read_byte(length!=0); ++address;
            if (length && ((address & 0xFFU)==0U)) break; }
        stop_cond();
    }
    return HAL_OK;
}
HAL_StatusTypeDef M24C04_Write(uint16_t address, const uint8_t *data, uint16_t length) {
    if (!data || address>=M24C04_CAPACITY || length>M24C04_CAPACITY-address) return HAL_ERROR;
    while (length) {
        uint16_t page_left=(uint16_t)(16U-(address&0x0FU));
        uint16_t n=length<page_left?length:page_left;
        start_cond();
        if (write_byte(dev_addr(address,0)) || write_byte((uint8_t)address)) { stop_cond(); return HAL_ERROR; }
        for (uint16_t i=0;i<n;i++) if (write_byte(data[i])) { stop_cond(); return HAL_ERROR; }
        stop_cond();
        HAL_Delay(6); /* internal write cycle; production can use ACK polling */
        address+=n; data+=n; length-=n;
    }
    return HAL_OK;
}

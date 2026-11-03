/**
 * @file    mock_hw.c
 * @brief   Host fakes of the hardware drivers (registers, BSP, EEPROM, NTAG 5,
 *          shift register, PWM).
 */
#include <string.h>
#include "mock_hw.h"
#include "bsp.h"
#include "pwm_drv.h"
#include "eeprom_m24c04.h"
#include "ntag5.h"
#include "shiftreg.h"

GPIO_TypeDef mockGPIOA, mockGPIOB, mockGPIOF;
TIM_TypeDef mockTIM3;
RCC_TypeDef mockRCC;
SYSCFG_TypeDef mockSYSCFG;
EXTI_TypeDef mockEXTI;

uint32_t g_tickMs = 0U;
bool g_powerStage = false;
uint16_t g_pwmDuty = 0U;
uint8_t g_eeprom[M24C04_SIZE];
uint32_t g_eepromWrites = 0U;
uint8_t g_tag[2048];
bool g_tagPresent = true;
uint8_t g_shiftReg = 0U;

void Mock_Reset(void)
{
  memset(g_eeprom, 0xFF, sizeof(g_eeprom));
  memset(g_tag, 0x00, sizeof(g_tag));
  g_tickMs = 0U;
  g_eepromWrites = 0U;
}

/* ---- BSP ---------------------------------------------------------------- */
uint32_t BSP_GetTickMs(void) { return g_tickMs; }
void BSP_TickIncrement(void) { g_tickMs++; }
void BSP_DelayMs(uint32_t ms) { g_tickMs += ms; }
void BSP_PowerStageEnable(bool enable) { g_powerStage = enable; }
bool BSP_PowerStageIsEnabled(void) { return g_powerStage; }
uint32_t BSP_EnterCritical(void) { return 0U; }
void BSP_ExitCritical(uint32_t primask) { (void)primask; }
void BSP_GetUid(uint8_t uid[12])
{
  uint32_t i;
  for (i = 0U; i < 12U; i++) { uid[i] = (uint8_t)(0x30U + i); }
}

/* ---- PWM ---------------------------------------------------------------- */
void PWM_DrvInit(void) { g_pwmDuty = 4096U; }
void PWM_DrvSetDuty(uint16_t duty) { g_pwmDuty = duty; }

/* ---- EEPROM ------------------------------------------------------------- */
bool M24C04_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
  if (((uint32_t)addr + len) > M24C04_SIZE) { return false; }
  memcpy(data, &g_eeprom[addr], len);
  return true;
}

bool M24C04_Write(uint16_t addr, const uint8_t *data, uint16_t len)
{
  if (((uint32_t)addr + len) > M24C04_SIZE) { return false; }
  memcpy(&g_eeprom[addr], data, len);
  g_eepromWrites++;
  return true;
}

/* ---- NTAG 5 ------------------------------------------------------------- */
bool NTAG5_ReadBlocks(uint16_t block_addr, uint8_t *buf, uint8_t len)
{
  const uint32_t off = (uint32_t)block_addr * 4U;
  if ((!g_tagPresent) || ((off + len) > sizeof(g_tag))) { return false; }
  memcpy(buf, &g_tag[off], len);
  return true;
}

bool NTAG5_WriteReg(uint16_t reg_addr, uint8_t reg_byte, uint8_t mask, uint8_t value)
{
  (void)reg_addr; (void)reg_byte; (void)mask; (void)value;
  return g_tagPresent;
}

/* ---- shift register ----------------------------------------------------- */
void ShiftReg_Write(uint8_t value) { g_shiftReg = value; }

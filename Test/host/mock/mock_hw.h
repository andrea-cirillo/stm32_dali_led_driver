/**
 * @file    mock_hw.h
 * @brief   Host fakes: observable state of the simulated hardware.
 */
#ifndef MOCK_HW_H
#define MOCK_HW_H

#include <stdint.h>
#include <stdbool.h>

extern uint32_t g_tickMs;
extern bool g_powerStage;
extern uint16_t g_pwmDuty;
extern uint8_t g_eeprom[512];
extern uint32_t g_eepromWrites;
extern uint8_t g_tag[2048];
extern bool g_tagPresent;
extern uint8_t g_shiftReg;

void Mock_Reset(void);

/* minimal assertion framework */
extern int g_failures;
extern int g_checks;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_failures++; \
  printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_EQ(a, b) do { long _a = (long)(a); long _b = (long)(b); g_checks++; \
  if (_a != _b) { g_failures++; \
  printf("  FAIL %s:%d: %s = %ld, expected %ld\n", __FILE__, __LINE__, #a, _a, _b); } } while (0)

#endif

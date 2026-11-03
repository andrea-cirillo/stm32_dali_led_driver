/**
 ******************************************************************************
 * @file    opdata.c
 * @brief   Operating data accounting and persistence.
 *
 *          The live counters are modified by the main loop (inside short
 *          critical sections) and by the DALI command handler (SysTick
 *          context) for the resettable counters.
 ******************************************************************************
 */
#include "opdata.h"
#include "app_config.h"
#include "bsp.h"
#include "light_ctrl.h"
#include "led_monitor.h"

#define MWS_PER_WH             (3600000UL)
#define COUNTER_1B_MAX         (0xFDU)

static NVM_Counters_t s_cnt;
static uint32_t s_lastSecondMs = 0U;
static uint32_t s_lastSaveMs = 0U;
static uint32_t s_energyMws = 0U;
static uint32_t s_lastStartEvents = 0U;
static volatile bool s_saveRequest = false;

static uint32_t inc_sat(uint32_t v)
{
  return (v < 0xFFFFFFFDUL) ? (v + 1UL) : v;
}

static uint8_t inc_sat8(uint8_t v)
{
  return (v < COUNTER_1B_MAX) ? (uint8_t)(v + 1U) : v;
}

void OpData_Init(void)
{
  if (!NVM_LoadCounters(&s_cnt))
  {
    s_cnt.gearOperatingTimeS = 0U;
    s_cnt.gearStartCounter = 0U;
    s_cnt.lsOnTimeS = 0U;
    s_cnt.lsOnTimeResS = 0U;
    s_cnt.lsStartCounter = 0U;
    s_cnt.lsStartCounterRes = 0U;
    s_cnt.activeEnergyWh = 0U;
    s_cnt.gearFailureCounter = 0U;
    s_cnt.lsFailureCounter = 0U;
    s_cnt.lsShortCounter = 0U;
    s_cnt.lsOpenCounter = 0U;
  }
  s_cnt.gearStartCounter = inc_sat(s_cnt.gearStartCounter);
  (void)NVM_SaveCounters(&s_cnt);
  s_lastSecondMs = BSP_GetTickMs();
  s_lastSaveMs = s_lastSecondMs;
  s_lastStartEvents = Light_GetStartEvents();
}

void OpData_OnLightSourceFault(uint8_t failureBits)
{
  const uint32_t pm = BSP_EnterCritical();

  s_cnt.lsFailureCounter = inc_sat8(s_cnt.lsFailureCounter);
  if ((failureBits & MON_FAIL_SHORT_CIRCUIT) != 0U)
  {
    s_cnt.lsShortCounter = inc_sat8(s_cnt.lsShortCounter);
  }
  if ((failureBits & MON_FAIL_OPEN_CIRCUIT) != 0U)
  {
    s_cnt.lsOpenCounter = inc_sat8(s_cnt.lsOpenCounter);
  }
  BSP_ExitCritical(pm);
  s_saveRequest = true;
}

void OpData_Service(void)
{
  const uint32_t now = BSP_GetTickMs();

  if ((now - s_lastSecondMs) >= 1000U)
  {
    const uint32_t starts = Light_GetStartEvents();
    const uint32_t newStarts = starts - s_lastStartEvents;
    const bool lampOn = Light_IsLampOn();
    uint32_t pm;
    uint32_t i;

    s_lastSecondMs += 1000U;
    s_lastStartEvents = starts;
    s_energyMws += Monitor_GetInputPowerMw();

    pm = BSP_EnterCritical();
    s_cnt.gearOperatingTimeS = inc_sat(s_cnt.gearOperatingTimeS);
    if (lampOn)
    {
      s_cnt.lsOnTimeS = inc_sat(s_cnt.lsOnTimeS);
      s_cnt.lsOnTimeResS = inc_sat(s_cnt.lsOnTimeResS);
    }
    for (i = 0U; i < newStarts; i++)
    {
      s_cnt.lsStartCounter = inc_sat(s_cnt.lsStartCounter);
      s_cnt.lsStartCounterRes = inc_sat(s_cnt.lsStartCounterRes);
    }
    while (s_energyMws >= MWS_PER_WH)
    {
      s_energyMws -= MWS_PER_WH;
      s_cnt.activeEnergyWh = inc_sat(s_cnt.activeEnergyWh);
    }
    BSP_ExitCritical(pm);
  }

  if (s_saveRequest || ((now - s_lastSaveMs) >= CFG_COUNTERS_SAVE_PERIOD_MS))
  {
    NVM_Counters_t copy;
    const uint32_t pm = BSP_EnterCritical();
    copy = s_cnt;
    s_saveRequest = false;
    BSP_ExitCritical(pm);
    s_lastSaveMs = now;
    (void)NVM_SaveCounters(&copy);
  }
}

const NVM_Counters_t *OpData_Get(void)
{
  return &s_cnt;
}

void OpData_ResetResettable(void)
{
  s_cnt.lsOnTimeResS = 0U;
  s_cnt.lsStartCounterRes = 0U;
  s_saveRequest = true;
}

void OpData_WriteResettableByte(uint8_t counter, uint8_t byteIdx, uint8_t nBytes, uint8_t value)
{
  const uint32_t shift = 8UL * (uint32_t)((nBytes - 1U) - byteIdx);
  const uint32_t mask = 0xFFUL << shift;
  uint32_t *const p = (counter == OPDATA_CNT_LS_STARTS_RES) ? &s_cnt.lsStartCounterRes
                                                             : &s_cnt.lsOnTimeResS;
  *p = (*p & ~mask) | ((uint32_t)value << shift);
  s_saveRequest = true;
}

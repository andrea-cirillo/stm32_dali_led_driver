/**
 ******************************************************************************
 * @file    main.c
 * @brief   DALI-2 LED driver for STM32F030F4P6 (IEC 62386-101/102/207/251/
 *          252/253). Register level implementation, no HAL.
 ******************************************************************************
 */
#include "main.h"
#include <stdbool.h>
#include "bsp.h"
#include "i2c_bus.h"
#include "dali_phy.h"
#include "dali_gear.h"
#include "dali_membank.h"
#include "light_ctrl.h"
#include "led_monitor.h"
#include "opdata.h"
#include "variant.h"

static volatile bool s_appReady = false;

int main(void)
{
  /* 1. power stage OFF before anything else */
  BSP_EarlyInit();
  BSP_Init();
  I2C_BusInit();

  /* 2. output stage: PWM at minimum current, stage OFF */
  Light_Init();

  /* 3. current limitation network (only at boot, stage still OFF) */
  Variant_ApplyAtBoot();

  /* 4. persistent data */
  MB_Init();
  OpData_Init();
  Monitor_Init();
  Gear_Init();

  /* 5. DALI bus and watchdog */
  DALI_PHY_Init();
  BSP_WatchdogInit();
  s_appReady = true;

  /* 6. power-on level (IEC 62386-102) */
  Gear_PowerOn();

  for (;;)
  {
    BSP_WatchdogRefresh();
    Monitor_Service();
    OpData_Service();
    Gear_NvmService();
    MB_Service();
    Variant_Service();
  }
}

void APP_Tick1ms(void)
{
  BSP_TickIncrement();
  if (s_appReady)
  {
    Gear_Tick1ms();
  }
}

void APP_NfcEventIrq(void)
{
  Variant_NfcEventIrq();
}

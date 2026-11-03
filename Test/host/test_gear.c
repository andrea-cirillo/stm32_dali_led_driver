/**
 * @file    test_gear.c
 * @brief   Host test of the DALI-2 control gear application layer:
 *          dali_gear.c, light_ctrl.c, dali_membank.c, nvm.c, variant.c,
 *          crc16.c running on fakes of the PHY / monitor / counters.
 */
#include <stdio.h>
#include <string.h>
#include "mock_hw.h"
#include "dali_phy.h"
#include "dali_gear.h"
#include "dali_membank.h"
#include "light_ctrl.h"
#include "led_monitor.h"
#include "opdata.h"
#include "variant.h"
#include "app_config.h"

int g_failures = 0;
int g_checks = 0;

/* ---- fakes: PHY ----------------------------------------------------------- */
static DALI_Frame_t s_rx;
static bool s_rxPending = false;
static int s_reply = -1;
static bool s_busIdle = true;

bool DALI_PHY_GetFrame(DALI_Frame_t *f)
{
  bool r = s_rxPending;
  if (r) { *f = s_rx; s_rxPending = false; }
  return r;
}
void DALI_PHY_SendBackward(uint8_t v) { s_reply = v; }
bool DALI_PHY_IsBusIdle(void) { return s_busIdle; }
uint16_t DALI_PHY_GetTimeUs(void) { return (uint16_t)(g_tickMs * 7919U); }

/* ---- fakes: monitor / counters -------------------------------------------- */
static uint8_t s_fail = 0U;
uint8_t Monitor_GetFailureStatus(void) { return s_fail; }
uint32_t Monitor_GetLedVoltageMv(void) { return 120000U; }
uint32_t Monitor_GetLedCurrentMa(void) { return 500U; }
uint32_t Monitor_GetInputPowerMw(void) { return 66700U; }
int16_t Monitor_GetTemperatureC(void) { return 40; }

static NVM_Counters_t s_cnt;
const NVM_Counters_t *OpData_Get(void) { return &s_cnt; }
void OpData_ResetResettable(void) { s_cnt.lsOnTimeResS = 0U; s_cnt.lsStartCounterRes = 0U; }
void OpData_WriteResettableByte(uint8_t c, uint8_t i, uint8_t n, uint8_t v)
{
  uint32_t *p = (c == OPDATA_CNT_LS_STARTS_RES) ? &s_cnt.lsStartCounterRes : &s_cnt.lsOnTimeResS;
  uint32_t sh = 8U * (uint32_t)((n - 1U) - i);
  *p = (*p & ~(0xFFU << sh)) | ((uint32_t)v << sh);
}

/* ---- helpers --------------------------------------------------------------- */
static void tick(uint32_t ms)
{
  while (ms-- > 0U)
  {
    g_tickMs++;
    Gear_Tick1ms();
  }
}

/* sends a forward frame, returns the backward frame or -1 */
static int send(uint8_t a, uint8_t d)
{
  int r;
  s_rx.data = ((uint32_t)a << 8) | d;
  s_rx.bits = 16U;
  s_rxPending = true;
  s_reply = -1;
  tick(1U);
  r = s_reply;
  tick(20U);
  return r;
}

static int send2(uint8_t a, uint8_t d)
{
  (void)send(a, d);
  return send(a, d);
}

#define BC          (0xFFU)            /* broadcast command               */
#define BC_DAPC     (0xFEU)
#define SA(n)       ((uint8_t)(((n) << 1) | 1U))
#define SA_DAPC(n)  ((uint8_t)((n) << 1))
#define DTR0(v)     ((void)send(0xA3U, (v)))
#define DTR1(v)     ((void)send(0xC3U, (v)))

static void boot(void)
{
  Light_Init();
  Variant_ApplyAtBoot();
  MB_Init();
  Gear_Init();
  Gear_PowerOn();
  tick(10U);
}

/* ---- tests ---------------------------------------------------------------- */

static void test_power_on(void)
{
  printf("power on\n");
  CHECK_EQ(send(BC, 0xA0), 254);            /* QUERY ACTUAL LEVEL */
  CHECK(g_powerStage);
  CHECK_EQ(g_pwmDuty, 0);                   /* 100 % current */
  CHECK_EQ(send(BC, 0x90), 0xE4);           /* status: pwr cycle, no addr, reset, lamp on */
  CHECK_EQ(send(BC, 0x9B), 0xFF);           /* QUERY POWER FAILURE */
  CHECK_EQ(send(BC, 0x91), 0xFF);           /* PRESENT */
  CHECK_EQ(send(BC, 0x97), 0x08);           /* version 2.0 */
  CHECK_EQ(send(BC, 0x9A), CFG_PHYSICAL_MIN_LEVEL);
  CHECK_EQ(g_shiftReg, 0x01);               /* default variant */
}

static void test_off_on(void)
{
  printf("DAPC off / on\n");
  (void)send(BC_DAPC, 0U);
  CHECK(!g_powerStage);
  CHECK_EQ(g_pwmDuty, 4096);
  CHECK_EQ(send(BC, 0x93), -1);             /* LAMP POWER ON: no */
  CHECK_EQ(send(BC, 0x9B), -1);             /* power failure cleared */
  (void)send(BC_DAPC, 254U);
  CHECK(g_powerStage);
  CHECK_EQ(send(BC, 0x93), 0xFF);
}

static void test_addressing(void)
{
  uint32_t lo = 0U;
  uint32_t hi = 0xFFFFFFU;

  printf("addressing (initialise, randomise, search, program)\n");
  (void)send2(0xA5U, 0x00U);                /* INITIALISE all */
  (void)send2(0xA7U, 0x00U);                /* RANDOMISE */
  while (lo < hi)
  {
    const uint32_t mid = (lo + hi) / 2U;
    (void)send(0xB1U, (uint8_t)(mid >> 16));
    (void)send(0xB3U, (uint8_t)(mid >> 8));
    (void)send(0xB5U, (uint8_t)mid);
    if (send(0xA9U, 0x00U) == 0xFF) { hi = mid; } else { lo = mid + 1U; }
  }
  (void)send(0xB1U, (uint8_t)(lo >> 16));
  (void)send(0xB3U, (uint8_t)(lo >> 8));
  (void)send(0xB5U, (uint8_t)lo);
  CHECK_EQ(send(BC, 0xC2), (int)((lo >> 16) & 0xFFU));   /* QUERY RANDOM ADDRESS H */
  CHECK_EQ(send(BC, 0xC4), (int)(lo & 0xFFU));
  (void)send(0xB7U, SA(5));                 /* PROGRAM SHORT ADDRESS 5 */
  CHECK_EQ(send(0xB9U, SA(5)), 0xFF);       /* VERIFY */
  CHECK_EQ(send(0xBBU, 0x00U), SA(5));      /* QUERY SHORT ADDRESS */
  (void)send(0xABU, 0x00U);                 /* WITHDRAW */
  CHECK_EQ(send(0xA9U, 0x00U), -1);         /* withdrawn: no answer */
  (void)send(0xA1U, 0x00U);                 /* TERMINATE */
  CHECK_EQ(send(0xB9U, SA(5)), -1);         /* initialisation disabled */
  CHECK_EQ(send(SA(5), 0x96), -1);          /* has short address */
  CHECK_EQ(send(SA(6), 0x91), -1);          /* other address */
  CHECK_EQ(send(SA(5), 0x91), 0xFF);
  CHECK_EQ(send(0xFDU, 0x91), -1);          /* broadcast unaddressed */
}

static void test_send_twice(void)
{
  printf("send twice\n");
  DTR0(200U);
  (void)send(SA(5), 0x2A);                  /* SET MAX once: ignored */
  CHECK_EQ(send(SA(5), 0xA1), 254);
  DTR0(200U);
  (void)send(SA(5), 0x2A);
  tick(150U);                               /* too late */
  (void)send(SA(5), 0x2A);
  CHECK_EQ(send(SA(5), 0xA1), 254);
  DTR0(200U);
  (void)send(SA(5), 0x2A);
  (void)send(SA(5), 0x91);                  /* other frame in between */
  (void)send(SA(5), 0x2A);
  CHECK_EQ(send(SA(5), 0xA1), 254);
  DTR0(200U);
  (void)send2(SA(5), 0x2A);
  CHECK_EQ(send(SA(5), 0xA1), 200);
  CHECK_EQ(send(SA(5), 0xA0), 200);         /* actual level limited to max */
}

static void test_fade(void)
{
  int lvl;

  printf("fade time / limits\n");
  DTR0(1U);
  (void)send2(SA(5), 0x2E);                 /* fade time 1 = 0.707 s */
  CHECK_EQ(send(SA(5), 0xA5), 0x17);        /* fade time 1, rate 7 */
  (void)send(SA_DAPC(5), 100U);
  tick(300U);
  lvl = send(SA(5), 0xA0);
  CHECK((lvl > 100) && (lvl < 200));
  CHECK((send(SA(5), 0x90) & 0x10) != 0);   /* fade running */
  tick(500U);
  CHECK_EQ(send(SA(5), 0xA0), 100);
  CHECK((send(SA(5), 0x90) & 0x10) == 0);

  (void)send(SA_DAPC(5), 0U);               /* fade to off */
  tick(400U);
  CHECK(g_powerStage);
  tick(400U);
  CHECK(!g_powerStage);
  CHECK_EQ(send(SA(5), 0xA0), 0);

  DTR0(0U);
  (void)send2(SA(5), 0x2E);                 /* no fade */
  (void)send(SA_DAPC(5), 10U);              /* below min -> min */
  CHECK_EQ(send(SA(5), 0xA0), CFG_PHYSICAL_MIN_LEVEL);
  CHECK_EQ(send(SA(5), 0x94), 0xFF);        /* limit error */
  (void)send(SA_DAPC(5), 100U);
  CHECK_EQ(send(SA(5), 0x94), -1);

  (void)send(SA(5), 0x01);                  /* UP: 200 ms at 44.7 steps/s */
  tick(300U);
  lvl = send(SA(5), 0xA0);
  CHECK((lvl >= 108) && (lvl <= 110));
  (void)send(SA(5), 0x04);                  /* STEP DOWN */
  CHECK_EQ(send(SA(5), 0xA0), lvl - 1);
  (void)send(SA(5), 0x05);                  /* RECALL MAX */
  CHECK_EQ(send(SA(5), 0xA0), 200);
  (void)send(SA(5), 0x06);                  /* RECALL MIN */
  CHECK_EQ(send(SA(5), 0xA0), CFG_PHYSICAL_MIN_LEVEL);
  (void)send(SA(5), 0x07);                  /* STEP DOWN AND OFF */
  CHECK_EQ(send(SA(5), 0xA0), 0);
  (void)send(SA(5), 0x08);                  /* ON AND STEP UP */
  CHECK_EQ(send(SA(5), 0xA0), CFG_PHYSICAL_MIN_LEVEL);
  (void)send(SA(5), 0x0A);                  /* GO TO LAST ACTIVE LEVEL */
  CHECK_EQ(send(SA(5), 0xA0), CFG_PHYSICAL_MIN_LEVEL);
}

static void test_groups_scenes(void)
{
  printf("groups / scenes\n");
  (void)send2(SA(5), 0x63);                 /* ADD TO GROUP 3 */
  CHECK_EQ(send(SA(5), 0xC0), 0x08);
  (void)send((uint8_t)(0x80U | (3U << 1)), 150U);  /* group 3 DAPC */
  CHECK_EQ(send(SA(5), 0xA0), 150);
  (void)send((uint8_t)(0x80U | (4U << 1)), 120U);  /* group 4: not member */
  CHECK_EQ(send(SA(5), 0xA0), 150);
  (void)send2(SA(5), 0x73);                 /* REMOVE FROM GROUP 3 */
  CHECK_EQ(send(SA(5), 0xC0), 0x00);

  DTR0(120U);
  (void)send2(SA(5), 0x42);                 /* SET SCENE 2 */
  CHECK_EQ(send(SA(5), 0xB2), 120);
  CHECK_EQ(send(SA(5), 0xB3), 0xFF);
  (void)send(SA(5), 0x12);                  /* GO TO SCENE 2 */
  CHECK_EQ(send(SA(5), 0xA0), 120);
  (void)send(SA(5), 0x13);                  /* scene 3 = MASK: no change */
  CHECK_EQ(send(SA(5), 0xA0), 120);
}

static void test_device_types(void)
{
  printf("device types / DT6\n");
  CHECK_EQ(send(SA(5), 0x99), 0xFF);
  CHECK_EQ(send(SA(5), 0xA7), 6);
  CHECK_EQ(send(SA(5), 0xA7), 50);
  CHECK_EQ(send(SA(5), 0xA7), 51);
  CHECK_EQ(send(SA(5), 0xA7), 52);
  CHECK_EQ(send(SA(5), 0xA7), 0xFE);
  (void)send(SA(5), 0x91);
  CHECK_EQ(send(SA(5), 0xA7), -1);
  CHECK_EQ(send(SA(5), 0x9F), 6);           /* light source LED */

  CHECK_EQ(send(SA(5), 0xFF), -1);          /* no ENABLE DEVICE TYPE */
  (void)send(0xC1U, 6U);
  CHECK_EQ(send(SA(5), 0xFF), 0x08);        /* QUERY EXTENDED VERSION */
  (void)send(0xC1U, 6U);
  CHECK_EQ(send(SA(5), 0xF0), 0x0F);        /* features */
  (void)send(0xC1U, 8U);
  CHECK_EQ(send(SA(5), 0xFF), -1);          /* DT8 not supported */

  DTR0(1U);
  (void)send(0xC1U, 6U);
  (void)send2(SA(5), 0xE3);                 /* SELECT DIMMING CURVE linear */
  (void)send(0xC1U, 6U);
  CHECK_EQ(send(SA(5), 0xEE), 1);
  (void)send(SA_DAPC(5), 127U);
  CHECK((g_pwmDuty >= 2040U) && (g_pwmDuty <= 2056U));   /* 50 % */
  DTR0(0U);
  (void)send(0xC1U, 6U);
  (void)send2(SA(5), 0xE3);
  (void)send(SA_DAPC(5), 170U);             /* log: 10 % */
  CHECK((g_pwmDuty >= 3680U) && (g_pwmDuty <= 3692U));

  DTR0(4U);
  (void)send(0xC1U, 6U);
  (void)send2(SA(5), 0xE4);                 /* SET FAST FADE TIME 100 ms */
  (void)send(0xC1U, 6U);
  CHECK_EQ(send(SA(5), 0xFD), 4);
  (void)send(SA_DAPC(5), 100U);
  tick(110U);
  CHECK_EQ(send(SA(5), 0xA0), 100);
  DTR0(0U);
  (void)send(0xC1U, 6U);
  (void)send2(SA(5), 0xE4);

  s_fail = MON_FAIL_OPEN_CIRCUIT;
  (void)send(0xC1U, 6U);
  CHECK_EQ(send(SA(5), 0xF1), 0x02);        /* failure status */
  CHECK_EQ(send(SA(5), 0x92), 0xFF);        /* lamp failure */
  s_fail = 0U;
}

static void test_memory_banks(void)
{
  printf("memory banks 0 / 1 / 202 / 206\n");
  DTR1(0U);
  DTR0(0U);
  CHECK_EQ(send(SA(5), 0xC5), 0x1A);        /* MB0 last location */
  CHECK_EQ(send(SA(5), 0xC5), -1);          /* 0x01 reserved */
  CHECK_EQ(send(SA(5), 0xC5), 206);         /* last accessible bank */
  CHECK_EQ(send(SA(5), 0x98), 3);           /* DTR0 incremented */
  DTR0(0x15U);
  CHECK_EQ(send(SA(5), 0xC5), 0x08);        /* 101 version */
  CHECK_EQ(send(SA(5), 0xC5), 0x08);        /* 102 version */
  DTR0(0x1BU);
  CHECK_EQ(send(SA(5), 0xC5), -1);          /* beyond last location */
  DTR1(3U);
  DTR0(0U);
  CHECK_EQ(send(SA(5), 0xC5), -1);          /* bank 3 not implemented */
  CHECK_EQ(send(SA(5), 0x98), 0);

  /* bank 1: lock, write, read back, persistence */
  DTR1(1U);
  DTR0(3U);
  (void)send2(SA(5), 0x81);                 /* ENABLE WRITE MEMORY */
  CHECK_EQ(send(0xC7U, 0x12U), -1);         /* locked */
  DTR0(2U);
  CHECK_EQ(send(0xC7U, 0x55U), 0x55);       /* unlock */
  CHECK_EQ(send(0xC7U, 0x12U), 0x12);       /* location 3 */
  CHECK_EQ(send(0xC7U, 0x34U), 0x34);       /* location 4 */
  DTR0(3U);
  CHECK_EQ(send(SA(5), 0xC5), 0x12);
  CHECK_EQ(send(SA(5), 0xC5), 0x34);
  CHECK_EQ(send(0xC7U, 0x56U), 0x56);       /* READ MEMORY keeps write enabled */
  (void)send(SA(5), 0x90);                  /* any other command disables it */
  CHECK_EQ(send(0xC7U, 0x99U), -1);
  DTR0(0x11U);
  CHECK_EQ(send(SA(5), 0xC5), 0x00);        /* content format ID 0x0003 */
  CHECK_EQ(send(SA(5), 0xC5), 0x03);
  tick(1100U);
  MB_Service();
  CHECK_EQ(g_eeprom[0x080], 0x12);
  CHECK_EQ(g_eeprom[0x081], 0x34);

  /* bank 202: latch */
  s_cnt.activeEnergyWh = 0x01020304U;
  DTR1(202U);
  DTR0(2U);
  (void)send2(SA(5), 0x81);
  CHECK_EQ(send(0xC9U, 0xAAU), -1);         /* latch, no reply */
  s_cnt.activeEnergyWh = 0U;                /* changes after the latch */
  DTR0(3U);
  CHECK_EQ(send(SA(5), 0xC5), 0x01);        /* version */
  CHECK_EQ(send(SA(5), 0xC5), 0x00);        /* scale Wh */
  CHECK_EQ(send(SA(5), 0xC5), 0x00);
  CHECK_EQ(send(SA(5), 0xC5), 0x00);
  CHECK_EQ(send(SA(5), 0xC5), 0x01);
  CHECK_EQ(send(SA(5), 0xC5), 0x02);
  CHECK_EQ(send(SA(5), 0xC5), 0x03);
  CHECK_EQ(send(SA(5), 0xC5), 0x04);
  CHECK_EQ(send(SA(5), 0xC5), 0xFF);        /* power scale 10^-1 */
  DTR0(0x0FU);
  CHECK_EQ(send(SA(5), 0xC5), 0x9B);        /* 667 (0x029B) x 0.1 W */

  /* bank 205 / 206 */
  DTR1(205U);
  DTR0(0x1BU);
  CHECK_EQ(send(SA(5), 0xC5), 100);         /* 40 degC + 60 */
  DTR0(0x0BU);
  CHECK_EQ(send(SA(5), 0xC5), 0xFF);        /* supply voltage: MASK */
  DTR1(206U);
  DTR0(0x12U);
  CHECK_EQ(send(SA(5), 0xC5), 0x04);        /* 1200 x 0.1 V */
  CHECK_EQ(send(SA(5), 0xC5), 0xB0);
  CHECK_EQ(send(SA(5), 0xC5), 0x01);        /* 500 mA */
  CHECK_EQ(send(SA(5), 0xC5), 0xF4);
  s_cnt.lsOnTimeResS = 1000U;
  DTR0(2U);
  (void)send2(SA(5), 0x81);
  (void)send(0xC9U, 0x55U);                 /* unlock bank 206 */
  DTR0(206U);
  (void)send2(SA(5), 0x24);                 /* RESET MEMORY BANK 206 */
  CHECK_EQ(s_cnt.lsOnTimeResS, 0);
}

static void test_reset_and_nvm(void)
{
  printf("reset / persistence\n");
  DTR0(100U);
  (void)send2(SA(5), 0x2C);                 /* SYSTEM FAILURE LEVEL 100 */
  CHECK_EQ(send(SA(5), 0x95), -1);          /* not in reset state */
  tick(CFG_NVM_SAVE_DELAY_MS + 10U);
  Gear_NvmService();

  /* reboot: variables come back from the EEPROM */
  Gear_Init();
  Gear_PowerOn();
  tick(5U);
  CHECK_EQ(send(SA(5), 0xA4), 100);
  CHECK_EQ(send(SA(5), 0xA1), 200);
  CHECK_EQ(send(SA(5), 0x9B), 0xFF);        /* power cycle seen */

  printf("system failure\n");
  s_busIdle = false;
  tick(600U);
  s_busIdle = true;
  tick(5U);
  CHECK_EQ(send(SA(5), 0xA0), 100);

  printf("identify\n");
  (void)send2(SA(5), 0x25);
  tick(100U);
  CHECK_EQ(g_pwmDuty, 0);                   /* first half: max */
  tick(500U);
  CHECK(g_pwmDuty > 3000U);                 /* second half: min */
  tick(10000U);
  CHECK_EQ(send(SA(5), 0xA0), 100);

  printf("RESET\n");
  (void)send2(SA(5), 0x20);
  CHECK_EQ(send(SA(5), 0x95), 0xFF);        /* reset state */
  CHECK_EQ(send(SA(5), 0xA0), 254);
  CHECK_EQ(send(SA(5), 0xA4), 254);
  CHECK_EQ(send(SA(5), 0xA2), CFG_PHYSICAL_MIN_LEVEL);
  CHECK_EQ(send(SA(5), 0x91), 0xFF);        /* short address kept */

  DTR0(0xFFU);
  (void)send2(SA(5), 0x80);                 /* delete short address */
  CHECK_EQ(send(BC, 0x96), 0xFF);
}

static void test_variant(void)
{
  uint8_t rec[8];
  uint8_t v = 0U;

  printf("variant record / NFC\n");
  Variant_EncodeRecord(7U, rec);
  CHECK(Variant_DecodeRecord(rec, &v));
  CHECK_EQ(v, 7);
  rec[5] = 8U;
  CHECK(!Variant_DecodeRecord(rec, &v));    /* CRC */
  Variant_EncodeRecord(17U, rec);
  CHECK(!Variant_DecodeRecord(rec, &v));    /* out of range */

  Variant_EncodeRecord(7U, rec);
  memcpy(&g_tag[CFG_NFC_VARIANT_BLOCK * 4U], rec, 8U);
  Variant_ApplyAtBoot();
  CHECK_EQ(Variant_GetActive(), 7);
  CHECK_EQ(g_shiftReg, 0x0F);
  CHECK_EQ(g_eeprom[0x105], 7);
  CHECK_EQ(g_eeprom[0x115], 7);

  /* runtime write: stored now, applied at next boot */
  Variant_EncodeRecord(16U, rec);
  memcpy(&g_tag[CFG_NFC_VARIANT_BLOCK * 4U], rec, 8U);
  Variant_NfcEventIrq();
  g_tickMs += CFG_NFC_EVENT_SETTLE_MS;
  Variant_Service();
  CHECK_EQ(Variant_GetStored(), 16);
  CHECK_EQ(Variant_GetActive(), 7);
  CHECK_EQ(g_shiftReg, 0x0F);
  Variant_ApplyAtBoot();
  CHECK_EQ(g_shiftReg, 0x21);

  /* corrupted tag record is ignored */
  g_tag[CFG_NFC_VARIANT_BLOCK * 4U + 6U] ^= 0x01U;
  g_tickMs += CFG_NFC_POLL_PERIOD_MS;
  Variant_Service();
  CHECK_EQ(Variant_GetStored(), 16);
}

int main(void)
{
  Mock_Reset();
  boot();

  test_power_on();
  test_off_on();
  test_addressing();
  test_send_twice();
  test_fade();
  test_groups_scenes();
  test_device_types();
  test_memory_banks();
  test_reset_and_nvm();
  test_variant();

  printf("\ntest_gear: %d checks, %d failures\n", g_checks, g_failures);
  return (g_failures == 0) ? 0 : 1;
}

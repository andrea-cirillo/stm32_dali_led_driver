/**
 ******************************************************************************
 * @file    dali_gear.c
 * @brief   DALI-2 control gear (IEC 62386-102 + 207 device type 6).
 ******************************************************************************
 */
#include "dali_gear.h"
#include <stdbool.h>
#include "app_config.h"
#include "bsp.h"
#include "crc16.h"
#include "dali_phy.h"
#include "dali_membank.h"
#include "led_monitor.h"
#include "light_ctrl.h"
#include "nvm.h"

/* ---- constants ------------------------------------------------------------ */
#define MASK                   (0xFFU)
#define YES                    (0xFFU)
#define LEVEL_MAX              (254U)
#define VERSION_2_0            (0x08U)
#define DT_LED                 (6U)
#define DT_MULTIPLE            (0xFFU)
#define DT_NO_MORE             (0xFEU)
#define DT_NONE                (0xFFU)   /* no ENABLE DEVICE TYPE pending   */
#define LIGHT_SOURCE_LED       (6U)

#define SEND_TWICE_MS          (100U)
#define INIT_TIMEOUT_MS        (15UL * 60UL * 1000UL)
#define IDENTIFY_MS            (10000U)
#define DAPC_SEQUENCE_MS       (200U)
#define UP_DOWN_MS             (200U)
#define SYS_FAILURE_MS         (500U)
#define NVM_RESET_ADDR         (0xFFFFFFUL)

/* DT6 (IEC 62386-207) */
#define DT6_GEAR_TYPE          (0x05U)   /* integrated LED power supply, AC */
#define DT6_FEATURES           (0x0FU)   /* short, open, load dec., load inc. */
#define DT6_MAX_FAST_FADE      (27U)

typedef enum
{
  INIT_DISABLED = 0,
  INIT_ENABLED,
  INIT_WITHDRAWN
} InitState_t;

/* Device types reported by QUERY NEXT DEVICE TYPE (ascending) */
static const uint8_t k_deviceTypes[] = { DT_LED, 50U, 51U, 52U };
#define DEVICE_TYPE_COUNT      ((uint8_t)(sizeof(k_deviceTypes) / sizeof(k_deviceTypes[0])))

/* Fade time T(n) = 0.5 * sqrt(2^n) s, n = 1..15 (ms) */
static const uint32_t k_fadeTimeMs[15] =
{
  707U, 1000U, 1414U, 2000U, 2828U, 4000U, 5657U, 8000U,
  11314U, 16000U, 22627U, 32000U, 45255U, 64000U, 90510U
};

/* Fade rate F(n) = 506 / sqrt(2^n) steps/s, n = 1..15, in Q16 levels per ms */
static const uint16_t k_fadeRateQ16[15] =
{
  23449U, 16581U, 11724U, 8290U, 5862U, 4145U, 2931U, 2073U,
  1466U, 1036U, 733U, 518U, 366U, 259U, 183U
};

/* Extended fade time multipliers (ms): 0, 100 ms, 1 s, 10 s, 1 min */
static const uint32_t k_extFadeMultMs[5] = { 0U, 100U, 1000U, 10000U, 60000U };

/* ---- state (SysTick context) ---------------------------------------------- */
static NVM_Gear_t s_nv;
static volatile bool s_nvDirty = false;
static volatile bool s_nvSaveNow = false;
static volatile uint32_t s_nvDirtyMs = 0U;

static uint8_t  s_dtr0 = 0U;
static uint8_t  s_dtr1 = 0U;
static uint8_t  s_dtr2 = 0U;
static uint32_t s_searchAddr = NVM_RESET_ADDR;
static InitState_t s_initState = INIT_DISABLED;
static uint32_t s_initTimerMs = 0U;
static bool     s_writeEnable = false;
static uint8_t  s_edt = DT_NONE;
static bool     s_powerCycleSeen = true;
static bool     s_limitError = false;
static uint8_t  s_lastActiveLevel = LEVEL_MAX;
static bool     s_qndtValid = false;
static uint8_t  s_qndtIdx = 0U;
static uint16_t s_dapcSeqMs = 0U;
static uint16_t s_identifyMs = 0U;
static uint16_t s_busLowMs = 0U;
static bool     s_sysFailure = false;
static uint32_t s_rng = 0x1234567UL;

/* send-twice pending frame */
static bool     s_pendValid = false;
static uint16_t s_pendFrame = 0U;
static uint8_t  s_pendEdt = DT_NONE;
static uint32_t s_pendMs = 0U;

/* ---- helpers --------------------------------------------------------------- */

static void nv_dirty(void)
{
  s_nvDirty = true;
  s_nvDirtyMs = BSP_GetTickMs();
}

static void reply(uint8_t value)
{
  DALI_PHY_SendBackward(value);
}

static void reply_yes_if(bool cond)
{
  if (cond)
  {
    reply(YES);
  }
}

static uint32_t random_address(void)
{
  uint32_t r;
  /* xorshift32, re-seeded with the arrival time of the command */
  s_rng ^= (uint32_t)DALI_PHY_GetTimeUs() << 7U;
  if (s_rng == 0U)
  {
    s_rng = 0x2545F491UL;                        /* xorshift must not be 0 */
  }
  do
  {
    s_rng ^= s_rng << 13U;
    s_rng ^= s_rng >> 17U;
    s_rng ^= s_rng << 5U;
    r = (s_rng ^ (s_rng >> 8U)) & 0xFFFFFFUL;
  } while (r == 0xFFFFFFUL);
  return r;
}

static uint32_t get_random_addr(void)
{
  return ((uint32_t)s_nv.randomAddress[0] << 16U) |
         ((uint32_t)s_nv.randomAddress[1] << 8U) |
         (uint32_t)s_nv.randomAddress[2];
}

static void set_random_addr(uint32_t a)
{
  s_nv.randomAddress[0] = (uint8_t)(a >> 16U);
  s_nv.randomAddress[1] = (uint8_t)(a >> 8U);
  s_nv.randomAddress[2] = (uint8_t)a;
  nv_dirty();
}

static uint16_t get_groups(void)
{
  return (uint16_t)(((uint16_t)s_nv.groups[1] << 8U) | s_nv.groups[0]);
}

static void set_groups(uint16_t g)
{
  s_nv.groups[0] = (uint8_t)g;
  s_nv.groups[1] = (uint8_t)(g >> 8U);
  nv_dirty();
}

static void set_nv(uint8_t *var, uint8_t value)
{
  if (*var != value)
  {
    *var = value;
    nv_dirty();
  }
}

static uint32_t fade_time_ms(void)
{
  uint32_t ms;

  if (s_nv.fadeTime != 0U)
  {
    ms = k_fadeTimeMs[s_nv.fadeTime - 1U];
  }
  else if (s_nv.fastFadeTime != 0U)
  {
    ms = 25UL * s_nv.fastFadeTime;               /* IEC 62386-207 */
  }
  else
  {
    const uint32_t base = ((uint32_t)s_nv.extFadeTime & 0x0FU) + 1U;
    const uint32_t mult = ((uint32_t)s_nv.extFadeTime >> 4U) & 0x07U;
    ms = (mult < 5U) ? (base * k_extFadeMultMs[mult]) : 0U;
  }
  return ms;
}

static uint32_t fade_rate_q16(void)
{
  const uint8_t r = ((s_nv.fadeRate >= 1U) && (s_nv.fadeRate <= 15U)) ? s_nv.fadeRate : 7U;
  return k_fadeRateQ16[r - 1U];
}

/* Clamps a requested level to [min, max] (0 = off) and sets limitError. */
static uint8_t clamp_level(uint8_t level)
{
  uint8_t l = level;

  s_limitError = false;
  if (l == 0U)
  {
    /* off */
  }
  else if (l < s_nv.minLevel)
  {
    l = s_nv.minLevel;
    s_limitError = true;
  }
  else if (l > s_nv.maxLevel)
  {
    l = s_nv.maxLevel;
    s_limitError = true;
  }
  else
  {
    /* in range */
  }
  return l;
}

/* Arc power change towards an already clamped level. */
static void go_to(uint8_t level, uint32_t fadeMs)
{
  Light_SetIdentify(false, s_nv.minLevel);
  s_identifyMs = 0U;
  Light_FadeTo(level, fadeMs, s_nv.minLevel);
  if (level != 0U)
  {
    s_lastActiveLevel = level;
  }
  s_powerCycleSeen = false;
}

static bool is_reset_state(void)
{
  uint32_t i;
  bool r = (s_nv.powerOnLevel == LEVEL_MAX) && (s_nv.systemFailureLevel == LEVEL_MAX) &&
           (s_nv.minLevel == CFG_PHYSICAL_MIN_LEVEL) && (s_nv.maxLevel == LEVEL_MAX) &&
           (s_nv.fadeRate == 7U) && (s_nv.fadeTime == 0U) && (s_nv.extFadeTime == 0U) &&
           (s_searchAddr == NVM_RESET_ADDR) && (get_random_addr() == NVM_RESET_ADDR) &&
           (get_groups() == 0U) && (s_nv.dimmingCurve == 0U) && (s_nv.fastFadeTime == 0U);

  for (i = 0U; (i < 16U) && r; i++)
  {
    r = (s_nv.scene[i] == MASK);
  }
  return r;
}

static void set_reset_values(void)
{
  uint32_t i;

  s_nv.powerOnLevel = LEVEL_MAX;
  s_nv.systemFailureLevel = LEVEL_MAX;
  s_nv.minLevel = CFG_PHYSICAL_MIN_LEVEL;
  s_nv.maxLevel = LEVEL_MAX;
  s_nv.fadeRate = 7U;
  s_nv.fadeTime = 0U;
  s_nv.extFadeTime = 0U;
  s_nv.operatingMode = 0U;
  s_nv.lastLightLevel = LEVEL_MAX;
  s_nv.dimmingCurve = 0U;
  s_nv.fastFadeTime = 0U;
  for (i = 0U; i < 16U; i++)
  {
    s_nv.scene[i] = MASK;
  }
  s_nv.groups[0] = 0U;
  s_nv.groups[1] = 0U;
  set_random_addr(NVM_RESET_ADDR);
  s_searchAddr = NVM_RESET_ADDR;
  s_lastActiveLevel = LEVEL_MAX;
  Light_SetCurve(LIGHT_CURVE_LOGARITHMIC);
  nv_dirty();
}

static uint8_t query_status(void)
{
  uint8_t st = 0U;

  if (Monitor_GetFailureStatus() != 0U)
  {
    st |= 0x02U;                                 /* lampFailure          */
  }
  if (Light_IsLampOn())
  {
    st |= 0x04U;                                 /* lampOn               */
  }
  if (s_limitError)
  {
    st |= 0x08U;
  }
  if (Light_IsFading())
  {
    st |= 0x10U;
  }
  if (is_reset_state())
  {
    st |= 0x20U;
  }
  if (s_nv.shortAddress == MASK)
  {
    st |= 0x40U;
  }
  if (s_powerCycleSeen)
  {
    st |= 0x80U;
  }
  return st;
}

static bool is_addressed(uint8_t a)
{
  bool r;

  if ((a & 0x80U) == 0U)
  {
    r = (s_nv.shortAddress != MASK) && (((a >> 1U) & 0x3FU) == s_nv.shortAddress);
  }
  else if ((a & 0xE0U) == 0x80U)
  {
    r = ((get_groups() >> ((a >> 1U) & 0x0FU)) & 1U) != 0U;
  }
  else if ((a & 0xFEU) == 0xFEU)
  {
    r = true;                                    /* broadcast            */
  }
  else if ((a & 0xFEU) == 0xFCU)
  {
    r = (s_nv.shortAddress == MASK);             /* broadcast unaddressed */
  }
  else
  {
    r = false;
  }
  return r;
}

static bool is_special(uint8_t a)
{
  return (a >= 0xA0U) && (a <= 0xCBU);
}

static bool needs_twice(uint8_t a, uint8_t d, uint8_t edt)
{
  bool r = false;

  if (is_special(a))
  {
    r = (a == 0xA5U) || (a == 0xA7U);            /* INITIALISE, RANDOMISE */
  }
  else if ((a & 1U) != 0U)
  {
    r = ((d >= 0x20U) && (d <= 0x81U)) ||
        ((edt == DT_LED) && (d >= 0xE0U) && (d <= 0xE4U));
  }
  else
  {
    r = false;                                   /* DAPC                 */
  }
  return r;
}

/* ---- memory bank access ----------------------------------------------------- */

static void inc_dtr0(void)
{
  if (s_dtr0 < 0xFFU)
  {
    s_dtr0++;
  }
}

static void read_memory(void)
{
  if (MB_IsImplemented(s_dtr1) && (s_dtr0 <= MB_LastLocation(s_dtr1)))
  {
    uint8_t v = 0U;
    if (MB_Read(s_dtr1, s_dtr0, &v))
    {
      reply(v);
    }
    inc_dtr0();
  }
}

static void write_memory(uint8_t value, bool answer)
{
  if (s_writeEnable && MB_IsImplemented(s_dtr1) && (s_dtr0 <= MB_LastLocation(s_dtr1)))
  {
    if (MB_Write(s_dtr1, s_dtr0, value) && answer)
    {
      reply(value);
    }
    inc_dtr0();
  }
}

/* ---- command handlers --------------------------------------------------------- */

static void cmd_dapc(uint8_t level)
{
  if (level == MASK)
  {
    Light_StopFade();
  }
  else
  {
    go_to(clamp_level(level), (s_dapcSeqMs != 0U) ? DAPC_SEQUENCE_MS : fade_time_ms());
    if (s_dapcSeqMs != 0U)
    {
      s_dapcSeqMs = DAPC_SEQUENCE_MS;
    }
  }
}

static void cmd_up_down(bool up)
{
  const uint8_t actual = Light_GetActualLevel();
  const uint32_t rate = fade_rate_q16();
  const uint32_t steps = ((rate * UP_DOWN_MS) + 0xFFFFUL) >> 16U;

  if (actual != 0U)
  {
    uint32_t target;
    if (up)
    {
      target = actual + steps;
      target = (target > s_nv.maxLevel) ? s_nv.maxLevel : target;
    }
    else
    {
      target = (actual > (s_nv.minLevel + steps)) ? (actual - steps) : s_nv.minLevel;
    }
    Light_SetIdentify(false, s_nv.minLevel);
    s_identifyMs = 0U;
    Light_FadeAtRate((uint8_t)target, rate);
    s_lastActiveLevel = (uint8_t)target;
    s_powerCycleSeen = false;
  }
}

static void cmd_arc(uint8_t d)
{
  const uint8_t actual = Light_GetActualLevel();

  switch (d)
  {
    case 0x00U:                                  /* OFF                  */
      go_to(0U, 0U);
      break;
    case 0x01U:                                  /* UP                   */
      cmd_up_down(true);
      break;
    case 0x02U:                                  /* DOWN                 */
      cmd_up_down(false);
      break;
    case 0x03U:                                  /* STEP UP              */
      if ((actual != 0U) && (actual < s_nv.maxLevel))
      {
        go_to((uint8_t)(actual + 1U), 0U);
      }
      break;
    case 0x04U:                                  /* STEP DOWN            */
      if ((actual != 0U) && (actual > s_nv.minLevel))
      {
        go_to((uint8_t)(actual - 1U), 0U);
      }
      break;
    case 0x05U:                                  /* RECALL MAX LEVEL     */
      go_to(s_nv.maxLevel, 0U);
      break;
    case 0x06U:                                  /* RECALL MIN LEVEL     */
      go_to(s_nv.minLevel, 0U);
      break;
    case 0x07U:                                  /* STEP DOWN AND OFF    */
      if (actual != 0U)
      {
        go_to((actual <= s_nv.minLevel) ? 0U : (uint8_t)(actual - 1U), 0U);
      }
      break;
    case 0x08U:                                  /* ON AND STEP UP       */
      if (actual == 0U)
      {
        go_to(s_nv.minLevel, 0U);
      }
      else if (actual < s_nv.maxLevel)
      {
        go_to((uint8_t)(actual + 1U), 0U);
      }
      else
      {
        /* already at max */
      }
      break;
    case 0x09U:                                  /* ENABLE DAPC SEQUENCE */
      s_dapcSeqMs = DAPC_SEQUENCE_MS;
      break;
    case 0x0AU:                                  /* GO TO LAST ACTIVE LEVEL */
      go_to(clamp_level(s_lastActiveLevel), fade_time_ms());
      break;
    case 0x0BU:                                  /* CONTINUOUS UP        */
    case 0x0CU:                                  /* CONTINUOUS DOWN      */
      if (actual != 0U)
      {
        Light_SetIdentify(false, s_nv.minLevel);
        s_identifyMs = 0U;
        Light_FadeAtRate((d == 0x0BU) ? s_nv.maxLevel : s_nv.minLevel, fade_rate_q16());
        s_powerCycleSeen = false;
      }
      break;
    default:                                     /* 0x10..0x1F GO TO SCENE */
      if ((d >= 0x10U) && (d <= 0x1FU) && (s_nv.scene[d & 0x0FU] != MASK))
      {
        go_to(clamp_level(s_nv.scene[d & 0x0FU]), fade_time_ms());
      }
      break;
  }
}

static void cmd_config(uint8_t d)
{
  uint8_t v;

  if ((d >= 0x40U) && (d <= 0x4FU))
  {
    set_nv(&s_nv.scene[d & 0x0FU], s_dtr0);     /* SET SCENE            */
  }
  else if ((d >= 0x50U) && (d <= 0x5FU))
  {
    set_nv(&s_nv.scene[d & 0x0FU], MASK);       /* REMOVE FROM SCENE    */
  }
  else if ((d >= 0x60U) && (d <= 0x6FU))
  {
    set_groups((uint16_t)(get_groups() | (uint16_t)(1UL << (d & 0x0FU))));
  }
  else if ((d >= 0x70U) && (d <= 0x7FU))
  {
    set_groups((uint16_t)(get_groups() & (uint16_t)~(1UL << (d & 0x0FU))));
  }
  else
  {
    switch (d)
    {
      case 0x20U:                                /* RESET                */
        set_reset_values();
        go_to(LEVEL_MAX, 0U);
        break;
      case 0x21U:                                /* STORE ACTUAL LEVEL IN DTR0 */
        s_dtr0 = Light_GetActualLevel();
        break;
      case 0x22U:                                /* SAVE PERSISTENT VARIABLES */
        s_nvSaveNow = true;
        break;
      case 0x23U:                                /* SET OPERATING MODE   */
        if (s_dtr0 == 0U)
        {
          set_nv(&s_nv.operatingMode, 0U);       /* only standard mode   */
        }
        break;
      case 0x24U:                                /* RESET MEMORY BANK    */
        MB_Reset(s_dtr0);
        break;
      case 0x25U:                                /* IDENTIFY DEVICE      */
        Light_SetIdentify(true, s_nv.minLevel);
        s_identifyMs = IDENTIFY_MS;
        break;
      case 0x2AU:                                /* SET MAX LEVEL        */
        v = (s_dtr0 < s_nv.minLevel) ? s_nv.minLevel : ((s_dtr0 > LEVEL_MAX) ? LEVEL_MAX : s_dtr0);
        set_nv(&s_nv.maxLevel, v);
        if (Light_GetActualLevel() > v)
        {
          go_to(v, 0U);
        }
        break;
      case 0x2BU:                                /* SET MIN LEVEL        */
        v = (s_dtr0 < CFG_PHYSICAL_MIN_LEVEL) ? CFG_PHYSICAL_MIN_LEVEL : s_dtr0;
        v = (v > s_nv.maxLevel) ? s_nv.maxLevel : v;
        set_nv(&s_nv.minLevel, v);
        if ((Light_GetActualLevel() != 0U) && (Light_GetActualLevel() < v))
        {
          go_to(v, 0U);
        }
        break;
      case 0x2CU:                                /* SET SYSTEM FAILURE LEVEL */
        set_nv(&s_nv.systemFailureLevel, s_dtr0);
        break;
      case 0x2DU:                                /* SET POWER ON LEVEL   */
        set_nv(&s_nv.powerOnLevel, s_dtr0);
        break;
      case 0x2EU:                                /* SET FADE TIME        */
        set_nv(&s_nv.fadeTime, (s_dtr0 > 15U) ? 15U : s_dtr0);
        break;
      case 0x2FU:                                /* SET FADE RATE        */
        v = (s_dtr0 > 15U) ? 15U : s_dtr0;
        set_nv(&s_nv.fadeRate, (v == 0U) ? 1U : v);
        break;
      case 0x30U:                                /* SET EXTENDED FADE TIME */
        set_nv(&s_nv.extFadeTime, (s_dtr0 > 0x4FU) ? 0U : s_dtr0);
        break;
      case 0x80U:                                /* SET SHORT ADDRESS    */
        if (s_dtr0 == MASK)
        {
          set_nv(&s_nv.shortAddress, MASK);
        }
        else if ((s_dtr0 & 0x81U) == 0x01U)
        {
          set_nv(&s_nv.shortAddress, (uint8_t)(s_dtr0 >> 1U));
        }
        else
        {
          /* invalid: ignore */
        }
        break;
      case 0x81U:                                /* ENABLE WRITE MEMORY  */
        s_writeEnable = true;
        break;
      default:
        /* reserved */
        break;
    }
  }
}

static void cmd_query(uint8_t d)
{
  if ((d >= 0xB0U) && (d <= 0xBFU))
  {
    reply(s_nv.scene[d & 0x0FU]);                /* QUERY SCENE LEVEL    */
  }
  else
  {
    switch (d)
    {
      case 0x90U: reply(query_status()); break;
      case 0x91U: reply(YES); break;            /* CONTROL GEAR PRESENT */
      case 0x92U: reply_yes_if(Monitor_GetFailureStatus() != 0U); break;
      case 0x93U: reply_yes_if(Light_IsLampOn()); break;
      case 0x94U: reply_yes_if(s_limitError); break;
      case 0x95U: reply_yes_if(is_reset_state()); break;
      case 0x96U: reply_yes_if(s_nv.shortAddress == MASK); break;
      case 0x97U: reply(VERSION_2_0); break;
      case 0x98U: reply(s_dtr0); break;
      case 0x99U:                                /* QUERY DEVICE TYPE    */
        reply(DT_MULTIPLE);
        s_qndtIdx = 0U;
        s_qndtValid = true;
        break;
      case 0x9AU: reply(CFG_PHYSICAL_MIN_LEVEL); break;
      case 0x9BU: reply_yes_if(s_powerCycleSeen); break;
      case 0x9CU: reply(s_dtr1); break;
      case 0x9DU: reply(s_dtr2); break;
      case 0x9EU: reply(s_nv.operatingMode); break;
      case 0x9FU: reply(LIGHT_SOURCE_LED); break;
      case 0xA0U: reply(Light_GetActualLevel()); break;
      case 0xA1U: reply(s_nv.maxLevel); break;
      case 0xA2U: reply(s_nv.minLevel); break;
      case 0xA3U: reply(s_nv.powerOnLevel); break;
      case 0xA4U: reply(s_nv.systemFailureLevel); break;
      case 0xA5U: reply((uint8_t)((uint8_t)(s_nv.fadeTime << 4U) | s_nv.fadeRate)); break;
      case 0xA6U: reply_yes_if(s_nv.operatingMode >= 0x80U); break;
      case 0xA7U:                                /* QUERY NEXT DEVICE TYPE */
        if (s_qndtValid)
        {
          reply((s_qndtIdx < DEVICE_TYPE_COUNT) ? k_deviceTypes[s_qndtIdx] : DT_NO_MORE);
          if (s_qndtIdx < DEVICE_TYPE_COUNT)
          {
            s_qndtIdx++;
          }
        }
        break;
      case 0xA8U: reply(s_nv.extFadeTime); break;
      case 0xAAU: break;                         /* CONTROL GEAR FAILURE: NO */
      case 0xC0U: reply(s_nv.groups[0]); break;
      case 0xC1U: reply(s_nv.groups[1]); break;
      case 0xC2U: reply(s_nv.randomAddress[0]); break;
      case 0xC3U: reply(s_nv.randomAddress[1]); break;
      case 0xC4U: reply(s_nv.randomAddress[2]); break;
      case 0xC5U: read_memory(); break;          /* READ MEMORY LOCATION */
      default: break;                            /* reserved             */
    }
  }
}

/* IEC 62386-207 ed.2 application extended commands */
static void cmd_dt6(uint8_t d)
{
  switch (d)
  {
    case 0xE3U:                                  /* SELECT DIMMING CURVE */
      if (s_dtr0 <= 1U)
      {
        set_nv(&s_nv.dimmingCurve, s_dtr0);
        Light_SetCurve(s_dtr0);
      }
      break;
    case 0xE4U:                                  /* SET FAST FADE TIME   */
      if (s_dtr0 == 0U)
      {
        set_nv(&s_nv.fastFadeTime, 0U);
      }
      else if (s_dtr0 <= DT6_MAX_FAST_FADE)
      {
        set_nv(&s_nv.fastFadeTime, (s_dtr0 < CFG_MIN_FAST_FADE_TIME) ? CFG_MIN_FAST_FADE_TIME : s_dtr0);
      }
      else
      {
        /* out of range: ignore */
      }
      break;
    case 0xEDU: reply(DT6_GEAR_TYPE); break;     /* QUERY CONTROL GEAR TYPE */
    case 0xEEU: reply(s_nv.dimmingCurve); break; /* QUERY DIMMING CURVE  */
    case 0xF0U: reply(DT6_FEATURES); break;      /* QUERY FEATURES       */
    case 0xF1U: reply(Monitor_GetFailureStatus()); break; /* QUERY FAILURE STATUS */
    case 0xFDU: reply(s_nv.fastFadeTime); break; /* QUERY FAST FADE TIME */
    case 0xFEU: reply(CFG_MIN_FAST_FADE_TIME); break;
    case 0xFFU: reply(VERSION_2_0); break;       /* QUERY EXTENDED VERSION NUMBER */
    default: break;
  }
}

static void cmd_special(uint8_t a, uint8_t d)
{
  const bool initOn = (s_initState != INIT_DISABLED);
  const bool selected = initOn && (get_random_addr() == s_searchAddr);

  switch (a)
  {
    case 0xA1U:                                  /* TERMINATE            */
      s_initState = INIT_DISABLED;
      Light_SetIdentify(false, s_nv.minLevel);
      s_identifyMs = 0U;
      break;
    case 0xA3U: s_dtr0 = d; break;               /* DTR0                 */
    case 0xA5U:                                  /* INITIALISE           */
      if ((d == 0x00U) ||
          ((d == 0xFFU) && (s_nv.shortAddress == MASK)) ||
          (((d & 0x81U) == 0x01U) && (s_nv.shortAddress == (uint8_t)(d >> 1U))))
      {
        s_initState = INIT_ENABLED;
        s_initTimerMs = INIT_TIMEOUT_MS;
      }
      break;
    case 0xA7U:                                  /* RANDOMISE            */
      if (initOn)
      {
        set_random_addr(random_address());
      }
      break;
    case 0xA9U:                                  /* COMPARE              */
      reply_yes_if((s_initState == INIT_ENABLED) && (get_random_addr() <= s_searchAddr));
      break;
    case 0xABU:                                  /* WITHDRAW             */
      if ((s_initState == INIT_ENABLED) && selected)
      {
        s_initState = INIT_WITHDRAWN;
      }
      break;
    case 0xADU: break;                           /* PING                 */
    case 0xB1U:
      s_searchAddr = (s_searchAddr & 0x00FFFFUL) | ((uint32_t)d << 16U);
      break;
    case 0xB3U:
      s_searchAddr = (s_searchAddr & 0xFF00FFUL) | ((uint32_t)d << 8U);
      break;
    case 0xB5U:
      s_searchAddr = (s_searchAddr & 0xFFFF00UL) | (uint32_t)d;
      break;
    case 0xB7U:                                  /* PROGRAM SHORT ADDRESS */
      if (selected)
      {
        if (d == MASK)
        {
          set_nv(&s_nv.shortAddress, MASK);
        }
        else if ((d & 0x81U) == 0x01U)
        {
          set_nv(&s_nv.shortAddress, (uint8_t)(d >> 1U));
        }
        else
        {
          /* invalid */
        }
      }
      break;
    case 0xB9U:                                  /* VERIFY SHORT ADDRESS */
      reply_yes_if(initOn && ((d & 0x81U) == 0x01U) && (s_nv.shortAddress == (uint8_t)(d >> 1U)));
      break;
    case 0xBBU:                                  /* QUERY SHORT ADDRESS  */
      if (selected)
      {
        reply((s_nv.shortAddress == MASK) ? MASK : (uint8_t)((uint8_t)(s_nv.shortAddress << 1U) | 1U));
      }
      break;
    case 0xC1U: s_edt = d; break;                /* ENABLE DEVICE TYPE   */
    case 0xC3U: s_dtr1 = d; break;               /* DTR1                 */
    case 0xC5U: s_dtr2 = d; break;               /* DTR2                 */
    case 0xC7U: write_memory(d, true); break;    /* WRITE MEMORY LOCATION */
    case 0xC9U: write_memory(d, false); break;   /* ... - NO REPLY       */
    default: break;                              /* reserved             */
  }
}

/* Commands that keep writeEnableState (memory bank access sequence) */
static bool keeps_write_enable(bool special, uint8_t a, uint8_t d)
{
  bool keep;
  if (special)
  {
    keep = (a == 0xA3U) || (a == 0xC3U) || (a == 0xC5U) || (a == 0xC7U) || (a == 0xC9U) ||
           (a == 0xC1U);
  }
  else
  {
    keep = (d == 0x81U) || (d == 0x98U) || (d == 0x9CU) || (d == 0x9DU) || (d == 0xC5U);
  }
  return keep;
}

static void execute(uint8_t a, uint8_t d, uint8_t edt)
{
  if (is_special(a))
  {
    if ((a & 1U) != 0U)
    {
      if (!keeps_write_enable(true, a, d))
      {
        s_writeEnable = false;
      }
      cmd_special(a, d);
    }
  }
  else if ((a >= 0xCCU) && (a <= 0xFBU))
  {
    /* reserved address bytes */
  }
  else if (is_addressed(a))
  {
    if ((a & 1U) == 0U)
    {
      s_writeEnable = false;
      s_qndtValid = false;
      cmd_dapc(d);
    }
    else
    {
      if (!keeps_write_enable(false, a, d))
      {
        s_writeEnable = false;
      }
      if ((d != 0x99U) && (d != 0xA7U))
      {
        s_qndtValid = false;
      }
      if (d <= 0x1FU)
      {
        cmd_arc(d);
      }
      else if (d <= 0x8FU)
      {
        cmd_config(d);
      }
      else if (d <= 0xDFU)
      {
        cmd_query(d);
      }
      else if (edt == DT_LED)
      {
        cmd_dt6(d);
      }
      else
      {
        /* application extended command of another device type */
      }
    }
  }
  else
  {
    /* not addressed to this gear */
  }
}

static void handle_frame(const DALI_Frame_t *f, uint32_t nowMs)
{
  if (f->bits != 16U)
  {
    /* 24-bit (control device) frames, backward frames, errors: ignored,
     * but they break a send-twice sequence. */
    s_pendValid = false;
  }
  else
  {
    const uint16_t frame = (uint16_t)f->data;
    const uint8_t a = (uint8_t)(frame >> 8U);
    const uint8_t d = (uint8_t)frame;
    const bool isRepeat = s_pendValid && (s_pendFrame == frame) &&
                          ((nowMs - s_pendMs) <= SEND_TWICE_MS);
    const uint8_t edt = isRepeat ? s_pendEdt : s_edt;

    s_edt = DT_NONE;                             /* applies to one command only */
    s_rng ^= ((uint32_t)DALI_PHY_GetTimeUs() << 16U) ^ frame;

    if (!needs_twice(a, d, edt))
    {
      s_pendValid = false;
      execute(a, d, edt);
    }
    else if (isRepeat)
    {
      s_pendValid = false;
      execute(a, d, edt);
    }
    else
    {
      s_pendValid = true;
      s_pendFrame = frame;
      s_pendEdt = edt;
      s_pendMs = nowMs;
    }
  }
}

/* ---- public API -------------------------------------------------------------- */

void Gear_Init(void)
{
  uint8_t uid[12];

  BSP_GetUid(uid);
  s_rng ^= ((uint32_t)CRC16_Compute(uid, 12U) << 8U) | uid[0];

  if (!NVM_LoadGear(&s_nv))
  {
    /* factory defaults */
    s_nv.shortAddress = MASK;
    set_reset_values();
    s_nvSaveNow = true;
  }
  /* sanitise values read from the EEPROM */
  if ((s_nv.minLevel < CFG_PHYSICAL_MIN_LEVEL) || (s_nv.minLevel > LEVEL_MAX))
  {
    s_nv.minLevel = CFG_PHYSICAL_MIN_LEVEL;
  }
  if ((s_nv.maxLevel < s_nv.minLevel) || (s_nv.maxLevel > LEVEL_MAX))
  {
    s_nv.maxLevel = LEVEL_MAX;
  }
  if ((s_nv.shortAddress > 63U) && (s_nv.shortAddress != MASK))
  {
    s_nv.shortAddress = MASK;
  }
  Light_SetCurve(s_nv.dimmingCurve);
}

void Gear_PowerOn(void)
{
  const uint8_t level = (s_nv.powerOnLevel == MASK) ? s_nv.lastLightLevel : s_nv.powerOnLevel;
  const uint32_t pm = BSP_EnterCritical();

  Light_FadeTo(clamp_level(level), 0U, s_nv.minLevel);
  if (level != 0U)
  {
    s_lastActiveLevel = clamp_level(level);
  }
  s_powerCycleSeen = true;
  BSP_ExitCritical(pm);
}

void Gear_Tick1ms(void)
{
  const uint32_t now = BSP_GetTickMs();
  DALI_Frame_t frame;

  /* DALI system failure: bus low for more than 500 ms */
  if (DALI_PHY_IsBusIdle())
  {
    s_busLowMs = 0U;
    s_sysFailure = false;
  }
  else if (s_busLowMs < SYS_FAILURE_MS)
  {
    s_busLowMs++;
  }
  else if (!s_sysFailure)
  {
    s_sysFailure = true;
    if (s_nv.systemFailureLevel != MASK)
    {
      const uint8_t l = clamp_level(s_nv.systemFailureLevel);
      Light_SetIdentify(false, s_nv.minLevel);
      Light_FadeTo(l, 0U, s_nv.minLevel);
      if (l != 0U)
      {
        s_lastActiveLevel = l;
      }
    }
  }
  else
  {
    /* failure already handled */
  }

  if (DALI_PHY_GetFrame(&frame))
  {
    handle_frame(&frame, now);
  }

  /* timers */
  if (s_initTimerMs != 0U)
  {
    s_initTimerMs--;
    if (s_initTimerMs == 0U)
    {
      s_initState = INIT_DISABLED;
    }
  }
  if (s_dapcSeqMs != 0U)
  {
    s_dapcSeqMs--;
  }
  if (s_identifyMs != 0U)
  {
    s_identifyMs--;
    if (s_identifyMs == 0U)
    {
      Light_SetIdentify(false, s_nv.minLevel);
    }
  }

  Light_Tick1ms();

  /* last light level (used by POWER ON LEVEL = MASK) */
  if ((s_nv.powerOnLevel == MASK) && (!Light_IsFading()) &&
      (s_nv.lastLightLevel != Light_GetActualLevel()))
  {
    s_nv.lastLightLevel = Light_GetActualLevel();
    nv_dirty();
  }
}

void Gear_NvmService(void)
{
  if (s_nvSaveNow || (s_nvDirty && ((BSP_GetTickMs() - s_nvDirtyMs) >= CFG_NVM_SAVE_DELAY_MS)))
  {
    NVM_Gear_t copy;
    const uint32_t pm = BSP_EnterCritical();
    copy = s_nv;
    s_nvDirty = false;
    s_nvSaveNow = false;
    BSP_ExitCritical(pm);
    (void)NVM_SaveGear(&copy);
  }
}

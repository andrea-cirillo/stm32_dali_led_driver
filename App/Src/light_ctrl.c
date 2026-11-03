/**
 ******************************************************************************
 * @file    light_ctrl.c
 * @brief   Light output control.
 ******************************************************************************
 */
#include "light_ctrl.h"
#include "app_config.h"
#include "bsp.h"
#include "pwm_drv.h"

#define LEVEL_MAX              (254U)
#define Q16(level)             ((int32_t)(level) << 16)
#define IDENTIFY_HALF_PERIOD_MS (500U)

/* 2^(-j/16) in Q16, j = 0..16 */
static const uint16_t k_exp2Neg[17] =
{
  65535U, 62757U, 60097U, 57549U, 55109U, 52773U, 50535U, 48393U, 46341U,
  44376U, 42495U, 40693U, 38968U, 37316U, 35734U, 34219U, 32768U
};

static const uint16_t k_pwmCal[CFG_PWM_CAL_POINTS] = CFG_PWM_CAL_TABLE;

static int32_t  s_levelQ16 = 0;
static int32_t  s_targetQ16 = 0;
static int32_t  s_stepQ16 = 0;
static bool     s_offAtEnd = false;
static uint8_t  s_curve = LIGHT_CURVE_LOGARITHMIC;

static bool     s_identify = false;
static uint8_t  s_identifyMin = 1U;
static uint16_t s_identifyMs = 0U;

static volatile bool     s_faultLatched = false;
static volatile bool     s_lampOn = false;
static volatile uint32_t s_onTimeMs = 0U;
static volatile uint32_t s_startEvents = 0U;
static volatile uint16_t s_relCurrentQ16 = 0U;

/*
 * Light output (Q16 fraction of full output) for a level in Q16.
 * Logarithmic: X(n) = 10^((n - 1) * 3 / 253 - 3)  (0.1 % .. 100 %)
 *            = 2^(-(254 - n) * 0.0393905)
 * Linear:      X(n) = n / 254
 */
static uint16_t curve_output(int32_t levelQ16)
{
  const uint32_t l8 = (uint32_t)levelQ16 >> 8U;   /* level in Q8 */
  uint32_t out;

  if (l8 == 0U)
  {
    out = 0U;
  }
  else if (s_curve == LIGHT_CURVE_LINEAR)
  {
    out = (l8 * 65535UL) / (LEVEL_MAX * 256UL);
  }
  else
  {
    const uint32_t k8 = (LEVEL_MAX * 256UL) - l8;
    const uint32_t e = (k8 * 5163UL) >> 9U;         /* exponent, Q16 */
    const uint32_t ip = e >> 16U;
    const uint32_t fp = e & 0xFFFFUL;
    const uint32_t j = fp >> 12U;
    const uint32_t r = fp & 0x0FFFUL;
    const uint32_t a = k_exp2Neg[j];
    const uint32_t b = k_exp2Neg[j + 1U];
    out = (a - (((a - b) * r) >> 12U)) >> ip;
  }
  return (out > 65535UL) ? 65535U : (uint16_t)out;
}

/* PWM compare value for a relative current (Q16), linear interpolation */
static uint16_t calibrated_duty(uint16_t relQ16)
{
  const uint32_t pos = (uint32_t)relQ16 * (CFG_PWM_CAL_POINTS - 1U);
  const uint32_t idx = pos >> 16U;
  uint32_t duty;

  if ((relQ16 == 0xFFFFU) || (idx >= (CFG_PWM_CAL_POINTS - 1U)))
  {
    duty = k_pwmCal[CFG_PWM_CAL_POINTS - 1U];
  }
  else
  {
    const int32_t a = (int32_t)k_pwmCal[idx];
    const int32_t b = (int32_t)k_pwmCal[idx + 1U];
    const int32_t frac = (int32_t)(pos & 0xFFFFUL);
    duty = (uint32_t)(a + (((b - a) * frac) / 65536L));
  }
  return (uint16_t)duty;
}

static void output_update(void)
{
  int32_t lvl = s_levelQ16;

  if (s_identify)
  {
    lvl = (s_identifyMs < IDENTIFY_HALF_PERIOD_MS) ? Q16(LEVEL_MAX) : Q16(s_identifyMin);
  }

  if ((lvl <= 0) || s_faultLatched)
  {
    PWM_DrvSetDuty(CFG_PWM_MAX_DUTY);
    BSP_PowerStageEnable(false);
    s_lampOn = false;
    s_onTimeMs = 0U;
    s_relCurrentQ16 = 0U;
  }
  else
  {
    const uint16_t rel = curve_output(lvl);
    PWM_DrvSetDuty(calibrated_duty(rel));
    s_relCurrentQ16 = rel;
    if (!s_lampOn)
    {
      s_startEvents++;
    }
    BSP_PowerStageEnable(true);
    s_lampOn = true;
    if (s_onTimeMs < 0xFFFFFFFFUL)
    {
      s_onTimeMs++;
    }
  }
}

void Light_Init(void)
{
  PWM_DrvInit();
  s_levelQ16 = 0;
  s_targetQ16 = 0;
  output_update();
}

void Light_Tick1ms(void)
{
  if (s_levelQ16 != s_targetQ16)
  {
    if (s_levelQ16 < s_targetQ16)
    {
      s_levelQ16 += s_stepQ16;
      if (s_levelQ16 > s_targetQ16)
      {
        s_levelQ16 = s_targetQ16;
      }
    }
    else
    {
      s_levelQ16 -= s_stepQ16;
      if (s_levelQ16 < s_targetQ16)
      {
        s_levelQ16 = s_targetQ16;
      }
    }
  }
  if ((s_levelQ16 == s_targetQ16) && s_offAtEnd)
  {
    s_offAtEnd = false;
    s_levelQ16 = 0;
    s_targetQ16 = 0;
  }

  if (s_identify)
  {
    s_identifyMs++;
    if (s_identifyMs >= (2U * IDENTIFY_HALF_PERIOD_MS))
    {
      s_identifyMs = 0U;
    }
  }
  output_update();
}

void Light_FadeTo(uint8_t target, uint32_t fadeMs, uint8_t minLevel)
{
  int32_t tgt = Q16(target);
  const int32_t minQ = Q16((minLevel == 0U) ? 1U : minLevel);

  s_offAtEnd = false;
  if ((fadeMs == 0U) || ((target == 0U) && (s_levelQ16 == 0)))
  {
    s_levelQ16 = tgt;
    s_targetQ16 = tgt;
  }
  else
  {
    int32_t delta;
    if (s_levelQ16 == 0)
    {
      s_levelQ16 = minQ;              /* switch on at minLevel, then fade */
    }
    if (target == 0U)
    {
      tgt = minQ;                     /* fade down to minLevel, then off */
      s_offAtEnd = true;
    }
    delta = (tgt > s_levelQ16) ? (tgt - s_levelQ16) : (s_levelQ16 - tgt);
    s_targetQ16 = tgt;
    s_stepQ16 = (int32_t)((uint32_t)delta / fadeMs);
    if (s_stepQ16 == 0)
    {
      s_stepQ16 = 1;
    }
  }
}

void Light_FadeAtRate(uint8_t target, uint32_t stepQ16PerMs)
{
  s_offAtEnd = false;
  s_targetQ16 = Q16(target);
  s_stepQ16 = (stepQ16PerMs == 0U) ? 1 : (int32_t)stepQ16PerMs;
}

void Light_StopFade(void)
{
  /* stop on an integer level */
  s_levelQ16 = Q16(Light_GetActualLevel());
  s_targetQ16 = s_levelQ16;
  s_offAtEnd = false;
}

uint8_t Light_GetActualLevel(void)
{
  return (uint8_t)((uint32_t)(s_levelQ16 + 0x8000) >> 16U);
}

bool Light_IsFading(void)
{
  return (s_levelQ16 != s_targetQ16) || s_offAtEnd;
}

void Light_SetCurve(uint8_t curve)
{
  s_curve = (curve == LIGHT_CURVE_LINEAR) ? LIGHT_CURVE_LINEAR : LIGHT_CURVE_LOGARITHMIC;
}

void Light_SetIdentify(bool enable, uint8_t minLevel)
{
  s_identify = enable;
  s_identifyMin = (minLevel == 0U) ? 1U : minLevel;
  s_identifyMs = 0U;
}

bool Light_IsIdentifying(void)
{
  return s_identify;
}

void Light_SetFaultLatch(void)
{
  s_faultLatched = true;
  BSP_PowerStageEnable(false);
}

bool Light_IsFaultLatched(void)
{
  return s_faultLatched;
}

bool Light_IsLampOn(void)
{
  return s_lampOn;
}

uint32_t Light_GetOnTimeMs(void)
{
  return s_onTimeMs;
}

uint32_t Light_GetStartEvents(void)
{
  return s_startEvents;
}

uint16_t Light_GetRelativeCurrentQ16(void)
{
  return s_relCurrentQ16;
}

/**
 ******************************************************************************
 * @file    dali_membank.c
 * @brief   DALI memory banks 0, 1, 202, 205, 206.
 *
 *          Location 0x02 of banks 1/202/205/206 is the lock / latch byte
 *          (RAM, 0xFF after power-up):
 *            0x55 unlocks the lockable locations (bank 1, reset of bank 206)
 *            0xAA latches a consistent snapshot of the multi-byte values
 *                 (banks 202 / 205 / 206); any other value releases it.
 *          Not implemented values read as MASK (all bytes 0xFF).
 ******************************************************************************
 */
#include "dali_membank.h"
#include "app_config.h"
#include "bsp.h"
#include "nvm.h"
#include "opdata.h"
#include "led_monitor.h"
#include "light_ctrl.h"
#include "crc16.h"

#define VERSION_2_0            (0x08U)   /* (major << 2) | minor          */
#define LOCK_BYTE_LOC          (0x02U)
#define UNLOCK_VALUE           (0x55U)
#define LATCH_VALUE            (0xAAU)
#define MB1_FIRST_DATA_LOC     (0x03U)
#define MB1_SAVE_DELAY_MS      (1000U)

#define MB0_LAST_LOC           (0x1AU)
#define MB1_LAST_LOC           (0x77U)
#define MB202_LAST_LOC         (0x0FU)
#define MB205_LAST_LOC         (0x1CU)
#define MB206_LAST_LOC         (0x20U)
#define IMG_SIZE               (MB206_LAST_LOC + 1U)

/* index of the latchable banks */
#define LB_202                 (0U)
#define LB_205                 (1U)
#define LB_206                 (2U)
#define LB_COUNT               (3U)

static uint8_t s_mb1[NVM_MB1_SIZE];
static uint8_t s_mb1Lock = 0xFFU;
static volatile bool s_mb1Dirty = false;
static volatile uint32_t s_mb1DirtyMs = 0U;

static uint8_t s_lock[LB_COUNT] = { 0xFFU, 0xFFU, 0xFFU };
static uint8_t s_snapshot[LB_COUNT][IMG_SIZE];

static uint8_t s_id[8];

/* ---- helpers ------------------------------------------------------------- */

/**
 * @brief   Stores a value big-endian into a memory bank field, with saturation.
 * @details The value is limited to the field maximum minus 2 (0x..FD), because
 *          0x..FE (TMASK) and 0x..FF (MASK) are reserved by IEC 62386-25x.
 * @param[out] dst    First byte of the field (MSB).
 * @param[in]  value  Value to store.
 * @param[in]  n      Field size in bytes (1..4).
 * @par     Return value
 *          None (void).
 */
static void put_be(uint8_t *dst, uint32_t value, uint8_t n)
{
  const uint32_t maxVal = (n >= 4U) ? 0xFFFFFFFDUL : ((1UL << (8U * n)) - 3UL);
  uint32_t v = (value > maxVal) ? maxVal : value;
  uint8_t i;

  for (i = n; i > 0U; i--)
  {
    dst[i - 1U] = (uint8_t)v;
    v >>= 8U;
  }
}

/**
 * @brief   Maps a latchable bank number to its index in the lock / snapshot arrays.
 * @param[in] bank  Memory bank number.
 * @return  LB_202, LB_205, LB_206, or -1 for any other bank.
 * @note    Context: SysTick.
 */
static int32_t latch_index(uint8_t bank)
{
  int32_t idx = -1;
  if (bank == 202U)
  {
    idx = (int32_t)LB_202;
  }
  else if (bank == 205U)
  {
    idx = (int32_t)LB_205;
  }
  else if (bank == 206U)
  {
    idx = (int32_t)LB_206;
  }
  else
  {
    idx = -1;
  }
  return idx;
}

/**
 * @brief   Loads the IEC 62386-251 factory defaults into the bank 1 RAM mirror.
 * @details GTIN / luminaire ID = 0xFF (not specified), content format ID,
 *          manufacturing date, powers, voltages, light output, CRI, CCT and
 *          light distribution from app_config.h, colour and identification
 *          strings = 0x00.
 * @par     Parameters
 *          None.
 * @par     Return value
 *          None (void).
 * @note    Context: main (init) or SysTick (RESET MEMORY BANK).
 */
static void mb1_defaults(void)
{
  uint8_t *const p = &s_mb1[0];
  uint32_t i;

  for (i = 0U; i < NVM_MB1_SIZE; i++)
  {
    p[i] = 0xFFU;                       /* GTIN, luminaire ID, ...       */
  }
  /* offsets are location - 0x03 */
  put_be(&p[0x11U - 3U], CFG_MB1_CONTENT_FORMAT_ID, 2U);
  p[0x13U - 3U] = CFG_MB1_YEAR_OF_MANUFACTURE;
  p[0x14U - 3U] = CFG_MB1_WEEK_OF_MANUFACTURE;
  p[0x15U - 3U] = (uint8_t)(CFG_MB1_INPUT_POWER_NOMINAL_W >> 8U);
  p[0x16U - 3U] = (uint8_t)CFG_MB1_INPUT_POWER_NOMINAL_W;
  p[0x17U - 3U] = (uint8_t)(CFG_MB1_INPUT_POWER_MIN_DIM_MW >> 8U);
  p[0x18U - 3U] = (uint8_t)CFG_MB1_INPUT_POWER_MIN_DIM_MW;
  p[0x19U - 3U] = (uint8_t)(CFG_MB1_MAINS_V_MIN >> 8U);
  p[0x1AU - 3U] = (uint8_t)CFG_MB1_MAINS_V_MIN;
  p[0x1BU - 3U] = (uint8_t)(CFG_MB1_MAINS_V_MAX >> 8U);
  p[0x1CU - 3U] = (uint8_t)CFG_MB1_MAINS_V_MAX;
  p[0x1DU - 3U] = (uint8_t)(CFG_MB1_LIGHT_OUTPUT_LM >> 16U);
  p[0x1EU - 3U] = (uint8_t)(CFG_MB1_LIGHT_OUTPUT_LM >> 8U);
  p[0x1FU - 3U] = (uint8_t)CFG_MB1_LIGHT_OUTPUT_LM;
  p[0x20U - 3U] = CFG_MB1_CRI;
  p[0x21U - 3U] = (uint8_t)(CFG_MB1_CCT_K >> 8U);
  p[0x22U - 3U] = (uint8_t)CFG_MB1_CCT_K;
  p[0x23U - 3U] = CFG_MB1_LIGHT_DISTRIBUTION;
  for (i = (0x24U - 3U); i < NVM_MB1_SIZE; i++)
  {
    p[i] = 0x00U;                       /* colour + identification text  */
  }
}

/**
 * @brief   Builds the live image of a latchable bank (202, 205 or 206).
 * @details Every location defaults to 0xFF (MASK = not implemented), location
 *          0x03 is the bank version 1.0, then the bank specific values are
 *          filled from the counters and monitor estimates:
 *          202: active energy (Wh) and power (0.1 W);
 *          205: gear operating time, starts, failure, temperature, output %;
 *          206: light source starts, on times, voltage, current, failures.
 * @param[in]  bank  202, 205 or 206 (any other value builds bank 206).
 * @param[out] img   Destination, IMG_SIZE bytes indexed by location.
 * @par     Return value
 *          None (void).
 * @note    Context: SysTick.
 */
static void build_image(uint8_t bank, uint8_t img[IMG_SIZE])
{
  const NVM_Counters_t *const c = OpData_Get();
  const uint8_t fail = Monitor_GetFailureStatus();
  uint32_t i;

  for (i = 0U; i < IMG_SIZE; i++)
  {
    img[i] = 0xFFU;
  }
  img[0x03] = 0x01U;                          /* bank version 1.0          */

  if (bank == 202U)
  {
    /* 0x04 scale 10^0 Wh, 0x05..0x0A energy, 0x0B scale 10^-1 W, power */
    img[0x04] = 0x00U;
    img[0x05] = 0x00U;
    img[0x06] = 0x00U;
    put_be(&img[0x07], c->activeEnergyWh, 4U);
    img[0x0B] = 0xFFU;                        /* -1 -> 0.1 W               */
    put_be(&img[0x0C], Monitor_GetInputPowerMw() / 100UL, 4U);
  }
  else if (bank == 205U)
  {
    const int16_t t = Monitor_GetTemperatureC();
    const uint32_t nomRel = Light_GetRelativeCurrentQ16();

    put_be(&img[0x04], c->gearOperatingTimeS, 4U);
    put_be(&img[0x08], c->gearStartCounter, 3U);
    /* 0x0B..0x0E supply voltage / frequency / power factor: not measured */
    img[0x0F] = 0x00U;                        /* no gear internal failure  */
    img[0x10] = c->gearFailureCounter;
    /* 0x11..0x1A supply / power limitation / thermal: not implemented     */
    img[0x1B] = (t < -60) ? 0x00U : ((t > 193) ? 0xFDU : (uint8_t)(t + 60));
    img[0x1C] = (uint8_t)((nomRel * 100UL + 32768UL) >> 16U);
  }
  else
  {
    /* bank 206 */
    const uint32_t vDeci = Monitor_GetLedVoltageMv() / 100UL;
    put_be(&img[0x04], c->lsStartCounterRes, 3U);
    put_be(&img[0x07], c->lsStartCounter, 3U);
    put_be(&img[0x0A], c->lsOnTimeResS, 4U);
    put_be(&img[0x0E], c->lsOnTimeS, 4U);
    put_be(&img[0x12], vDeci, 2U);
    put_be(&img[0x14], Monitor_GetLedCurrentMa(), 2U);
    img[0x16] = (fail != 0U) ? 1U : 0U;
    img[0x17] = c->lsFailureCounter;
    img[0x18] = ((fail & MON_FAIL_SHORT_CIRCUIT) != 0U) ? 1U : 0U;
    img[0x19] = c->lsShortCounter;
    img[0x1A] = ((fail & MON_FAIL_OPEN_CIRCUIT) != 0U) ? 1U : 0U;
    img[0x1B] = c->lsOpenCounter;
    /* 0x1C..0x20 thermal derating / shutdown / temperature: not implemented */
  }
}

/**
 * @brief   Reads one location of memory bank 0 (IEC 62386-102 identification).
 * @details GTIN and versions from app_config.h, identification number derived
 *          from the UID, part 101 / 102 versions 2.0, part 103 not implemented,
 *          0 logical control devices, 1 logical control gear unit.
 * @param[in]  loc    Location 0x00..0x1A.
 * @param[out] value  Location content; written only on success.
 * @return  true if implemented, false for reserved locations (no answer).
 * @note    Context: SysTick.
 */
static bool read_mb0(uint8_t loc, uint8_t *value)
{
  bool ok = true;

  if ((loc >= 0x03U) && (loc <= 0x08U))
  {
    static const uint8_t gtin[6] =
    {
      CFG_MB0_GTIN_B0, CFG_MB0_GTIN_B1, CFG_MB0_GTIN_B2,
      CFG_MB0_GTIN_B3, CFG_MB0_GTIN_B4, CFG_MB0_GTIN_B5
    };
    *value = gtin[loc - 0x03U];
  }
  else if ((loc >= 0x0BU) && (loc <= 0x12U))
  {
    *value = s_id[loc - 0x0BU];
  }
  else
  {
    switch (loc)
    {
      case 0x00U: *value = MB0_LAST_LOC; break;
      case 0x02U: *value = MB_LAST_BANK; break;
      case 0x09U: *value = CFG_FW_VERSION_MAJOR; break;
      case 0x0AU: *value = CFG_FW_VERSION_MINOR; break;
      case 0x13U: *value = CFG_HW_VERSION_MAJOR; break;
      case 0x14U: *value = CFG_HW_VERSION_MINOR; break;
      case 0x15U: *value = VERSION_2_0; break;   /* IEC 62386-101          */
      case 0x16U: *value = VERSION_2_0; break;   /* IEC 62386-102          */
      case 0x17U: *value = 0xFFU; break;         /* no control device part */
      case 0x18U: *value = 0x00U; break;         /* logical device units   */
      case 0x19U: *value = 0x01U; break;         /* logical gear units     */
      case 0x1AU: *value = 0x00U; break;         /* index of this unit     */
      default:    ok = false; break;             /* 0x01 reserved          */
    }
  }
  return ok;
}

/* ---- public API ---------------------------------------------------------- */

/* Public API (full description in dali_membank.h): identification number and bank 1 load. */
void MB_Init(void)
{
  uint8_t uid[12];
  uint16_t crcA;
  uint16_t crcB;

#if (CFG_MB0_ID_FROM_UID != 0)
  /* 64-bit identification number derived from the 96-bit device UID */
  BSP_GetUid(uid);
  crcA = CRC16_Compute(uid, 6U);
  crcB = CRC16_Compute(&uid[6], 6U);
  s_id[0] = (uint8_t)(crcA >> 8U);
  s_id[1] = (uint8_t)crcA;
  s_id[2] = (uint8_t)(crcB >> 8U);
  s_id[3] = (uint8_t)crcB;
  s_id[4] = uid[0] ^ uid[4];   /* wafer X / lot              */
  s_id[5] = uid[1] ^ uid[5];
  s_id[6] = uid[2] ^ uid[8];   /* wafer Y / lot              */
  s_id[7] = uid[3] ^ uid[11];
#else
  (void)uid;
  (void)crcA;
  (void)crcB;
  put_be(&s_id[0], (uint32_t)(CFG_MB0_ID_NUMBER >> 32U), 4U);
  put_be(&s_id[4], (uint32_t)CFG_MB0_ID_NUMBER, 4U);
#endif

  if (!NVM_LoadMb1(s_mb1))
  {
    mb1_defaults();
    (void)NVM_SaveMb1(s_mb1);
  }
}

/* Public API (full description in dali_membank.h): deferred bank 1 persistence. */
void MB_Service(void)
{
  if (s_mb1Dirty && ((BSP_GetTickMs() - s_mb1DirtyMs) >= MB1_SAVE_DELAY_MS))
  {
    uint8_t copy[NVM_MB1_SIZE];
    uint32_t i;
    const uint32_t pm = BSP_EnterCritical();

    for (i = 0U; i < NVM_MB1_SIZE; i++)
    {
      copy[i] = s_mb1[i];
    }
    s_mb1Dirty = false;
    BSP_ExitCritical(pm);
    (void)NVM_SaveMb1(copy);
  }
}

/* Public API (full description in dali_membank.h): bank existence. */
bool MB_IsImplemented(uint8_t bank)
{
  return (bank == 0U) || (bank == 1U) || (latch_index(bank) >= 0);
}

/* Public API (full description in dali_membank.h): last accessible location. */
uint8_t MB_LastLocation(uint8_t bank)
{
  uint8_t last;

  switch (bank)
  {
    case 0U:   last = MB0_LAST_LOC; break;
    case 1U:   last = MB1_LAST_LOC; break;
    case 202U: last = MB202_LAST_LOC; break;
    case 205U: last = MB205_LAST_LOC; break;
    default:   last = MB206_LAST_LOC; break;
  }
  return last;
}

/* Public API (full description in dali_membank.h): READ MEMORY LOCATION. */
bool MB_Read(uint8_t bank, uint8_t loc, uint8_t *value)
{
  const int32_t li = latch_index(bank);
  bool ok = MB_IsImplemented(bank) && (loc <= MB_LastLocation(bank));

  if (!ok)
  {
    /* not implemented */
  }
  else if (bank == 0U)
  {
    ok = read_mb0(loc, value);
  }
  else if (loc == 0x00U)
  {
    *value = MB_LastLocation(bank);
  }
  else if (loc == 0x01U)
  {
    ok = false;                               /* reserved                  */
  }
  else if (bank == 1U)
  {
    *value = (loc == LOCK_BYTE_LOC) ? s_mb1Lock : s_mb1[loc - MB1_FIRST_DATA_LOC];
  }
  else if (loc == LOCK_BYTE_LOC)
  {
    *value = s_lock[li];
  }
  else if (s_lock[li] == LATCH_VALUE)
  {
    *value = s_snapshot[li][loc];
  }
  else
  {
    uint8_t img[IMG_SIZE];
    build_image(bank, img);
    *value = img[loc];
  }
  return ok;
}

/* Public API (full description in dali_membank.h): WRITE MEMORY LOCATION. */
bool MB_Write(uint8_t bank, uint8_t loc, uint8_t value)
{
  const int32_t li = latch_index(bank);
  bool ok = false;

  if ((!MB_IsImplemented(bank)) || (bank == 0U) || (loc > MB_LastLocation(bank)))
  {
    ok = false;
  }
  else if (bank == 1U)
  {
    if (loc == LOCK_BYTE_LOC)
    {
      s_mb1Lock = value;
      ok = true;
    }
    else if ((loc >= MB1_FIRST_DATA_LOC) && (s_mb1Lock == UNLOCK_VALUE))
    {
      s_mb1[loc - MB1_FIRST_DATA_LOC] = value;
      s_mb1Dirty = true;
      s_mb1DirtyMs = BSP_GetTickMs();
      ok = true;
    }
    else
    {
      ok = false;
    }
  }
  else if (loc == LOCK_BYTE_LOC)
  {
    s_lock[li] = value;
    if (value == LATCH_VALUE)
    {
      build_image(bank, s_snapshot[li]);
    }
    ok = true;
  }
  else if ((bank == 206U) && (loc >= 0x04U) && (loc <= 0x06U))
  {
    OpData_WriteResettableByte(OPDATA_CNT_LS_STARTS_RES, (uint8_t)(loc - 0x04U), 3U, value);
    ok = true;
  }
  else if ((bank == 206U) && (loc >= 0x0AU) && (loc <= 0x0DU))
  {
    OpData_WriteResettableByte(OPDATA_CNT_LS_ONTIME_RES, (uint8_t)(loc - 0x0AU), 4U, value);
    ok = true;
  }
  else
  {
    ok = false;                               /* read-only location        */
  }
  return ok;
}

/* Public API (full description in dali_membank.h): RESET MEMORY BANK. */
void MB_Reset(uint8_t bank)
{
  if (((bank == 0U) || (bank == 1U)) && (s_mb1Lock == UNLOCK_VALUE))
  {
    mb1_defaults();
    s_mb1Dirty = true;
    s_mb1DirtyMs = BSP_GetTickMs();
  }
  if (((bank == 0U) || (bank == 206U)) && (s_lock[LB_206] == UNLOCK_VALUE))
  {
    OpData_ResetResettable();
  }
}

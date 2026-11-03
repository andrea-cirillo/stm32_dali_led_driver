/**
 ******************************************************************************
 * @file    variant.c
 * @brief   Lamp variant management.
 ******************************************************************************
 */
#include "variant.h"
#include "app_config.h"
#include "bsp.h"
#include "crc16.h"
#include "nvm.h"
#include "ntag5.h"
#include "shiftreg.h"

#define REC_ID_0               (0x44U)  /* 'D' */
#define REC_ID_1               (0x4CU)  /* 'L' */
#define REC_ID_2               (0x56U)  /* 'V' */
#define REC_ID_3               (0x52U)  /* 'R' */
#define REC_FORMAT_VERSION     (0x01U)
#define TAG_READ_RETRIES       (3U)

/*
 * Shift register value of each variant (bit n -> Qn, '1' = resistor to GND).
 * Q0 = R92 301k, Q1 = R98 78k7, Q2 = R93 39k2, Q3 = R99 19k6, Q4 = R94 9k76,
 * Q5 = R100 4k87. Source: Docs/Schematics/Table_RvsQx.pdf.
 */
static const uint8_t k_variantSr[CFG_VARIANT_COUNT] =
{
  0x01U,  /*  0: Req 300.12 k  Q0                 */
  0x03U,  /*  1: Req  61.544 k Q0 Q1              */
  0x05U,  /*  2: Req  34.288 k Q0 Q2              */
  0x07U,  /*  3: Req  23.763 k Q0 Q1 Q2           */
  0x09U,  /*  4: Req  18.182 k Q0 Q3              */
  0x0BU,  /*  5: Req  14.724 k Q0 Q1 Q3           */
  0x0DU,  /*  6: Req  12.371 k Q0 Q2 Q3           */
  0x0FU,  /*  7: Req  10.667 k Q0 Q1 Q2 Q3        */
  0x11U,  /*  8: Req   9.375 k Q0 Q4              */
  0x13U,  /*  9: Req   8.363 k Q0 Q1 Q4           */
  0x15U,  /* 10: Req   7.547 k Q0 Q2 Q4           */
  0x17U,  /* 11: Req   6.877 k Q0 Q1 Q2 Q4        */
  0x19U,  /* 12: Req   6.316 k Q0 Q3 Q4           */
  0x1BU,  /* 13: Req   5.840 k Q0 Q1 Q3 Q4        */
  0x1DU,  /* 14: Req   5.430 k Q0 Q2 Q3 Q4        */
  0x1FU,  /* 15: Req   5.074 k Q0 Q1 Q2 Q3 Q4     */
  0x21U   /* 16: Req   4.762 k Q0 Q5              */
};

static uint8_t s_active = CFG_VARIANT_DEFAULT;
static uint8_t s_stored = CFG_VARIANT_DEFAULT;
static bool s_storedValid = false;

static volatile bool s_nfcEvent = false;
static volatile uint32_t s_nfcEventMs = 0U;
static uint32_t s_lastPollMs = 0U;

bool Variant_DecodeRecord(const uint8_t rec[VARIANT_RECORD_SIZE], uint8_t *variant)
{
  const uint16_t crc = CRC16_Compute(rec, 6U);
  const uint16_t stored = (uint16_t)(((uint16_t)rec[6] << 8U) | rec[7]);
  const bool ok = (rec[0] == REC_ID_0) && (rec[1] == REC_ID_1) &&
                  (rec[2] == REC_ID_2) && (rec[3] == REC_ID_3) &&
                  (rec[4] == REC_FORMAT_VERSION) &&
                  (rec[5] < CFG_VARIANT_COUNT) && (crc == stored);
  if (ok)
  {
    *variant = rec[5];
  }
  return ok;
}

void Variant_EncodeRecord(uint8_t variant, uint8_t rec[VARIANT_RECORD_SIZE])
{
  uint16_t crc;

  rec[0] = REC_ID_0;
  rec[1] = REC_ID_1;
  rec[2] = REC_ID_2;
  rec[3] = REC_ID_3;
  rec[4] = REC_FORMAT_VERSION;
  rec[5] = variant;
  crc = CRC16_Compute(rec, 6U);
  rec[6] = (uint8_t)(crc >> 8U);
  rec[7] = (uint8_t)crc;
}

static bool read_tag(uint8_t *variant)
{
  uint8_t rec[VARIANT_RECORD_SIZE];
  bool ok = false;
  uint8_t retry;

  for (retry = 0U; (retry < TAG_READ_RETRIES) && (!ok); retry++)
  {
    ok = NTAG5_ReadBlocks(CFG_NFC_VARIANT_BLOCK, rec, VARIANT_RECORD_SIZE) &&
         Variant_DecodeRecord(rec, variant);
  }
  return ok;
}

/* Stores a variant read from the tag if it differs from the EEPROM one. */
static void sync_from_tag(void)
{
  uint8_t tagVariant = 0U;

  if (read_tag(&tagVariant))
  {
    if ((!s_storedValid) || (tagVariant != s_stored))
    {
      uint8_t rec[VARIANT_RECORD_SIZE];
      Variant_EncodeRecord(tagVariant, rec);
      if (NVM_SaveVariantRecord(rec))
      {
        s_stored = tagVariant;
        s_storedValid = true;
      }
    }
  }
}

void Variant_ApplyAtBoot(void)
{
  uint8_t rec[VARIANT_RECORD_SIZE];
  uint8_t copy;

  /* lowest current selection until the stored variant is known */
  ShiftReg_Write(k_variantSr[CFG_VARIANT_DEFAULT]);

  for (copy = 0U; (copy < 2U) && (!s_storedValid); copy++)
  {
    if (NVM_LoadVariantRecord(rec, copy))
    {
      s_storedValid = Variant_DecodeRecord(rec, &s_stored);
    }
  }

  sync_from_tag();

  s_active = s_storedValid ? s_stored : CFG_VARIANT_DEFAULT;
  ShiftReg_Write(k_variantSr[s_active]);

  /* NTAG 5 ED pin: active on every RF write (session register, volatile) */
  (void)NTAG5_WriteReg(NTAG5_REG_ED_CONFIG, 0U, NTAG5_ED_MASK, NTAG5_ED_WRITE_CMD);

  /* EXTI0 <- PA0, falling edge (ED is active low) */
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI0) | SYSCFG_EXTICR1_EXTI0_PA;
  EXTI->FTSR |= EXTI_FTSR_TR0;
  EXTI->PR = EXTI_PR_PR0;
  EXTI->IMR |= EXTI_IMR_MR0;
  s_lastPollMs = BSP_GetTickMs();
}

uint8_t Variant_GetActive(void)
{
  return s_active;
}

uint8_t Variant_GetStored(void)
{
  return s_stored;
}

void Variant_NfcEventIrq(void)
{
  s_nfcEventMs = BSP_GetTickMs();
  s_nfcEvent = true;
}

void Variant_Service(void)
{
  const uint32_t now = BSP_GetTickMs();
  bool check = false;

  if (s_nfcEvent && ((now - s_nfcEventMs) >= CFG_NFC_EVENT_SETTLE_MS))
  {
    s_nfcEvent = false;
    check = true;
  }
  if ((now - s_lastPollMs) >= CFG_NFC_POLL_PERIOD_MS)
  {
    check = true;
  }
  if (check)
  {
    s_lastPollMs = now;
    sync_from_tag();
  }
}

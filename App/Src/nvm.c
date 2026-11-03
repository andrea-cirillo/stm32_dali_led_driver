/**
 ******************************************************************************
 * @file    nvm.c
 * @brief   Non-volatile storage (M24C04) with redundant / rotating records.
 ******************************************************************************
 */
#include "nvm.h"
#include "eeprom_m24c04.h"
#include "crc16.h"

#define NVM_GEAR_BASE          (0x000U)
#define NVM_GEAR_SLOT_SIZE     (0x40U)
#define NVM_GEAR_SLOTS         (2U)

#define NVM_MB1_BASE           (0x080U)
#define NVM_MB1_MARK_ADDR      (0x0F6U)
#define NVM_MB1_MARK_0         (0x4DU)  /* 'M' */
#define NVM_MB1_MARK_1         (0x31U)  /* '1' */

#define NVM_VARIANT_BASE       (0x100U)
#define NVM_VARIANT_SLOT_SIZE  (0x10U)
#define NVM_VARIANT_LEN        (8U)

#define NVM_CNT_BASE           (0x120U)
#define NVM_CNT_SLOT_SIZE      (0x30U)
#define NVM_CNT_SLOTS          (4U)

#define NVM_REC_OVERHEAD       (4U)     /* seq (2) + CRC (2)        */
#define NVM_REC_MAX_DATA       (40U)

/* Compile-time checks: records must fit the buffers and their slots. */
typedef char nvm_check_gear_size[((sizeof(NVM_Gear_t) <= NVM_REC_MAX_DATA) &&
                                  ((sizeof(NVM_Gear_t) + NVM_REC_OVERHEAD) <= NVM_GEAR_SLOT_SIZE)) ? 1 : -1];
typedef char nvm_check_cnt_size[((sizeof(NVM_Counters_t) <= NVM_REC_MAX_DATA) &&
                                 ((sizeof(NVM_Counters_t) + NVM_REC_OVERHEAD) <= NVM_CNT_SLOT_SIZE)) ? 1 : -1];

typedef struct
{
  uint16_t base;
  uint16_t slotSize;
  uint8_t  slots;
  uint8_t  current;   /* slot of the newest valid record             */
  uint16_t seq;       /* sequence number of the newest valid record  */
} NvmArea_t;

static NvmArea_t s_gearArea = { NVM_GEAR_BASE, NVM_GEAR_SLOT_SIZE, NVM_GEAR_SLOTS, 0U, 0U };
static NvmArea_t s_cntArea  = { NVM_CNT_BASE, NVM_CNT_SLOT_SIZE, NVM_CNT_SLOTS, 0U, 0U };

/* MISRA 11.5/11.3: conversion to a pointer to unsigned char is permitted. */
static bool load_newest(NvmArea_t *area, uint8_t *data, uint8_t len)
{
  uint8_t buf[NVM_REC_MAX_DATA + NVM_REC_OVERHEAD];
  bool found = false;
  uint8_t slot;
  uint8_t i;

  for (slot = 0U; slot < area->slots; slot++)
  {
    const uint16_t addr = (uint16_t)(area->base + ((uint16_t)slot * area->slotSize));
    if (M24C04_Read(addr, buf, (uint16_t)len + NVM_REC_OVERHEAD))
    {
      const uint16_t crc = CRC16_Compute(buf, (uint32_t)len + 2U);
      const uint16_t stored = (uint16_t)(((uint16_t)buf[len + 2U] << 8U) | buf[len + 3U]);
      const uint16_t seq = (uint16_t)(((uint16_t)buf[1] << 8U) | buf[0]);
      /* newer = sequence difference interpreted as signed (wrap safe) */
      const bool newer = (!found) || ((int16_t)(uint16_t)(seq - area->seq) > 0);
      if ((crc == stored) && newer)
      {
        found = true;
        area->current = slot;
        area->seq = seq;
        for (i = 0U; i < len; i++)
        {
          data[i] = buf[i + 2U];
        }
      }
    }
  }
  if (!found)
  {
    area->current = (uint8_t)(area->slots - 1U);
    area->seq = 0U;
  }
  return found;
}

static bool save_next(NvmArea_t *area, const uint8_t *data, uint8_t len)
{
  uint8_t buf[NVM_REC_MAX_DATA + NVM_REC_OVERHEAD];
  const uint8_t slot = (uint8_t)((area->current + 1U) % area->slots);
  const uint16_t seq = (uint16_t)(area->seq + 1U);
  const uint16_t addr = (uint16_t)(area->base + ((uint16_t)slot * area->slotSize));
  uint16_t crc;
  uint8_t i;
  bool ok;

  buf[0] = (uint8_t)seq;
  buf[1] = (uint8_t)(seq >> 8U);
  for (i = 0U; i < len; i++)
  {
    buf[i + 2U] = data[i];
  }
  crc = CRC16_Compute(buf, (uint32_t)len + 2U);
  buf[len + 2U] = (uint8_t)(crc >> 8U);
  buf[len + 3U] = (uint8_t)crc;

  /* the oldest slot is overwritten: data first, CRC last (power fail safe) */
  ok = M24C04_Write(addr, buf, (uint16_t)len + 2U) &&
       M24C04_Write((uint16_t)(addr + len + 2U), &buf[len + 2U], 2U);
  if (ok)
  {
    area->current = slot;
    area->seq = seq;
  }
  return ok;
}

bool NVM_LoadGear(NVM_Gear_t *gear)
{
  return load_newest(&s_gearArea, (uint8_t *)gear, (uint8_t)sizeof(NVM_Gear_t));
}

bool NVM_SaveGear(const NVM_Gear_t *gear)
{
  return save_next(&s_gearArea, (const uint8_t *)gear, (uint8_t)sizeof(NVM_Gear_t));
}

bool NVM_LoadCounters(NVM_Counters_t *cnt)
{
  return load_newest(&s_cntArea, (uint8_t *)cnt, (uint8_t)sizeof(NVM_Counters_t));
}

bool NVM_SaveCounters(const NVM_Counters_t *cnt)
{
  return save_next(&s_cntArea, (const uint8_t *)cnt, (uint8_t)sizeof(NVM_Counters_t));
}

bool NVM_LoadMb1(uint8_t data[NVM_MB1_SIZE])
{
  uint8_t mark[2] = { 0U, 0U };
  bool ok = M24C04_Read(NVM_MB1_MARK_ADDR, mark, 2U);

  ok = ok && (mark[0] == NVM_MB1_MARK_0) && (mark[1] == NVM_MB1_MARK_1);
  ok = ok && M24C04_Read(NVM_MB1_BASE, data, NVM_MB1_SIZE);
  return ok;
}

bool NVM_SaveMb1(const uint8_t data[NVM_MB1_SIZE])
{
  static const uint8_t mark[2] = { NVM_MB1_MARK_0, NVM_MB1_MARK_1 };
  return M24C04_Write(NVM_MB1_BASE, data, NVM_MB1_SIZE) &&
         M24C04_Write(NVM_MB1_MARK_ADDR, mark, 2U);
}

bool NVM_LoadVariantRecord(uint8_t rec[8], uint8_t copy)
{
  const uint16_t addr = (uint16_t)(NVM_VARIANT_BASE + ((uint16_t)(copy & 1U) * NVM_VARIANT_SLOT_SIZE));
  return M24C04_Read(addr, rec, NVM_VARIANT_LEN);
}

bool NVM_SaveVariantRecord(const uint8_t rec[8])
{
  return M24C04_Write(NVM_VARIANT_BASE, rec, NVM_VARIANT_LEN) &&
         M24C04_Write(NVM_VARIANT_BASE + NVM_VARIANT_SLOT_SIZE, rec, NVM_VARIANT_LEN);
}

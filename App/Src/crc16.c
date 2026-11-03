/**
 ******************************************************************************
 * @file    crc16.c
 * @brief   CRC-16/CCITT-FALSE, bitwise implementation (small code size).
 ******************************************************************************
 */
#include "crc16.h"

uint16_t CRC16_Compute(const uint8_t *data, uint32_t len)
{
  uint16_t crc = 0xFFFFU;
  uint32_t i;
  uint32_t b;

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)((uint16_t)data[i] << 8U);
    for (b = 0U; b < 8U; b++)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1U);
      }
    }
  }
  return crc;
}

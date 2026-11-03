/**
 ******************************************************************************
 * @file    eeprom_m24c04.c
 * @brief   ST M24C04 EEPROM driver.
 ******************************************************************************
 */
#include "eeprom_m24c04.h"
#include <stddef.h>
#include "i2c_bus.h"
#include "bsp.h"

#define M24C04_DEV_ADDR        (0x50U)  /* 7-bit, A8 is added as bit 0 */
#define M24C04_WRITE_TIME_MS   (10U)    /* tW max 5 ms, margin x2        */
#define M24C04_READ_CHUNK      (128U)
#define M24C04_RETRIES         (3U)

static uint8_t dev_addr(uint16_t addr)
{
  return (uint8_t)(M24C04_DEV_ADDR | (uint8_t)((addr >> 8U) & 1U));
}

/* Waits for the end of the internal write cycle (device answers again). */
static bool ack_polling(uint16_t addr)
{
  const uint32_t start = BSP_GetTickMs();
  const uint8_t word = (uint8_t)addr;
  bool ok = false;

  while ((!ok) && ((BSP_GetTickMs() - start) <= M24C04_WRITE_TIME_MS))
  {
    uint8_t dummy;
    ok = (I2C_BusTransfer(dev_addr(addr), &word, 1U, &dummy, 1U) == I2C_BUS_OK);
  }
  return ok;
}

bool M24C04_Read(uint16_t addr, uint8_t *data, uint16_t len)
{
  bool ok = (data != NULL) && (len != 0U) && (((uint32_t)addr + len) <= M24C04_SIZE);
  uint16_t a = addr;
  uint16_t remaining = len;
  uint8_t *p = data;

  while (ok && (remaining != 0U))
  {
    /* do not cross the 256-byte block boundary inside one transfer */
    const uint16_t toBlockEnd = (uint16_t)(256U - (a & 0xFFU));
    uint16_t n = (remaining < toBlockEnd) ? remaining : toBlockEnd;
    const uint8_t word = (uint8_t)a;
    uint8_t retry = 0U;
    I2C_BusStatus_t st;

    if (n > M24C04_READ_CHUNK)
    {
      n = M24C04_READ_CHUNK;
    }
    do
    {
      st = I2C_BusTransfer(dev_addr(a), &word, 1U, p, (uint8_t)n);
      retry++;
    } while ((st != I2C_BUS_OK) && (retry < M24C04_RETRIES));

    ok = (st == I2C_BUS_OK);
    a = (uint16_t)(a + n);
    p = &p[n];
    remaining = (uint16_t)(remaining - n);
  }
  return ok;
}

bool M24C04_Write(uint16_t addr, const uint8_t *data, uint16_t len)
{
  bool ok = (data != NULL) && (len != 0U) && (((uint32_t)addr + len) <= M24C04_SIZE);
  uint16_t a = addr;
  uint16_t remaining = len;
  const uint8_t *p = data;

  while (ok && (remaining != 0U))
  {
    const uint16_t toPageEnd = (uint16_t)(M24C04_PAGE_SIZE - (a & (M24C04_PAGE_SIZE - 1U)));
    const uint16_t n = (remaining < toPageEnd) ? remaining : toPageEnd;
    uint8_t buf[M24C04_PAGE_SIZE + 1U];
    uint16_t i;
    uint8_t retry = 0U;
    I2C_BusStatus_t st;

    buf[0] = (uint8_t)a;
    for (i = 0U; i < n; i++)
    {
      buf[i + 1U] = p[i];
    }
    do
    {
      st = I2C_BusTransfer(dev_addr(a), buf, (uint8_t)(n + 1U), NULL, 0U);
      retry++;
    } while ((st != I2C_BUS_OK) && (retry < M24C04_RETRIES));

    ok = (st == I2C_BUS_OK) && ack_polling(a);
    a = (uint16_t)(a + n);
    p = &p[n];
    remaining = (uint16_t)(remaining - n);
  }
  return ok;
}

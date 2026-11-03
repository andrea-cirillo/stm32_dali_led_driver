/**
 ******************************************************************************
 * @file    i2c_bus.h
 * @brief   Blocking I2C1 master driver (register level, polling, timeouts).
 *          Shared by the M24C04 EEPROM and the NTAG 5 link. Only to be used
 *          from the main loop context.
 ******************************************************************************
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  I2C_BUS_OK = 0,
  I2C_BUS_NACK,      /* address or data not acknowledged (busy device) */
  I2C_BUS_TIMEOUT,   /* bus stuck or peripheral error                  */
  I2C_BUS_PARAM
} I2C_BusStatus_t;

void I2C_BusInit(void);

/**
 * @brief  Generic transfer: writes wlen bytes then, if rlen > 0, issues a
 *         repeated start and reads rlen bytes. addr7 is the 7-bit address.
 */
I2C_BusStatus_t I2C_BusTransfer(uint8_t addr7,
                                const uint8_t *wdata, uint8_t wlen,
                                uint8_t *rdata, uint8_t rlen);

#endif /* I2C_BUS_H */

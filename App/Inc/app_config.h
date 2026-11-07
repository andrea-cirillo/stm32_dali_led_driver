/**
 ******************************************************************************
 * @file    app_config.h
 * @brief   Central build-time configuration of the DALI-2 LED driver firmware.
 *
 *          Every value marked "PLACEHOLDER" must be reviewed and replaced with
 *          the final product data / characterisation results before release.
 ******************************************************************************
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ========================================================================== */
/* DALI physical layer polarity                                               */
/* ========================================================================== */
/*
 * DALI_CFG_TX_ACTIVE_HIGH
 *   1 (default, standard interface): MCU_TXD = HIGH drives the DALI bus LOW
 *     (active state), MCU_TXD = LOW releases the bus (idle HIGH).
 *   0: inverted interface, MCU_TXD = LOW drives the DALI bus LOW.
 *
 * DALI_CFG_RX_INVERTED
 *   0 (default, standard interface): MCU_RXD level follows the bus level
 *     (HIGH = bus idle/high, LOW = bus active/low).
 *   1: inverted interface, MCU_RXD = HIGH when the bus is LOW.
 *
 * Note: with the default polarity the on-board pull-down R72 makes a missing
 * DALI board look like a bus held LOW, i.e. a DALI system failure, which is
 * the expected fail-safe behaviour.
 */
#ifndef DALI_CFG_TX_ACTIVE_HIGH
#define DALI_CFG_TX_ACTIVE_HIGH        (1)
#endif

#ifndef DALI_CFG_RX_INVERTED
#define DALI_CFG_RX_INVERTED           (0)
#endif

/* ========================================================================== */
/* Power stage control (OFF signal, PA3)                                      */
/* ========================================================================== */
/*
 * CFG_OFF_ENABLE_HIGH
 *   1 (default, current hardware): OFF = HIGH turns Q5 on and ENABLES the
 *     power stage, OFF = LOW disables it. At reset PA3 is a floating input
 *     and R28 keeps Q5 off, so the converter stays disabled until the
 *     firmware enables it (fail-safe).
 *   0: inverted hardware, OFF = HIGH disables the power stage.
 */
#ifndef CFG_OFF_ENABLE_HIGH
#define CFG_OFF_ENABLE_HIGH            (1)
#endif

/* ========================================================================== */
/* Product / manufacturer information (IEC 62386-102 memory bank 0)          */
/* PLACEHOLDER values: replace with the real product data.                    */
/* ========================================================================== */
/* GTIN, 6 bytes, MSB first (48-bit binary value of the 13-digit GTIN).      */
#define CFG_MB0_GTIN_B0                (0x00U)
#define CFG_MB0_GTIN_B1                (0x00U)
#define CFG_MB0_GTIN_B2                (0x00U)
#define CFG_MB0_GTIN_B3                (0x00U)
#define CFG_MB0_GTIN_B4                (0x00U)
#define CFG_MB0_GTIN_B5                (0x01U)

#define CFG_FW_VERSION_MAJOR           (0x01U)
#define CFG_FW_VERSION_MINOR           (0x00U)
#define CFG_HW_VERSION_MAJOR           (0x01U)
#define CFG_HW_VERSION_MINOR           (0x00U)

/*
 * Identification number (MB0 0x0B..0x12, 8 bytes): the combination GTIN +
 * identification number shall be globally unique. When set to 1 the firmware
 * derives it from the STM32 96-bit unique device ID (recommended); when 0 the
 * constant CFG_MB0_ID_NUMBER is used (only for debugging).
 */
#define CFG_MB0_ID_FROM_UID            (1)
#define CFG_MB0_ID_NUMBER              (0x0000000000000001ULL)

/* ========================================================================== */
/* IEC 62386-251 memory bank 1 factory defaults (luminaire information)       */
/* PLACEHOLDER values: the luminaire manufacturer normally writes them over   */
/* the bus at end of line. 0xFF / 0x00 = "not specified" as per IEC 62386-251.*/
/* ========================================================================== */
#define CFG_MB1_CONTENT_FORMAT_ID      (0x0003U)
#define CFG_MB1_YEAR_OF_MANUFACTURE    (0xFFU)
#define CFG_MB1_WEEK_OF_MANUFACTURE    (0xFFU)
#define CFG_MB1_INPUT_POWER_NOMINAL_W  (0xFFFFU)
#define CFG_MB1_INPUT_POWER_MIN_DIM_MW (0xFFFFU)
#define CFG_MB1_MAINS_V_MIN            (0xFFFFU)
#define CFG_MB1_MAINS_V_MAX            (0xFFFFU)
#define CFG_MB1_LIGHT_OUTPUT_LM        (0xFFFFFFUL)
#define CFG_MB1_CRI                    (0xFFU)
#define CFG_MB1_CCT_K                  (0xFFFFU)
#define CFG_MB1_LIGHT_DISTRIBUTION     (0xFFU)

/* ========================================================================== */
/* Dimming / light output                                                     */
/* ========================================================================== */
/* PLACEHOLDER: physical minimum level (PHM, 1..254) reachable by hardware.   */
#define CFG_PHYSICAL_MIN_LEVEL         (85U)

/* Minimum fast fade time supported (IEC 62386-207, units of 25 ms, 1..27).   */
#define CFG_MIN_FAST_FADE_TIME         (1U)

/* PWM_OUT (TIM14 CH1) resolution: 48 MHz / 4096 = 11.7 kHz, 12 bit.          */
#define CFG_PWM_PERIOD_COUNTS          (4096U)
#define CFG_PWM_MAX_DUTY               (CFG_PWM_PERIOD_COUNTS)

/*
 * Output current calibration vector (linear, calibratable).
 * CFG_PWM_CAL_POINTS points equally spaced in relative output current
 * (0 %, 10 %, ... 100 % of the nominal variant current). Each entry is the
 * PWM_OUT compare value (0..CFG_PWM_MAX_DUTY) that produces that current.
 * Higher duty = lower LED current (PWM_OUT offsets the current sense).
 * Values between two points are linearly interpolated.
 * PLACEHOLDER: ideal linear characteristic, replace with measured data.
 */
#define CFG_PWM_CAL_POINTS             (11U)
#define CFG_PWM_CAL_TABLE                                                     \
  {                                                                           \
    4096U, 3686U, 3277U, 2867U, 2458U, 2048U, 1638U, 1229U, 819U, 410U, 0U    \
  }

/* ========================================================================== */
/* Lamp variants (current limitation selection, SN74HCS596 Q0..Q5)           */
/* ========================================================================== */
#define CFG_VARIANT_COUNT              (17U)
/* Variant used when no valid variant is stored: Req = 300 kOhm (lowest       */
/* LED current, i.e. the safest choice).                                      */
#define CFG_VARIANT_DEFAULT            (0U)

/*
 * PLACEHOLDER: nominal LED current (mA) of each variant at 100 % level.
 * Used only for the energy / current estimation of IEC 62386-252/253.
 */
#define CFG_VARIANT_NOMINAL_CURRENT_MA                                        \
  {                                                                           \
    350U, 400U, 450U, 500U, 550U, 600U, 650U, 700U, 750U,                     \
    800U, 850U, 900U, 950U, 1000U, 1050U, 1100U, 1150U                        \
  }

/* ========================================================================== */
/* NFC (NTAG 5 link NTP5332) variant configuration block                      */
/* ========================================================================== */
#define CFG_NTAG5_I2C_ADDR             (0x54U)
/* Block address (4-byte blocks) of the 8-byte variant record in the NTAG 5   */
/* user EEPROM. The same block number is used on the RF (ISO 15693) side.     */
#define CFG_NFC_VARIANT_BLOCK          (0x01F0U)
/* Delay after the last NFC event before the tag is read (ms).                */
#define CFG_NFC_EVENT_SETTLE_MS        (500U)
/* Periodic fall-back poll of the tag (ms).                                    */
#define CFG_NFC_POLL_PERIOD_MS         (5000U)

/* ========================================================================== */
/* LED output monitoring (uVLED on PA1) - PLACEHOLDER thresholds              */
/* ========================================================================== */
/* uVLED divider: R64 430k + R65 390k over R66 6k2.                           */
#define CFG_UVLED_DIV_NUM              (826200UL)
#define CFG_UVLED_DIV_DEN              (6200UL)

#define CFG_FAULT_SHORT_CIRCUIT_MV     (5000UL)    /* V_LED below -> short    */
#define CFG_FAULT_UNDER_VOLTAGE_MV     (60000UL)   /* V_LED below -> UV       */
#define CFG_FAULT_OVER_VOLTAGE_MV      (280000UL)  /* V_LED above -> OV       */
#define CFG_FAULT_OPEN_CIRCUIT_MV      (320000UL)  /* V_LED above -> open     */

/* Blanking time after the power stage is enabled before checking faults.     */
#define CFG_FAULT_BLANKING_MS          (1500U)
/* Monitoring sample period and number of consecutive samples to confirm.     */
#define CFG_MONITOR_PERIOD_MS          (5U)
#define CFG_FAULT_DEBOUNCE_SAMPLES     (20U)

/* ========================================================================== */
/* Energy estimation (IEC 62386-252) - PLACEHOLDER                            */
/* ========================================================================== */
#define CFG_EFFICIENCY_PERCENT         (90UL)
#define CFG_STANDBY_POWER_MW           (500UL)

/* ========================================================================== */
/* Non-volatile storage                                                       */
/* ========================================================================== */
/* Delay between the last change of a DALI variable and the EEPROM write.     */
#define CFG_NVM_SAVE_DELAY_MS          (2000U)
/* Period of the operating counters / energy snapshot (ms): 15 minutes.       */
#define CFG_COUNTERS_SAVE_PERIOD_MS    (900000UL)

#endif /* APP_CONFIG_H */

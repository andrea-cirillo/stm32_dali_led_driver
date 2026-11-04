# STM32F030F4 DALI-2 LED driver firmware

Firmware for a 160 W LED driver control board (see
`Docs/Schematics/stm32f030f4_dali_sch.pdf`). The STM32F030F4P6 (16 KB flash,
4 KB RAM) implements a DALI-2 control gear for LED modules.

| Standard            | Content                                         | Status        |
|---------------------|-------------------------------------------------|---------------|
| IEC 62386-101 ed.2  | Physical layer, Manchester coding, timing       | implemented   |
| IEC 62386-102 ed.2  | Control gear general requirements               | implemented   |
| IEC 62386-207 ed.2  | LED modules, device type 6                      | implemented   |
| IEC 62386-251       | Memory bank 1 (luminaire data), device type 50  | implemented   |
| IEC 62386-252       | Energy reporting, MB 202, device type 51        | implemented (estimated values) |
| IEC 62386-253       | Diagnostics and maintenance, MB 205/206, DT 52  | implemented (partial, see below) |

The code is written in C99 against the CMSIS register definitions only:
**STM32Cube HAL is not used**. Without it the full DALI-2 stack fits in the
16 KB device. Coding rules follow MISRA C:2012. The few intentional deviations
are commented in the code: Rule 11.4 for fixed hardware addresses (UID,
factory calibration) and Rule 11.3/11.5 for byte access to the NVM structures.

## Pin map

| Pin  | Signal            | Use                                                     |
|------|-------------------|---------------------------------------------------------|
| PF0  | MCU_TXD           | DALI transmit (GPIO, TIM3 CC2 timing)                   |
| PF1  | MCU_RXD           | DALI receive (EXTI1, TIM3 1 MHz time stamps)            |
| PA0  | NFC_EventDetect   | NTAG 5 ED pin, EXTI0 falling edge                       |
| PA1  | uVLED             | ADC_IN1, LED voltage (divider 826.2k / 6.2k)            |
| PA3  | OFF               | HIGH = power stage off                                  |
| PA4  | PWM_OUT           | TIM14 CH1, 11.7 kHz / 12 bit, higher duty = lower current |
| PA6  | STCP              | SN74HCS596 storage clock                                |
| PA7  | SHCP              | SN74HCS596 shift clock                                  |
| PB1  | CurrentLimit_DS   | SN74HCS596 serial data                                  |
| PA9/PA10 | SCL/SDA       | I2C1 100 kHz: M24C04 (0x50/0x51), NTAG 5 (0x54)          |

## Software architecture

```
Core/   register level drivers
  bsp            clock 48 MHz (HSI/2 x 12), GPIO, SysTick, IWDG, OFF pin
  dali_phy       IEC 62386-101 Manchester RX/TX (EXTI1 + TIM3)
  i2c_bus        blocking I2C1 master with timeouts
  eeprom_m24c04  M24C04 page write + ACK polling
  ntag5          minimal NTAG 5 link access (user memory, session registers)
  shiftreg       SN74HCS596
  adc_drv        uVLED, VREFINT (actual VDDA), internal temperature sensor
  pwm_drv        TIM14 PWM_OUT
App/    application
  app_config.h   ALL product data, thresholds, calibration (placeholders)
  dali_gear      IEC 62386-102 + 207 command handler and state
  dali_membank   memory banks 0, 1, 202, 205, 206
  light_ctrl     fade engine, dimming curves, PWM calibration, identify, fault latch
  led_monitor    open / short / over / under voltage detection, estimates
  opdata         operating counters and energy (253 / 252)
  variant        lamp variant: EEPROM + NFC + shift register
  nvm            EEPROM layout, redundant records with CRC
Test/host/  host unit tests (gcc + sanitizers) with register mocks
```

Execution model (bare metal):

| Context                 | Priority | Work                                                         |
|-------------------------|----------|--------------------------------------------------------------|
| EXTI0_1, TIM3           | 0        | DALI bit timing (RX decoding, TX half bits), NFC ED flag     |
| SysTick (1 ms)          | 1        | DALI command execution, fades, PWM / OFF output              |
| main loop               | -        | ADC monitoring and faults, EEPROM writes, NFC, counters, IWDG |

All DALI variables are owned by the SysTick context. The main loop only
persists snapshots, so a slow EEPROM write can never delay a backward frame.
The backward frame starts 7.0 ms after the end of the forward frame (the
allowed window is 5.5 to 10.5 ms).

## Boot sequence

1. `OFF` is driven HIGH as the first instruction, so the power stage stays off.
2. The clock and GPIO are set up, then PWM is set to maximum duty (minimum current).
3. The shift register is written with the default variant (lowest current).
4. The variant is read from the EEPROM. If the NTAG 5 holds a valid, different
   record, that record is stored first. The shift register is then written
   with the final variant. **This is the only point where the current
   limitation is switched.**
5. Memory bank 1, the counters and the DALI persistent variables are loaded.
6. The DALI PHY and the IWDG (~1 s) are started.
7. The power-on level (IEC 62386-102) is applied and the lamp switches on.

## Configuration (`App/Inc/app_config.h`)

Every value marked **PLACEHOLDER** must be replaced:

- GTIN, firmware and hardware versions (memory bank 0). The identification
  number is derived from the STM32 UID, so it is unique for every device.
- Memory bank 1 factory defaults (IEC 62386-251).
- `CFG_PHYSICAL_MIN_LEVEL` (PHM).
- `CFG_PWM_CAL_TABLE`: the calibration vector. It has 11 points at 0..100 %
  of the nominal current; each point holds the PWM compare value (0..4096),
  and values between points are interpolated linearly.
- `CFG_VARIANT_NOMINAL_CURRENT_MA`: the nominal current of each variant, used
  only for the 252/253 estimates.
- Fault thresholds: `CFG_FAULT_*_MV`, blanking time and debounce.
- Efficiency and standby power used for the energy estimate.
- DALI interface polarity: `DALI_CFG_TX_ACTIVE_HIGH` (default 1: TXD HIGH
  pulls the bus low) and `DALI_CFG_RX_INVERTED` (default 0: RXD follows the
  bus level). Both can be overridden on the compiler command line.

## Lamp variant and NFC

The 17 resistor combinations of `Docs/Schematics/Table_RvsQx.pdf` map to
variant index 0..16. Bit n of the shift register drives Qn, and Q0 (R92) is
always on. Variant 0 (Req 300 kΩ) gives the lowest LED current and is the
fallback when no valid variant is stored.

The end-of-line NFC card writes an 8-byte record into the NTAG 5 user memory
at **block 0x01F0** (`CFG_NFC_VARIANT_BLOCK`, 4-byte blocks, same address on
the RF and I2C sides):

| Byte | Content                                                |
|------|--------------------------------------------------------|
| 0..3 | identification code `"DLVR"` = `44 4C 56 52`           |
| 4    | record format version `01`                             |
| 5    | variant index `00`..`10` (hex)                         |
| 6..7 | CRC-16/CCITT-FALSE of bytes 0..5, MSB first            |

Example records (CRC check value of `"123456789"` = 0x29B1):

```
variant  0: 44 4C 56 52 01 00 A9 DF
variant  7: 44 4C 56 52 01 07 D9 38
variant 16: 44 4C 56 52 01 10 BB EE
```

The MCU reads the tag at boot, 0.5 s after every RF write (the ED pin is set
to "WRITE command" through the volatile session register), and every 5 s as
a fall-back. A valid new record is written immediately to the EEPROM (copies
A and B). The current limitation changes only at the next power-up. The MCU
never writes the tag.

## EEPROM map (M24C04, 512 bytes)

| Address     | Content                                                        |
|-------------|----------------------------------------------------------------|
| 0x000-0x07F | DALI persistent variables, 2 copies, sequence number + CRC16   |
| 0x080-0x0F4 | Memory bank 1, locations 0x03..0x77                            |
| 0x0F6-0x0F7 | Memory bank 1 "initialised" marker                             |
| 0x100/0x110 | Variant record, copies A / B (same format as the NFC record)   |
| 0x120-0x1DF | Operating counters, 4 rotating slots of 48 bytes               |

DALI variables are saved 2 s after the last change, or immediately on SAVE
PERSISTENT VARIABLES. Counters and energy are saved every 15 minutes and after
every fault. The board has no power-fail signal, so at most the last 15
minutes of counters can be lost.

## Fault handling

uVLED is sampled every 5 ms: 4 samples are averaged and scaled with the
measured VDDA. Faults are evaluated only while the power stage is on and at
least 1.5 s after it was switched on. A fault is confirmed after 20
consecutive samples (100 ms):

| Condition                       | IEC 62386-207 failure bit        |
|---------------------------------|----------------------------------|
| V_LED < short threshold         | short circuit (bit 0)            |
| V_LED > open threshold          | open circuit (bit 1)             |
| V_LED > over-voltage threshold  | load increase (bit 3)            |
| V_LED < under-voltage threshold | load decrease (bit 2)            |

Every confirmed fault latches `OFF` until the next reset. lampFailure is set,
and the 253 counters (overall, short, open) are incremented and saved. After
a restart, the converter switches on again and the check repeats: if the fault
is gone, normal operation resumes.

## DALI implementation notes

- **Device types**: QUERY DEVICE TYPE answers MASK, and QUERY NEXT DEVICE TYPE
  answers 6, 50, 51, 52, then 254.
- **207 ed.2 commands**: SELECT DIMMING CURVE, SET FAST FADE TIME, QUERY
  CONTROL GEAR TYPE, QUERY DIMMING CURVE, QUERY FEATURES, QUERY FAILURE
  STATUS, QUERY (MIN) FAST FADE TIME and QUERY EXTENDED VERSION NUMBER. The
  ed.1 commands that ed.2 removed are ignored.
- **Fade time priority**: fade time > 0 is used first, then the 207 fast fade
  time if not 0, then the 102 extended fade time.
- **Not measured, reported as MASK in 253**: supply voltage, frequency, power
  factor, under/over-voltage, power limitation, thermal derating and
  shutdown, light source temperature.
- **Estimated values**: control gear temperature comes from the MCU internal
  sensor. Light source current, output current %, active power and energy are
  estimated from V_LED, the variant nominal current, the dimming level and
  the configured efficiency.
- **Not certified**: the stack still has to be validated with a DALI-2 test
  bench (DiiA test sequences) before certification.

## Build

- **STM32CubeIDE**: import the project. Debug builds with `-Og` (~15.2 KB,
  93 %) and Release with `-Os` (~13.2 KB, 81 %).
- **Command line**: run `make` (needs `arm-none-eabi-gcc`). It produces
  `build/stm32f030f4_dali.elf/.hex/.bin`.
- **Host unit tests**: run `make -C Test/host`. They cover the Manchester
  codec, the 102/207 commands, the memory banks, NVM, variant/NFC, fault latch
  and counters.
- **API documentation**: every function, public and static, carries a
  Doxygen block with description, inputs (`@param[in/out]`), output
  (`@return` / return value), preconditions and execution context. Public
  APIs are fully described in the headers (`Core/Inc`, `App/Inc`). The `.c`
  files document the internal functions and point to the header for the
  public ones. Run `doxygen` from the repository root to get the HTML in
  `build/doc/html` (`doxygen` package needed).

## Open hardware points

1. **Shift register at power-up.** OE is tied low (R107) and MR high (R105),
   so the outputs are enabled with random content until the MCU writes the
   register. The firmware drives `OFF` HIGH first and writes the safe variant
   within a few ms of reset. The converter can still start before the MCU
   runs, though, because R28 pulls the Q5 base low, which means "converter
   ON". Recommended changes: a pull-up on OFF (converter off by default)
   and/or OE driven or pulled up.
2. **I2C speed.** I2C runs at 100 kHz because of the 10 kΩ pull-ups and the
   NFC board connector.
3. **Req values.** The Req computed from the fitted E96 values is about 1 %
   above the "Req target" column of the table. The SN74HCS596 R_on adds a
   little more.
4. **No supply or output current measurement**: the 252/253 values listed
   above are estimates.

 /*
  Copyright (c) 2014-2015 Arduino LLC.  All right reserved.
  Copyright (c) 2016 Sandeep Mistry All right reserved.
  Copyright (c) 2018, Adafruit Industries (adafruit.com)

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_FEATHER52840_
#define _VARIANT_FEATHER52840_

/** Master clock frequency */
#define VARIANT_MCK       (64000000ul)

#define USE_LFXO      // Board uses 32khz crystal for LF
// define USE_LFRC    // Board uses RC for LF

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "WVariant.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

// Number of pins defined in PinDescription array
#define PINS_COUNT           (48)
#define NUM_DIGITAL_PINS     (48)
#define NUM_ANALOG_INPUTS    (0) // A6 is used for battery, A7 is analog reference
#define NUM_ANALOG_OUTPUTS   (0)

// LEDs
// #define PIN_LED1             (3)
// #define PIN_LED2             (4)
// #define PIN_NEOPIXEL         (8)
// #define NEOPIXEL_NUM         1


#define LED_BUILTIN           (15)

// #define LED_BUILTIN          PIN_LED1
// #define LED_CONN             PIN_LED2

#define LED_RED              LED_BUILTIN
#define LED_BLUE             LED_BUILTIN

#define LED_STATE_ON         1         // State when LED is litted

/*
 * Buttons
 */
// #define PIN_BUTTON1          (7)

/*
 * Analog pins
 */
// #define PIN_A0               (14)
// #define PIN_A1               (15)
// #define PIN_A2               (16)
// #define PIN_A3               (17)
// #define PIN_A4               (18)
// #define PIN_A5               (19)
// #define PIN_A6               (20)
// #define PIN_A7               (21)

// static const uint8_t A0  = PIN_A0 ;
// static const uint8_t A1  = PIN_A1 ;
// static const uint8_t A2  = PIN_A2 ;
// static const uint8_t A3  = PIN_A3 ;
// static const uint8_t A4  = PIN_A4 ;
// static const uint8_t A5  = PIN_A5 ;
// static const uint8_t A6  = PIN_A6 ;
// static const uint8_t A7  = PIN_A7 ;
// #define ADC_RESOLUTION    14

// Other pins
// #define PIN_AREF           PIN_A7
// #define PIN_VBAT           PIN_A6
// #define PIN_NFC1           (33)
// #define PIN_NFC2           (2)

// static const uint8_t AREF = PIN_AREF;

/*
 * Serial interfaces
 */
#define PIN_SERIAL1_RX       (1)
#define PIN_SERIAL1_TX       (0)

/*
 * SPI Interfaces
 */
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (24)
#define PIN_SPI_MOSI         (25)
#define PIN_SPI_SCK          (26)

static const uint8_t SS   = (5);
static const uint8_t MOSI = PIN_SPI_MOSI ;
static const uint8_t MISO = PIN_SPI_MISO ;
static const uint8_t SCK  = PIN_SPI_SCK ;

/*
 * Wire Interfaces
 */
#define WIRE_INTERFACES_COUNT 1

#define PIN_WIRE_SDA         (22)
#define PIN_WIRE_SCL         (23)

// QSPI Pins
#define PIN_QSPI_SCK         27
#define PIN_QSPI_CS          28
#define PIN_QSPI_IO0         29
#define PIN_QSPI_IO1         30
#define PIN_QSPI_IO2         31
#define PIN_QSPI_IO3         32

// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

#define PIN_P0_BASE 0
#define PIN_P1_BASE 32

#define PIN_P0_00 (PIN_P0_BASE)
#define PIN_P0_01 (PIN_P0_BASE+1)
#define PIN_P0_02 (PIN_P0_BASE+2)
#define PIN_P0_03 (PIN_P0_BASE+3)
#define PIN_P0_04 (PIN_P0_BASE+4)
#define PIN_P0_05 (PIN_P0_BASE+5)
#define PIN_P0_06 (PIN_P0_BASE+6)
#define PIN_P0_07 (PIN_P0_BASE+7)
#define PIN_P0_08 (PIN_P0_BASE+8)
#define PIN_P0_09 (PIN_P0_BASE+9)
#define PIN_P0_10 (PIN_P0_BASE+10)
#define PIN_P0_11 (PIN_P0_BASE+11)
#define PIN_P0_12 (PIN_P0_BASE+12)
#define PIN_P0_13 (PIN_P0_BASE+13)
#define PIN_P0_14 (PIN_P0_BASE+14)
#define PIN_P0_15 (PIN_P0_BASE+15)
#define PIN_P0_16 (PIN_P0_BASE+16)
#define PIN_P0_17 (PIN_P0_BASE+17)
#define PIN_P0_18 (PIN_P0_BASE+18)
#define PIN_P0_19 (PIN_P0_BASE+19)
#define PIN_P0_20 (PIN_P0_BASE+20)
#define PIN_P0_21 (PIN_P0_BASE+21)
#define PIN_P0_22 (PIN_P0_BASE+22)
#define PIN_P0_23 (PIN_P0_BASE+23)
#define PIN_P0_24 (PIN_P0_BASE+24)
#define PIN_P0_25 (PIN_P0_BASE+25)
#define PIN_P0_26 (PIN_P0_BASE+26)
#define PIN_P0_27 (PIN_P0_BASE+27)
#define PIN_P0_28 (PIN_P0_BASE+28)
#define PIN_P0_29 (PIN_P0_BASE+29)
#define PIN_P0_30 (PIN_P0_BASE+30)
#define PIN_P0_31 (PIN_P0_BASE+31)

#define PIN_P1_00 (PIN_P1_BASE)
#define PIN_P1_01 (PIN_P1_BASE+1)
#define PIN_P1_02 (PIN_P1_BASE+2)
#define PIN_P1_03 (PIN_P1_BASE+3)
#define PIN_P1_04 (PIN_P1_BASE+4)
#define PIN_P1_05 (PIN_P1_BASE+5)
#define PIN_P1_06 (PIN_P1_BASE+6)
#define PIN_P1_07 (PIN_P1_BASE+7)
#define PIN_P1_08 (PIN_P1_BASE+8)
#define PIN_P1_09 (PIN_P1_BASE+9)
#define PIN_P1_10 (PIN_P1_BASE+10)
#define PIN_P1_11 (PIN_P1_BASE+11)
#define PIN_P1_12 (PIN_P1_BASE+12)
#define PIN_P1_13 (PIN_P1_BASE+13)
#define PIN_P1_14 (PIN_P1_BASE+14)
#define PIN_P1_15 (PIN_P1_BASE+15)

#ifdef __cplusplus
}
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#endif

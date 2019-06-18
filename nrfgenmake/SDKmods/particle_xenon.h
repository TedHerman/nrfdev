#ifndef PCA10056_H
#define PCA10056_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define _PINNUM(port, pin)    ((port)*32 + (pin))

#define LEDS_NUMBER 1
#define LED_PRIMARY_PIN _PINNUM(1, 12)
#define LED_1  NRF_GPIO_PIN_MAP(1,12)
#define BOARD_RGB_BRIGHTNESS  0x040404

#define LED_RGB_RED_PIN _PINNUM(0,13)
#define LED_RGB_GREEN_PIN _PINNUM(0,14)
#define LED_RGB_BLUE_PIN _PINNUM(0,15)

// other source code has 0x101010 instead
#define LED_STATE_ON          1
#define LED_START             LED_1
#define LED_STOP              LED_1
#define LEDS_ACTIVE_STATE 1
#define LEDS_LIST { LED_1 }
#define LEDS_INV_MASK  LEDS_MASK
#define BSP_LED_0      LED_PRIMARY_PIN 


#define BUTTONS_NUMBER 2
#define BUTTON_1 _PINNUM(0,11) 
#define BUTTON_2 _PINNUM(0,03) 
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
#define BUTTONS_ACTIVE_STATE 0
#define BUTTONS_LIST { BUTTON_1 }
#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  8 
#define TX_PIN_NUMBER  6 
#define CTS_PIN_NUMBER 0
#define RTS_PIN_NUMBER 0
#define HWFC           false

/*
 
#define BSP_QSPI_SCK_PIN   19
#define BSP_QSPI_CSN_PIN   9
#define BSP_QSPI_IO0_PIN   20
#define BSP_QSPI_IO1_PIN   21
#define BSP_QSPI_IO2_PIN   22
#define BSP_QSPI_IO3_PIN   23


// serialization APPLICATION board - temp. setup for running serialized MEMU tests
#define SER_APP_RX_PIN              NRF_GPIO_PIN_MAP(0, 15)  // UART RX pin number.
#define SER_APP_TX_PIN              NRF_GPIO_PIN_MAP(0, 17)  // UART TX pin number.
#define SER_APP_CTS_PIN             NRF_GPIO_PIN_MAP(0, 19)  // UART Clear To Send pin number.
#define SER_APP_RTS_PIN             NRF_GPIO_PIN_MAP(0, 20)  // UART Request To Send pin number.

#define SER_APP_SPIM0_SCK_PIN       NRF_GPIO_PIN_MAP(0,30)   // SPI clock GPIO pin number.
#define SER_APP_SPIM0_MOSI_PIN      NRF_GPIO_PIN_MAP(0,3)    // SPI Master Out Slave In GPIO pin number
#define SER_APP_SPIM0_MISO_PIN      NRF_GPIO_PIN_MAP(0,31)   // SPI Master In Slave Out GPIO pin number
#define SER_APP_SPIM0_SS_PIN        NRF_GPIO_PIN_MAP(1,2)    // SPI Slave Select GPIO pin number
#define SER_APP_SPIM0_RDY_PIN       NRF_GPIO_PIN_MAP(1,15)   // SPI READY GPIO pin number
#define SER_APP_SPIM0_REQ_PIN       NRF_GPIO_PIN_MAP(1,14)   // SPI REQUEST GPIO pin number

// serialization CONNECTIVITY board
#define SER_CON_RX_PIN              NRF_GPIO_PIN_MAP(1,14)   // UART RX pin number.
#define SER_CON_TX_PIN              NRF_GPIO_PIN_MAP(1,13)   // UART TX pin number.
#define SER_CON_CTS_PIN             NRF_GPIO_PIN_MAP(1,15)   // UART Clear To Send pin number. Not used if HWFC is set to false.
#define SER_CON_RTS_PIN             NRF_GPIO_PIN_MAP(0,2)    // UART Request To Send pin number. Not used if HWFC is set to false.


#define SER_CON_SPIS_SCK_PIN        NRF_GPIO_PIN_MAP(0,27)   // SPI SCK signal.
#define SER_CON_SPIS_MOSI_PIN       NRF_GPIO_PIN_MAP(0,2)    // SPI MOSI signal.
#define SER_CON_SPIS_MISO_PIN       NRF_GPIO_PIN_MAP(0,26)   // SPI MISO signal.
#define SER_CON_SPIS_CSN_PIN        NRF_GPIO_PIN_MAP(1,13)   // SPI CSN signal.
#define SER_CON_SPIS_RDY_PIN        NRF_GPIO_PIN_MAP(1,15)   // SPI READY GPIO pin number.
#define SER_CON_SPIS_REQ_PIN        NRF_GPIO_PIN_MAP(1,14)   // SPI REQUEST GPIO pin number.

#define SER_CONN_CHIP_RESET_PIN     NRF_GPIO_PIN_MAP(1,1)    // Pin used to reset connectivity chip

// Arduino board mappings
#define ARDUINO_SCL_PIN             11   // SCL signal pin
#define ARDUINO_SDA_PIN             8    // SDA signal pin
#define ARDUINO_AREF_PIN            2    // Aref pin

#define ARDUINO_13_PIN              NRF_GPIO_PIN_MAP(0, 2)  // Digital pin 13
#define ARDUINO_12_PIN              NRF_GPIO_PIN_MAP(0, 3)  // Digital pin 12
#define ARDUINO_11_PIN              NRF_GPIO_PIN_MAP(0, 31)  // Digital pin 11
#define ARDUINO_10_PIN              NRF_GPIO_PIN_MAP(0, 30)  // Digital pin 10
#define ARDUINO_9_PIN               NRF_GPIO_PIN_MAP(0, 10)  // Digital pin 9
#define ARDUINO_8_PIN               NRF_GPIO_PIN_MAP(0, 9)  // Digital pin 8

#define ARDUINO_7_PIN               NRF_GPIO_PIN_MAP(0, 23) // Digital pin 7
#define ARDUINO_6_PIN               NRF_GPIO_PIN_MAP(0, 22) // Digital pin 6
#define ARDUINO_5_PIN               NRF_GPIO_PIN_MAP(0, 21) // Digital pin 5
#define ARDUINO_4_PIN               NRF_GPIO_PIN_MAP(0, 20) // Digital pin 4
#define ARDUINO_3_PIN               NRF_GPIO_PIN_MAP(0, 19) // Digital pin 3
#define ARDUINO_2_PIN               NRF_GPIO_PIN_MAP(1, 3) // Digital pin 2
#define ARDUINO_1_PIN               NRF_GPIO_PIN_MAP(0, 15) // Digital pin 1
#define ARDUINO_0_PIN               NRF_GPIO_PIN_MAP(0,17) // Digital pin 0

#define ARDUINO_A0_PIN              29   // Analog channel 0
#define ARDUINO_A1_PIN              28   // Analog channel 1
#define ARDUINO_A2_PIN              5    // Analog channel 2
#define ARDUINO_A3_PIN              4    // Analog channel 3
#define ARDUINO_A4_PIN              2    // Analog channel 4
#define ARDUINO_A5_PIN              3    // Analog channel 5

*/


#ifdef __cplusplus
}
#endif

#endif // PCA10056_H

/*
  // D0 .. D13
  25,  // D0  is P0.25 (UART TX)
  24,  // D1  is P0.24 (UART RX
  10,  // D2  is P0.10 (NFC2)
  47,  // D3  is P1.15 (LED1)
  42,  // D4  is P1.10 (LED2)
  40,  // D5  is P1.08
   7,  // D6  is P0.07
  34,  // D7  is P1.02 (Button)
  16,  // D8  is P0.16 (NeoPixel)
  26,  // D9  is P0.26
  27,  // D10 is P0.27
   6,  // D11 is P0.06
   8,  // D12 is P0.08
  41,  // D13 is P1.09

  // D14 .. D21 (aka A0 .. A7)
   4,  // D14 is P0.04 (A0)
   5,  // D15 is P0.05 (A1)
  30,  // D16 is P0.30 (A2)
  28,  // D17 is P0.28 (A3)
   2,  // D18 is P0.02 (A4)
   3,  // D19 is P0.03 (A5)
  29,  // D20 is P0.29 (A6, Battery)
  31,  // D21 is P0.31 (A7, ARef)

  // D22 .. D23 (aka I2C pins)
  12,  // D22 is P0.12 (SDA)
  11,  // D23 is P0.11 (SCL)

  // D24 .. D26 (aka SPI pins)
  15,  // D24 is P0.15 (SPI MISO)
  13,  // D25 is P0.13 (SPI MOSI)
  14,  // D26 is P0.14 (SPI SCK )

  // QSPI pins (not exposed via any header / test point)
  19,  // D27 is P0.19 (QSPI CLK)
  20,  // D28 is P0.20 (QSPI CS)
  17,  // D29 is P0.17 (QSPI Data 0)
  22,  // D30 is P0.22 (QSPI Data 1)
  23,  // D31 is P0.23 (QSPI Data 2)
  21,  // D32 is P0.21 (QSPI Data 3)

  // The remaining NFC pin
   9,  // D33 is P0.09 (NFC1, exposed only via test point on bottom of board)

  // Thus, there are 34 defined pins

  // The remaining pins are not usable:
  //
  //
  // The following pins were never listed as they were considered unusable
  //  0,      // P0.00 is XL1   (attached to 32.768kHz crystal)
  //  1,      // P0.01 is XL2   (attached to 32.768kHz crystal)
  // 18,      // P0.18 is RESET (attached to switch)
  // 32,      // P1.00 is SWO   (attached to debug header)
  //
  // The remaining pins are not connected (per schematic)
  // 33,      // P1.01 is not connected per schematic
  // 35,      // P1.03 is not connected per schematic
  // 36,      // P1.04 is not connected per schematic
  // 37,      // P1.05 is not connected per schematic
  // 38,      // P1.06 is not connected per schematic
  // 39,      // P1.07 is not connected per schematic
  // 43,      // P1.11 is not connected per schematic
  // 44,      // P1.12 is not connected per schematic
  // 45,      // P1.13 is not connected per schematic
  // 46,      // P1.14 is not connected per schematic
  */

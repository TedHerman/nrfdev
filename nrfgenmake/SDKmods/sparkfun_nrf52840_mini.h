#ifndef PCA10056_H
#define PCA10056_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define LEDS_NUMBER    2

#define LED_1          NRF_GPIO_PIN_MAP(0,7)
#define LED_2          NRF_GPIO_PIN_MAP(0,14)
#define LED_START      LED_1
#define LED_STOP       LED_2

#define LEDS_ACTIVE_STATE 1

#define LEDS_LIST { LED_1, LED_2 }

#define LEDS_INV_MASK  LEDS_MASK

#define BSP_LED_0      7
#define BSP_LED_1      7

#define BUTTONS_NUMBER 1

#define BUTTON_1       13
#define BUTTON_2 
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }

#define BSP_BUTTON_0   BUTTON_1

#define RX_PIN_NUMBER  15
#define TX_PIN_NUMBER  17
#define CTS_PIN_NUMBER 19
#define RTS_PIN_NUMBER 20
#define HWFC           false

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


#ifdef __cplusplus
}
#endif

#endif // PCA10056_H

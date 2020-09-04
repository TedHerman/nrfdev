#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "bsp.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rng.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"
#include "nrf_pwr_mgmt.h"
#include "nrf.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
 
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

void uart_error_handle(app_uart_evt_t * p_event) {
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_communication);
    }
  else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_code);
    }
  }


// define scheduler parameters
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE            10
// define timer for blinking
APP_TIMER_DEF(blink_timer_id);      
APP_TIMER_DEF(gen_timer_id);      

// clockapi references
extern void clockapi_init();
extern uint32_t get_clock();
extern void message_clock_set(void * parameter, uint16_t size);
extern uint32_t fresh_epoch_clock;
extern uint8_t clock_buffer[4];
extern uint8_t ds3231clock[6];  // for sec,min,hour,day,month,year

// define universal err_code variable once
ret_code_t err_code;

// date/time values
uint32_t newclock;  // will be new value for clock
uint8_t buffer[10]; // ascii of uint32_t using hexadecimal + \r\n 
uint8_t buflen;
bool show;

void buffclock() {
    uint8_t i,j,digit;
    uint8_t localcopy[6];
    char c;
    newclock = get_clock();
    memcpy(localcopy,ds3231clock,6);
    // DEBUG
    for (i=0; i<8; i++) {
      digit = newclock >> (32-4);
      newclock <<= 4;
      if (digit < 0xa) c = digit + '0';
      else c = (digit - 0xa) + 'a'; 
      buffer[i] = c;
      }
    buffer[8] = '\r';
    buffer[9] = '\n';

    i = 0;
    for (j=2; j<6; j++) {
      digit = localcopy[j] & 0xf;
      if (digit < 0xa) c = digit + '0';
      else c = (digit - 0xa) + 'a'; 
      buffer[i] = c;
      digit = (localcopy[j] >> 4) & 0xf;
      if (digit < 0xa) c = digit + '0';
      else c = (digit - 0xa) + 'a'; 
      buffer[i+1] = c;
      i += 2;
      }

    // DEBUG show = false;
    app_timer_start(gen_timer_id,APP_TIMER_TICKS(100), NULL);
    }

void showdigit(void * parameter, uint16_t size) {
    if (buflen >= 10) {
       buflen = 0;
       return;
       }
    app_uart_put(buffer[buflen]);
    buflen++;
    app_timer_start(gen_timer_id,APP_TIMER_TICKS(100), NULL);
    }

void gatherdigits(void * parameter, uint16_t size) {
    uint8_t character;
    err_code = app_timer_start(blink_timer_id,
               APP_TIMER_TICKS(10), NULL);
    APP_ERROR_CHECK(err_code);
    // attempt to read some input character
    err_code = app_uart_get(&character);
    if (err_code != NRF_SUCCESS) return;
    if (buflen >= 8) {
      buflen = 0;
      return; 
      }
    // add character to buffer
    buffer[buflen] = character;
    buflen++;
    // test for completion
    if (buflen < 8) return;
    // construct new clock
    newclock = 0;
    for (int i=0;i<8;i++) {
      uint8_t nibble = 0;
      if (buffer[i]>='0' && buffer[i]<='9') nibble = buffer[i]-'0'; 
      else if (buffer[i]>='a' && buffer[i]<='f') nibble = 10 + (buffer[i]-'a'); 
      else if (buffer[i]>='A' && buffer[i]<='F') nibble = 10 + (buffer[i]-'A'); 
      else {
        bsp_board_led_off(0);
        bsp_board_led_off(1);
        bsp_board_led_off(2);
        bsp_board_led_off(3);
	while (true) { };
        }
      newclock = (newclock << 4) | nibble;
      }
    if (((newclock >> 24) & 0xff) != 0x5f) {
       bsp_board_led_off(0);
       bsp_board_led_on(1);
       bsp_board_led_off(2);
       bsp_board_led_off(3);
       while (true) { };
       }
    bsp_board_led_off(0);
    bsp_board_led_off(1);
    bsp_board_led_off(2);
    bsp_board_led_on(3);
    err_code = app_uart_put('>');
    // right here, one could check validity (minute < 60, 1 < day < 32, etc)
    // but this is too tricky when considering month/day/leap combinations
    // write new clock value to DS3231
    memset(clock_buffer,0,4);
    fresh_epoch_clock = newclock;
    app_sched_event_put(NULL,1,&message_clock_set);
    show = true;
    buflen = 0;
    }

// callback from timer expiration
void blink_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&gatherdigits);
    }

void gen_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&showdigit);
    }	

// handler called regularly by clockapi, once per second
void auxiliary_tick_handler(uint32_t t) {
  if (show) buffclock(); 
  }

void initialize() {
    // configure board for leds, set up clock and rng 
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_on(0);
    bsp_board_led_on(1);
    bsp_board_led_on(2);
    bsp_board_led_on(3);
    const app_uart_comm_params_t comm_params = {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          0, // RTS_PIN_NUMBER,
          0, // CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200
          };
    APP_UART_FIFO_INIT(&comm_params,
          UART_RX_BUF_SIZE,
          UART_TX_BUF_SIZE,
          uart_error_handle,
          APP_IRQ_PRIORITY_LOWEST,
          err_code);
    APP_ERROR_CHECK(err_code);
    buflen = 0;  // currently input buffer is empty
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);
    // use power management later in main loop, so init here
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    // initialize app scheduler and app timer, with initial timer
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&blink_timer_id,
               APP_TIMER_MODE_SINGLE_SHOT, 
               blink_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&gen_timer_id,
               APP_TIMER_MODE_SINGLE_SHOT, 
               gen_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // clear UART state
    app_uart_flush();
    err_code = app_timer_start(blink_timer_id,
               APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
    // DS3231 module init 
    clockapi_init();
    }

int main(void) {
    initialize();
    // show = true;
    for (;;) {
	app_sched_execute();
	NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
        }
    }

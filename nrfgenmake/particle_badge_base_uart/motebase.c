#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrfx_wdt.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_802154.h"
#include "nrf_radio.h"
#include "nrf_802154_types.h"
#include "nrf_802154_clock.h"
#include "boards.h"
#include "IEEE802154.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_uart.h"
#include "nrf_uarte.h"


#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define MAX_MESSAGE_SIZE 37 
#define CHANNEL 26 
#define PACKET_MAX_PAYLOAD 26 
#define PACKET_POOL_SIZE 256

#define APP_BLE_CONN_CFG_TAG 1 // identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3 
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

// for mote 802.15.4    
typedef uint16_t mac_addr_t;
struct motepacket {
  uint8_t length;
  uint16_t fcf;
  uint8_t data_seq_no;
  mac_addr_t pan;
  mac_addr_t dest;
  mac_addr_t src;
  uint8_t iframe;    // field for 6lowpan, is 0x3f for TinyOS
  uint8_t type;
  uint8_t data[PACKET_MAX_PAYLOAD];
  uint8_t fcs[2];    // will become CRC, don't count this in length.
  };
typedef struct motepacket motepacket_t;

struct mote_received {
  uint8_t mote_id;   // truncated id of other mote
  int8_t rssi;       // rssi value on message
  };
typedef struct mote_received mote_received_t;

struct mote_report {
  uint32_t time;     // timestamp of this report
  uint8_t mote_id;   // truncated id of reporting mote
  uint8_t num_reports;  // how many reports we have
  mote_received_t reports[6];  // possibly six other mote reports
  };
typedef struct mote_report mote_report_t;

struct recpacket {
  motepacket_t packet;
  bool occupied;
  int8_t rssi;
  uint32_t time;
  };
typedef struct recpacket recpacket_t;

recpacket_t packetpool[PACKET_POOL_SIZE]; 
uint32_t packet_count;

void pool_init() {
  for (int i=0; i<PACKET_POOL_SIZE; i++) packetpool[i].occupied = false;
  }

APP_TIMER_DEF(mote_timer_id);
APP_TIMER_DEF(server_clock_id);

uint32_t clock = 0;
uint8_t data_seq_no = 0;
motepacket_t sync_message;
uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * packetptr = (motepacket_t *)buffer;

// int8_t  powervalues[] = {-30,-20,-16,-12,-8,-4,0,4};

uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t short_address[]    = {0x06, 0x07};
uint8_t pan_id[]           = {0x04, 0x05};

uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * p = (motepacket_t *)&buffer;


void clock_bcast(void);
static void mote_timeout_handler(void * p_context);
void motesetup(void);

void uart_error_handle(app_uart_evt_t * p_event) {
  if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_communication);
    }
  else if (p_event->evt_type == APP_UART_FIFO_ERROR) {
    APP_ERROR_HANDLER(p_event->data.error_code);
    }
  }

void hdlc_send_uart(uint8_t * p, uint8_t size) {
    // ref: https://en.wikipedia.org/wiki/High-Level_Data_Link_Control
    if (size == 0) return;   
    app_uart_put(0x7e); 
    while (size > 0) {
      if (*p == 0x7e) { app_uart_put(0x7d); app_uart_put(0x5e); }
      else if (*p == 0x7d) { app_uart_put(0x7d); app_uart_put(0x5d); }
      else app_uart_put(*p);
      p++;
      size--;
      }
    app_uart_put(0x7e); 
    }

void get_uart_clock(void * parameter, uint16_t size) {
    ret_code_t err_code;
    uint8_t character;
    uint8_t buffer[4];
    uint32_t newclock; 
    uint32_t i;
    // we expect first four zeroes (not a valid clock)
    for (i=0; i<4; i++) {
       err_code = app_uart_get(&character);
       if (err_code != NRF_SUCCESS || character != 0) return; 
       }
    // then four clock digits 
    for (i=0; i<4; i++) {
       err_code = app_uart_get(&character);
       if (err_code != NRF_SUCCESS) return; 
       buffer[i] = character; 
       }
    // convert to host mode  
    newclock = 0;
    for (i=0; i<4; i++) {
       newclock *= 256;
       newclock += buffer[i];
       }
    clock = newclock;
    clock_bcast();
    }

void nrf_802154_transmitted_raw(const uint8_t *p_frame,
                uint8_t *p_ack, int8_t power, uint8_t lqi) {
    nrf_802154_receive();
    }
void nrf_802154_transmit_failed (const uint8_t *p_frame, nrf_802154_tx_error_t error) {
    nrf_802154_receive();
    }

void clock_bcast() {
    memset((uint8_t*)&sync_message,0,sizeof(motepacket_t));
    sync_message.fcf = 0x0000;
    sync_message.fcf &= 1 << IEEE154_FCF_ACK_REQ;
    sync_message.fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    sync_message.data_seq_no = data_seq_no++;
    sync_message.pan = 0x22;     // GROUP
    sync_message.dest = 0xFFFF;  // BCAST_ADDR
    sync_message.src = 0;        // ID of sender (base station)
    sync_message.iframe = 0x3f;  // 6LowPan designator
    sync_message.type = 0x36;    // 0x36 means time sync 
    // -1 for length, +2 for unknown reason - seems to sort of work,
    // perhaps because of the CRC bytes in the fcs?
    sync_message.length =  offsetof(motepacket_t,data) + 2 + 4 - 1;
    memcpy(sync_message.data,(uint8_t*)&clock,4);
    nrf_802154_transmit_raw((uint8_t*)&sync_message,true);
    nrfx_wdt_feed();
    }

void ble_stack_init(void) {
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    }

void moterestart(void) {
    ret_code_t err_code;
    nrf_802154_deinit();
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
    ble_stack_init();
    nrf_drv_clock_init();
    nrf_802154_init();
    if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    }

void motedump(void * parameter, uint16_t size) {
    recpacket_t * k;
    motepacket_t * m;
    // mote_report_t * p;
    // mote_received_t * q;
    for (int n=0; n<PACKET_POOL_SIZE; n++) {
       if (!packetpool[n].occupied) continue;
       k = &packetpool[n];
       m = &k->packet;
       // p = (mote_report_t *)m->data;
       hdlc_send_uart(m->data,sizeof(mote_report_t));
       /*
       // NRF_LOG_INFO("Packet count %d time %d rssi %d",
		    // packet_count, packetpool[n].time, packetpool[n].rssi);
       NRF_LOG_INFO("mote %d (time %d)",p->mote_id,p->time);
       for (int j=0; j<p->num_reports; j++) {
	   q = &p->reports[j];
	   NRF_LOG_INFO("  heard other mote %d (rssi %d)",
	       q->mote_id, q->rssi);
           }
       */
       packetpool[n].occupied = false;
       packet_count++;
       }
    }

// void do_test_uart(void * parameter, uint16_t size) { test_uart(); }
void set_mote_receive(void * parameter, uint16_t size) { nrf_802154_receive(); }
void nrf_802154_received_timestamp_raw(uint8_t * p_data, int8_t power, uint8_t lqi, uint32_t time) {
    motepacket_t * m = (motepacket_t *) p_data;
    uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t);
    uint32_t n;
    UNUSED_PARAMETER(lqi);
    bsp_board_led_invert(0);
    if (m->type == 0x37) {
       for (n=0; n<PACKET_POOL_SIZE; n++) {
          if (!packetpool[n].occupied) break;
          }
       if (n < PACKET_POOL_SIZE) {
          memcpy((uint8_t*)&packetpool[n].packet,p_data,size);
          packetpool[n].occupied = true;
          packetpool[n].rssi = power;
          packetpool[n].time = time;
          app_sched_event_put(NULL,1,&motedump);
          }
       }
    nrf_802154_buffer_free_raw(p_data);
    // temporarily put state of 802154 stack into sleep, then in just
    // microseconds later, renew the receive state -- this seems to trick
    // the stack into asking SDH for a new RAAL timeslot, so we continue
    // to receive (total continuous timeslot is 120 seconds)
    nrf_802154_sleep();
    app_sched_event_put(NULL,1,&set_mote_receive);
    // app_sched_event_put(NULL,1,&do_test_uart);
    }

void motesetup() {
    ble_stack_init();
    nrf_drv_clock_init();
    nrf_802154_init();
    if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    }

/*
void nrf_802154_tx_started (const uint8_t *p_frame) {
    NRF_LOG_INFO("802154 tx started");
    NRF_LOG_FLUSH();
    }
    */

static void mote_timeout_handler(void * p_context) {
    // app_sched_event_put(NULL,1,&motetx);
    // NRF_LOG_INFO("timeout scheduled event");
    }
static void server_timeout_handler(void * p_context) {
    clock++;  // once per second to keep track of time
    app_sched_event_put(NULL,1,&get_uart_clock);
    }

void leds_init(void) {
    // ret_code_t err_code;
    bsp_board_init(BSP_INIT_LEDS);
    }

void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    }

void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    }

static void timers_init(void) {
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    }

void uart_init(void) {
    ret_code_t err_code;
    /* this is for maybe Spark or Feather
    const app_uart_comm_params_t comm_params = {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          0, // RTS_PIN_NUMBER,
          0, // CTS_PIN_NUMBER,
          UART_HWFC,
          false,
          NRF_UART_BAUDRATE_115200
          };
    */
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
    }

void watchdog_handler(void) { };
void watchdog_init(void) {
    // NOTE - watchdog timeout is set for 15 seconds in
    // sdk_config.h, but could have been set here also
    nrfx_wdt_channel_id m_channel = 0;
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
    uint32_t err_code = nrfx_wdt_init(&config,watchdog_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
    }

int main(void) {
    ret_code_t err_code;
    // stifle log_init to test uart on the nrf52840dk board
    // log_init();
    // NRF_LOG_INFO("base starting");
    // NRF_LOG_FLUSH();
    timers_init();
    leds_init();
    pool_init();
    uart_init();
    power_management_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    watchdog_init();
    motesetup();
    err_code = app_timer_create(&mote_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                mote_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&server_clock_id,
		                APP_TIMER_MODE_REPEATED,
                                server_timeout_handler);
    APP_ERROR_CHECK(err_code);
    app_timer_start(server_clock_id,APP_TIMER_TICKS(1000),NULL);

    // Enter main loop.
    for (;;) {
	app_sched_execute();
  	sd_app_evt_wait();
        // nrf_pwr_mgmt_run();
        }
    }


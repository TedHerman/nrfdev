#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"

#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "app_error.h"
#include "app_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf_802154.h"
#include "nrf_radio.h"
#include "nrf_802154_types.h"
#include "nrf_802154_clock.h"
#include "boards.h"
#include "IEEE802154.h"

#include "bsp.h"

#define LED_BLINK_INTERVAL 800

#define MAX_MESSAGE_SIZE 37
#define CHANNEL 26
#define PACKET_MAX_PAYLOAD 26
#define PACKET_POOL_SIZE 256

#define APP_BLE_CONN_CFG_TAG 1 // identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);
#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define READ_SIZE 1

// static char m_rx_buffer[READ_SIZE];
// static char m_tx_buffer[NRF_DRV_USBD_EPSIZE]; // sixty-four bytes

// USB DEFINES START
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);
static char m_cdc_data_array[64];


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

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
  app_error_handler(0xDEADBEEF, line_num, p_file_name);
  }

void get_clock(void * parameter, uint16_t size) {
    /*
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
    */
    }

void timers_init(void) {
  ret_code_t err_code;
  // nrf_drv_clock_lfclk_request(NULL);
  // while(!nrf_drv_clock_lfclk_is_running()) { }
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
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

static void leds_init(void) {
  bsp_board_init(BSP_INIT_LEDS);
  }

static void mote_timeout_handler(void * p_context) {
    //app_sched_event_put(NULL,1,&motetx);
    NRF_LOG_INFO("timeout scheduled event");
    }
static void server_timeout_handler(void * p_context) {
    clock++;  // once per second to keep track of time
    app_sched_event_put(NULL,1,&get_clock);
    }

void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  }

static void power_manage(void) {
  uint32_t err_code = sd_app_evt_wait();
  APP_ERROR_CHECK(err_code);
  }

void motesetup() {
    ble_stack_init();
    // nrf_drv_clock_init();
    nrf_802154_init();
    // if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    }

// USB CODE START
static bool m_usb_connected = false;


/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    // app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
            bsp_board_led_on(0);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            if (m_usb_connected)
            {
            }
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;

        case APP_USBD_EVT_DRV_RESUME:
            break;

        case APP_USBD_EVT_STARTED:
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
        {
            NRF_LOG_INFO("USB power removed");
            bsp_board_led_off(0);
            m_usb_connected = false;
            app_usbd_stop();
        }
            break;

        case APP_USBD_EVT_POWER_READY:
        {
            NRF_LOG_INFO("USB ready");
            m_usb_connected = true;
            app_usbd_start();
        }
            break;

        default:
            break;
    }
}

void usb_init(void) {
  static const app_usbd_config_t usbd_config = {
    .ev_state_proc = usbd_user_ev_handler
    };
  ret_code_t err_code;
  app_usbd_serial_num_generate();
  err_code  = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("USBD BLE UART example started.");

  err_code = app_usbd_init(&usbd_config);
  APP_ERROR_CHECK(err_code);

  app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
  err_code = app_usbd_class_append(class_cdc_acm);
  APP_ERROR_CHECK(err_code);
  }


/** @brief Application main function. */
int main(void) {
    ret_code_t err_code;
    log_init();
    timers_init();
    leds_init();
    pool_init();
    usb_init();

    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    // ble_stack_init();
    motesetup();

    err_code = app_usbd_power_events_enable();
    APP_ERROR_CHECK(err_code);

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
      while (app_usbd_event_queue_process()) { }
      app_sched_execute();
      NRF_LOG_FLUSH();
      power_manage();
      }
    }

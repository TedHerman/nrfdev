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
#include "ble_gap.h"


#define MAX_MESSAGE_SIZE 37 
#define CHANNEL 26 
#define PACKET_MAX_PAYLOAD 26 
#define PACKET_POOL_SIZE 256

#define APP_BLE_CONN_CFG_TAG 1 // identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3 
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

extern int16_t find_mac_addr(ble_gap_addr_t ble_addr);
uint32_t random_write_delay(uint32_t bound);
static void mote_timeout_handler(void * p_context);
void motesetup(void);

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
uint16_t local_mote_id;
uint16_t data_seq_no;
motepacket_t prepare_report; 
motepacket_t old_report; 
uint32_t randvalue;
uint32_t clock;
bool recently_active; 

void pool_init() {
  for (int i=0; i<PACKET_POOL_SIZE; i++) packetpool[i].occupied = false;
  }

APP_TIMER_DEF(mote_timer_id);
APP_TIMER_DEF(transmit_timer_id);
APP_TIMER_DEF(clock_id);
APP_TIMER_DEF(watchdog_id);

uint8_t seqno = 0; 
uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * packetptr = (motepacket_t *)buffer;

// int8_t  powervalues[] = {-30,-20,-16,-12,-8,-4,0,4};

uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t short_address[]    = {0x06, 0x07};
uint8_t pan_id[]           = {0x04, 0x05};

uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * p = (motepacket_t *)&buffer;

static void clock_timeout_handler(void * p_context) { clock++; }

static void watchdog_timeout_handler(void * p_context) {
    if (recently_active) {
       recently_active = false;
       return;
       }
    sd_nvic_SystemReset();
    }

void report_init() {
    mote_report_t * p = (mote_report_t*)prepare_report.data;
    memset((uint8_t*)&prepare_report,0,sizeof(motepacket_t));
    prepare_report.fcf = 0x0000;
    prepare_report.fcf &= 1 << IEEE154_FCF_ACK_REQ;
    prepare_report.fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    prepare_report.data_seq_no = data_seq_no++;
    prepare_report.pan = 0x22;     // GROUP
    prepare_report.dest = 0xFFFF;  // BCAST_ADDR
    prepare_report.src = local_mote_id; // ID of sender
    prepare_report.iframe = 0x3f;  // 6LowPan designator
    prepare_report.type = 0x37;    // (bogus for now, could have meaning later)
    // -1 for length, +2 for unknown reason - seems to sort of work,
    // perhaps because of the CRC bytes in the fcs?
    prepare_report.length =  offsetof(motepacket_t,data) + 2 + sizeof(mote_report_t) - 1;
    p->time = clock;
    p->mote_id = local_mote_id;
    }

void enter_report(recpacket_t * r) {
    mote_report_t * s = (mote_report_t*)prepare_report.data; 
    mote_received_t * q = &s->reports[s->num_reports];
    if (s->num_reports >= 6) return; 
    s->num_reports++;
    q->mote_id = r->packet.src;
    q->rssi = r->rssi; 
    }

void ble_stack_init(void) {
    ret_code_t err_code;
    int16_t moteaddr;
    ble_gap_addr_t ble_addr;
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

    // read the device MAC address
    sd_ble_gap_addr_get(&ble_addr);
    moteaddr = find_mac_addr(ble_addr);
    local_mote_id = moteaddr;
    if (moteaddr < 0) {
       NRF_LOG_INFO("mac address, not in table, is");
       NRF_LOG_HEXDUMP_INFO((uint8_t*)&ble_addr.addr,BLE_GAP_ADDR_LEN);
       NRF_LOG_FLUSH();
       APP_ERROR_CHECK(moteaddr);
       }
    }

void moterestart(void) {
    ret_code_t err_code;
    nrf_802154_deinit();
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
    pool_init();
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
    for (int n=0; n<PACKET_POOL_SIZE; n++) {
       if (!packetpool[n].occupied) continue;
       if (false) {
          NRF_LOG_INFO("Packet time %d rssi %d",
		    packetpool[n].time, packetpool[n].rssi);
          NRF_LOG_HEXDUMP_INFO((uint8_t*)&packetpool[n].packet, 40);
          }
       packetpool[n].occupied = false;
       }
    if (false) NRF_LOG_FLUSH();
    }

void nrf_802154_received_timestamp_raw(uint8_t * p_data, int8_t power, uint8_t lqi, uint32_t time) {
    uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t);
    uint32_t n;
    motepacket_t * q = (motepacket_t *)p_data;
    UNUSED_PARAMETER(lqi);
    if (q->type == 0x36) {
       clock = *(uint32_t *)q->data;
       nrf_802154_buffer_free_raw(p_data);
       return;
       }
    if (q->type != 0x37) {
       nrf_802154_buffer_free_raw(p_data);
       return;
       }
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
    nrf_802154_buffer_free_raw(p_data);
    enter_report(&packetpool[n]);  // bug if n beyond pool size
    }

// currently unused (was for debugging)
void motestate(void * parameter, uint16_t size) {
    nrf_802154_state_t c;
    c = nrf_802154_state_get(); 
    NRF_LOG_INFO("state of 802154 = %d",c);
    }

void motesetup() {
    recently_active = false;
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

// doing a complete restart probably isn't needed, but maybe rejuvenation
// gets around any bugs in the BLE / 802.15.4 stack
void do_moterestart(void * parameter, uint16_t size) { moterestart(); }
static void mote_timeout_handler(void * p_context) {
    ret_code_t err_code;
    bsp_board_led_invert(1);
    memcpy((uint8_t*)&old_report,(uint8_t*)&prepare_report,sizeof(motepacket_t));
    report_init();  
    app_sched_event_put(NULL,1,&do_moterestart);
    err_code = app_timer_start(mote_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(transmit_timer_id,
               APP_TIMER_TICKS(random_write_delay(1000)), NULL);
    APP_ERROR_CHECK(err_code);
    }

/*
void nrf_802154_tx_started (const uint8_t *p_frame) {
    NRF_LOG_INFO("802154 tx started");
    NRF_LOG_FLUSH();
    }
    */
void nrf_802154_transmitted_raw(const uint8_t *p_frame,
                uint8_t *p_ack, int8_t power, uint8_t lqi) {
    nrf_802154_receive();
    }
void nrf_802154_transmit_failed (const uint8_t *p_frame, nrf_802154_tx_error_t error) {
    nrf_802154_receive();
    }
void mote_transmit(void * parameter, uint16_t size) { 
    mote_report_t * s = (mote_report_t*)old_report.data; 
    nrf_802154_transmit_raw((uint8_t*)&old_report,true);
    if (s->num_reports > 0) recently_active = true;
    nrfx_wdt_feed();
    }
static void transmit_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&mote_transmit);
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


void timers_init(void) {
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    // report generation & restart 
    err_code = app_timer_create(&mote_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                mote_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // report/probe transmission
    err_code = app_timer_create(&transmit_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                transmit_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // clock tick
    err_code = app_timer_create(&clock_id,
		                APP_TIMER_MODE_REPEATED,
                                clock_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // soft watchdog
    err_code = app_timer_create(&watchdog_id,
		                APP_TIMER_MODE_REPEATED,
                                watchdog_timeout_handler);
    APP_ERROR_CHECK(err_code);
    }

void timers_start(void) {
    ret_code_t err_code;
    err_code = app_timer_start(clock_id,APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    // kick off first cycle, report and restart 
    err_code = app_timer_start(mote_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(transmit_timer_id,
               APP_TIMER_TICKS(random_write_delay(1000)), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(watchdog_id,APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);
    }

uint32_t random_write_delay(uint32_t bound) {
    ret_code_t err_code;
    uint8_t num_rand_avail;
    uint32_t R;
    uint8_t * p = (uint8_t*)&randvalue;
    randvalue = 1 + (bound/2);  // poor, but default 
    err_code = sd_rand_application_bytes_available_get(&num_rand_avail);
    APP_ERROR_CHECK(err_code);
    if (num_rand_avail >= 4) {
       err_code = sd_rand_application_vector_get(p,4);
       APP_ERROR_CHECK(err_code);
       }
    R = randvalue % 32647;  // way too big, but a good start
    R = R % (bound - 100);  // force 100 millsec away from 
    if (R < 100) R = 100;   // boundary of the bound either way
    return R;
    }

void watchdog_handler(void) { };
void watchdog_init(void) {
    nrfx_wdt_channel_id m_channel = 0;
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
    uint32_t err_code = nrfx_wdt_init(&config,watchdog_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
    }

int main(void) {
    log_init();
    leds_init();
    pool_init();
    power_management_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    motesetup();
    report_init();   // first-time empty report, probably
    memcpy((uint8_t*)&old_report,(uint8_t*)&prepare_report,sizeof(motepacket_t));
    timers_init();
    timers_start();
    watchdog_init();

    // Enter main loop.
    for (;;) {
	app_sched_execute();
  	sd_app_evt_wait();
        // nrf_pwr_mgmt_run();
        }
    }


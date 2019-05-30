#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"

#include "bsp.h"
#include "nrf_clock.h"
#include "nrf_sdh.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "peer_manager.h"
#include "nrf_ble_scan.h"
#include "raal/nrf_raal_api.h"

#include "nrf_sdh_ble.h"

#include "nrf_802154.h"
#include "nrf_radio.h"
#include "nrf_802154_types.h"
#include "nrf_802154_clock.h"
#include "boards.h"
#include "IEEE802154.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#include "nrf_drv_clock.h"

#define MAX_MESSAGE_SIZE 37 
#define CHANNEL 26 
#define PACKET_MAX_PAYLOAD 26 

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

APP_TIMER_DEF(mote_timer_id);
APP_TIMER_DEF(clock_timer_id);

static uint32_t system_clock = 0;
static uint32_t last_receive_clock = 0;

uint8_t nnqslotstate = 0;
uint8_t nnqswi = 0;
uint32_t nnqswitype = 0;
uint8_t nnqreceive = 0;
uint8_t nnqreceived = 0;
uint8_t rxattempt = 0;
uint8_t seqno = 0;
uint8_t nnqvect[3];
uint32_t ticksremain = 0;
bool needinit = true;
uint8_t locbuffer[64]; 
uint32_t timebuff;
int8_t rssibuff;
uint8_t receivecount = 0;
bool restarting = false;

uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * packetptr = (motepacket_t *)buffer;

uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t short_address[]    = {0x06, 0x07};
uint8_t pan_id[]           = {0x04, 0x05};

void ble_stack_init(void) {
    uint32_t err_code;
    uint32_t ram_start = 0;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_ble_default_cfg_set(1, &ram_start);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    }
void ble_stack_deinit(void) {
    uint32_t err_code;
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);
    ble_db_discovery_close();
    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
    nrf_clock_event_clear(NRF_CLOCK_EVENT_HFCLKSTARTED);
    ble_stack_init();
    }

static void mote_timeout_handler(void * p_context);
static void clock_timeout_handler(void * p_context);
void motesetup() {
    ret_code_t err_code;
    if (needinit == false) return;
    NRF_LOG_INFO("motesetup start");
    ble_stack_init(); 
    nrf_drv_clock_init();
    if (false) ble_stack_deinit();
    nrf_802154_init();
    NRF_LOG_INFO("motesetup after init");
    if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);

    err_code = app_timer_create(&mote_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                mote_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&clock_timer_id,
		                APP_TIMER_MODE_REPEATED,
                                clock_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(100), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(clock_timer_id, 
		    APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    packetptr->fcf = 0x0000;
    packetptr->fcf &= 1 << IEEE154_FCF_ACK_REQ;
    packetptr->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    packetptr->data_seq_no = 0;
    packetptr->pan = 0x22;     // GROUP
    packetptr->dest = 0xFFFF;  // BCAST_ADDR
    packetptr->src = 99;       // ID of sender
    packetptr->iframe = 0x3f;  // 6LowPan designator 
    packetptr->type = 0x37;    // (bogus for now, could have meaning later)
    // -1 for length, +2 for unknown reason - seems to sort of work, 
    // perhaps because of the CRC bytes in the fcs?
    packetptr->length =  offsetof(motepacket_t,data) + 2 + PACKET_MAX_PAYLOAD - 1;
    needinit = false;
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
    NRF_LOG_INFO("** restart of stacks **");
    }

void motestate(void * parameter, uint16_t size) {
    nrf_802154_state_t c;
    c = nrf_802154_state_get(); 
    NRF_LOG_INFO("state of 802154 = %d",c);
    }

void motetx(void * parameter, uint16_t size) {
    bool b;
    uint8_t c;
    ret_code_t err_code;
    NRF_LOG_INFO("hfclk_is_ready = %d",
		    nrf_802154_clock_hfclk_is_running());
    if (needinit) {
       motesetup();
       err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(5000), NULL);
       APP_ERROR_CHECK(err_code);
       return;
       }
    c = nrf_802154_state_get(); 
    if (c != NRF_802154_STATE_SLEEP) {
       NRF_LOG_INFO("802154 state = %d clock = %d",c,system_clock);
       nrf_802154_sleep();
       nrf_802154_deinit();
       err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(5000), NULL);
       APP_ERROR_CHECK(err_code);
       return;
       }
    err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);
    packetptr->data_seq_no = seqno++;
    for (int i=0; i<PACKET_MAX_PAYLOAD; i++) packetptr->data[i] = seqno+i;
    b = nrf_802154_transmit_raw(buffer,true);
    c = nrf_802154_state_get(); 
    NRF_LOG_INFO("802154_transmit_raw attempted with result %d,%d",b,c);
    NRF_LOG_FLUSH();
    }

void moterx(void * parameter, uint16_t size) {
    uint8_t c;
    ret_code_t err_code;
    uint32_t delay = 20*1000;
    if (restarting) {
       err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(100), NULL);
       APP_ERROR_CHECK(err_code);
       return;
       }
    if (needinit) {
       motesetup();
       err_code = app_timer_start(mote_timer_id, 
		    APP_TIMER_TICKS(100), NULL);
       APP_ERROR_CHECK(err_code);
       return;
       }
    // ticksremain = nrf_raal_timeslot_us_left_get();
    NRF_LOG_INFO("hfclk_is_ready = %d rxattempt = %d",
                    nrf_802154_clock_hfclk_is_running(),rxattempt++);
    c = nrf_802154_state_get(); 
    NRF_LOG_INFO("802154 state = %d clock = %d",c,system_clock);
    /*
    NRF_LOG_INFO("nnqslotstate = %d", nnqslotstate);
    NRF_LOG_INFO("nnqreceive = %d nnqreceived = %d nnqswi = %d nnqswitype = %x",
		    nnqreceive,nnqreceived,nnqswi,nnqswitype);
    NRF_LOG_INFO("ticksremain = %d nnqvect = %d %d %d",
		    ticksremain,nnqvect[0],nnqvect[1],nnqvect[2]);
    NRF_LOG_FLUSH();
    */

    // speculative restart here
    // if (nnqslotstate == 0 && system_clock > 20) {
    if (false) {
       moterestart();	    
       c = nrf_802154_state_get(); 
       }

    if (c != NRF_802154_STATE_RECEIVE) {
       nrf_802154_receive();
       delay = 100;
       }
    err_code = app_timer_start(mote_timer_id, APP_TIMER_TICKS(delay), NULL);
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
    nrf_802154_sleep();
    }

void nrf_802154_transmit_failed (const uint8_t *p_frame, nrf_802154_tx_error_t error) {
    nrf_802154_sleep();
    }

void showbuff(void * parameter, uint16_t size) {
    NRF_LOG_INFO("Packet rssi = %d timestamp = %d",rssibuff,timebuff);
    NRF_LOG_HEXDUMP_INFO(locbuffer,40);
    NRF_LOG_FLUSH();
    }	
void nrf_802154_received_timestamp_raw(
    uint8_t * p_data, 
    int8_t power, 
    uint8_t lqi,
    uint32_t time) {
    memcpy(locbuffer,p_data,64);
    rssibuff = power;
    timebuff = time / 1000000L;
    last_receive_clock = system_clock;
    app_sched_event_put(NULL,1,&showbuff);
    nrf_802154_buffer_free_raw(p_data);
    for (int i=0; i<4; i++) bsp_board_led_invert(i);
}

void getcca(void * parameter, uint16_t size) {
    NRF_LOG_INFO("cca initiated");
    nrf_802154_cca();
    }	
void nrf_802154_cca_done(bool channel_free) {
    nrf_802154_sleep();
    app_sched_event_put(NULL,1,&moterx);
    }
void nrf_802154_cca_failed(nrf_802154_cca_error_t error) {
    nrf_802154_sleep();
    app_sched_event_put(NULL,1,&moterx);
    }

void restartask(void * parameter, uint16_t size) {
    moterestart();
    restarting = false;
    }

static void mote_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&moterx);
    }

static void clock_timeout_handler(void * p_context) {
    system_clock++;	
    bsp_board_led_invert(0);
    if ((system_clock - last_receive_clock) > 60) {
       last_receive_clock = system_clock;	    
       app_sched_event_put(NULL,1,&getcca);
       // restarting = true;
       // app_sched_event_put(NULL,1,&restartask);
       }
    }

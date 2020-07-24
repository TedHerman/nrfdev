#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "nrfx_wdt.h"
#include "nrf_drv_twi.h"

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "nrf_802154.h"
#include "IEEE802154.h"

#include "boards.h"
#include "bsp.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "motebadge.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

extern int16_t get_node_id();
extern int data_store(uint8_t* data, uint16_t data_len);
extern void clockapi_init();
extern uint32_t get_clock();
extern void message_clock_set(void * parameter, uint16_t size); 
extern uint8_t * clock_buffer;

// data for messages and recording mote information
uint8_t fence[] = {0x00,0x00,0x00,0x00,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55};
uint8_t motepool[66*1024];     // up to 64k, we have a bit of slop here 
volatile uint16_t motepool_size = 0; // offset to last byte recorded in motepool
uint32_t last_pool_init;       // time of last write to SD card 
uint8_t data_buffer[PACKET_MAX_PAYLOAD];  // for data transfer (extra byte for rssi)
uint8_t data_buffer_rssi;                 // rssi for above data
int16_t node_addr;
static volatile bool file_op_running;     // when SD card being active

// variables for packet generation
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

// code for messages
void clock_bcast(void);
void motesetup(void);

void nrf_802154_transmitted_raw(const uint8_t *p_frame,
                uint8_t *p_ack, int8_t power, uint8_t lqi) {
    nrf_802154_receive();  // reset mode of radio stack after transmit
    }
void nrf_802154_transmit_failed (const uint8_t *p_frame, nrf_802154_tx_error_t error) {
    nrf_802154_receive();
    }

void mote_send_clock(void * parameter, uint16_t size) {
  uint32_t t;
  bsp_board_led_invert(1);
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
  t = get_clock();
  memcpy(sync_message.data,(uint8_t*)&t,4);
  nrf_802154_transmit_raw((uint8_t*)&sync_message,true);
  }

void set_mote_receive(void * parameter, uint16_t size) { 
  nrf_802154_receive(); 
  }
void poolsetup() {
  memcpy(motepool,fence,sizeof(fence));
  memcpy(motepool+sizeof(fence),(uint8_t*)&node_addr,sizeof(node_addr));
  motepool_size = sizeof(node_addr) + sizeof(fence);
  last_pool_init = get_clock();
  }
void data_task(void * parameter, uint16_t size) {
  // do not save if there is nothing to save, or less than 6K because
  // we get timing problems with disk initialization on very small writes
  if (motepool_size <= sizeof(node_addr) + sizeof(fence)) {
    last_pool_init = get_clock();  // restart the init clock
    return; 
    }
  file_op_running = true;
  nrf_802154_sleep();
  nrf_802154_deinit();
  data_store(motepool,motepool_size);
  file_op_running = false;
  poolsetup();
  motesetup();
  }
void storepool(void * parameter, uint16_t size) {
  mote_report_t * p = (mote_report_t *)data_buffer; // skip over rssi for struct ptr
  uint8_t * q = motepool + motepool_size;   
  uint8_t v = offsetof(mote_report_t,reports);   // offset of variable portion of data_buffer
  uint8_t w = 4 + 1 + v; 
  // above is timestamp+rssi+fixed-part-of-mote_report 
  uint8_t x = w + sizeof(mote_received_t)*p->num_reports;
  // then x is total size, including variable part
  uint32_t t = get_clock();
  // timestamp and save message payload
  memcpy(q,(uint8_t*)&t,4); 
  *(q+4) = data_buffer_rssi; // rssi of this message
  memcpy(q+5,p,v);
  if (p->num_reports > 0) memcpy(q+w,p->reports,sizeof(mote_received_t)*p->num_reports);
  motepool_size += x;
  // NRF_LOG_INFO("%d message from mote %d time %d reports %d",
  //		  t,p->mote_id,p->time,p->num_reports);
  // NRF_LOG_HEXDUMP_INFO(q,x);
  // NRF_LOG_INFO("pool size is %d",motepool_size);
  // trigger to dump to SD card will be here, but should also test elapsed time
  if (motepool_size > 4096) {
    app_sched_event_put(NULL,1,&data_task);
    }
  }
void nrf_802154_received_timestamp_raw(uint8_t * p_data, int8_t power, 
		uint8_t lqi, uint32_t time) {
  uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t);
  motepacket_t * m = (motepacket_t *)p_data;
  UNUSED_PARAMETER(lqi);
  UNUSED_VARIABLE(size);  // may be tested in future revisions of this code
  bsp_board_led_invert(0);
  if (file_op_running) {
    nrf_802154_buffer_free_raw(p_data);
    return;
    }
  if (m->type == 0x36) {
    memcpy(clock_buffer,m->data,4);
    app_sched_event_put(NULL,1,&message_clock_set);
    }
  if (m->type == 0x37) { 
    data_buffer_rssi = (uint8_t)power;  // save rssi before payload
    memcpy(data_buffer,m->data,PACKET_MAX_PAYLOAD);
    app_sched_event_put(NULL,1,&storepool);
    }
  nrf_802154_buffer_free_raw(p_data);
    // temporarily put state of 802154 stack into sleep, then in just
    // microseconds later, renew the receive state -- this seems to trick
    // the stack into asking SDH for a new RAAL timeslot, so we continue
    // to receive (total continuous timeslot is 120 seconds)
  nrf_802154_sleep();
  app_sched_event_put(NULL,1,&set_mote_receive);
  }

void motesetup() {
    nrf_802154_init();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    uint8_t state = nrf_802154_state_get();
    NRF_LOG_INFO("Launch with 802.15.4 in state %d.",state)
    bsp_board_led_invert(1); bsp_board_led_invert(2); bsp_board_led_invert(3);
    }

/*
void nrf_802154_tx_started (const uint8_t *p_frame) {
    NRF_LOG_INFO("802154 tx started");
    NRF_LOG_FLUSH();
    }
    */

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

// handler called regularly by each clock tick
void auxiliary_tick_handler(uint32_t t) {  
  if (t % 60 == 3) app_sched_event_put(NULL,1,mote_send_clock);
  if (t % 60 == 5 && (t-last_pool_init > 20*60)) 
    app_sched_event_put(NULL,1,data_task);
  }

void timers_init(void) {
  ret_code_t err_code;
  // nrf_drv_clock_lfclk_request(NULL);
  // while(!nrf_drv_clock_lfclk_is_running()) { }
  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
  }

int main(void) {
  log_init();
  node_addr = get_node_id();
  file_op_running = false;
  if (false) {  // unclear if this will be compatible with USB and 802.15.4
       nrf_drv_clock_lfclk_request(NULL);
       while(!nrf_drv_clock_lfclk_is_running()) { }
       }
  timers_init();
  leds_init();
  power_management_init();
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
  clockapi_init();
  poolsetup();
  motesetup();

  // Enter main loop.
  for (;;) {
    NRF_LOG_FLUSH();
    //  while (app_usbd_event_queue_process()) { }
    app_sched_execute();
    nrf_pwr_mgmt_run();
    }
  }


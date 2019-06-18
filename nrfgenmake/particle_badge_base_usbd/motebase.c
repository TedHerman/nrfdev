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

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf_802154.h"
#include "IEEE802154.h"

#include "boards.h"
#include "bsp.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define MAX_MESSAGE_SIZE 37 
#define CHANNEL 26 
#define PACKET_MAX_PAYLOAD 26 
#define PACKET_POOL_SIZE 256

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

char m_rx_buffer[8];
uint8_t m_rx_buffer_index = 0;
uint8_t m_clock_buffer[8];

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
void get_clock(void * parameter, uint16_t size);

void read_usb(void * parameter, uint16_t size) {
    m_rx_buffer_index = 0;
    app_usbd_cdc_acm_read(&m_app_cdc_acm,
        m_rx_buffer,
        1);
    m_rx_buffer_index++;
    }

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event) {
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
            /*Setup first transfer*/

            // NRF_LOG_INFO("port open");
            m_rx_buffer_index = 0;
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   1);
            m_rx_buffer_index++;
            UNUSED_VARIABLE(ret);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            // NRF_LOG_INFO("port close");
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
	    uint8_t zeros[] = {0x0,0x0,0x0,0x0}; // fence for clock
	    size_t size = app_usbd_cdc_acm_bytes_stored(p_cdc_acm);
	    if (size == 0) {
               // NRF_LOG_INFO("user evt rx with no bytes stored");
               app_usbd_cdc_acm_read(&m_app_cdc_acm,
                        &m_rx_buffer[m_rx_buffer_index],
                        1);
	       break;
	       }
            app_usbd_cdc_acm_read(&m_app_cdc_acm,
                        &m_rx_buffer[m_rx_buffer_index],
                        size);
            size_t size2 = app_usbd_cdc_acm_rx_size(p_cdc_acm);
            // NRF_LOG_INFO("read size = %d, read = %x",
	    //	    size2,m_rx_buffer[m_rx_buffer_index]);
	    if (m_rx_buffer_index <= 3 &&
	        memcmp(m_rx_buffer,zeros,m_rx_buffer_index+1) != 0) {
		m_rx_buffer_index = 0;
                app_usbd_cdc_acm_read(&m_app_cdc_acm,
                        &m_rx_buffer[m_rx_buffer_index],
                        1);
		break;
	        }
	    m_rx_buffer_index += size2;
	    // NRF_LOG_INFO("buffer so far has %d bytes",m_rx_buffer_index); 
	    // NRF_LOG_RAW_HEXDUMP_INFO(m_rx_buffer,m_rx_buffer_index);
	    if (m_rx_buffer_index >= 8 &&
	        memcmp(m_rx_buffer,zeros,4) == 0) {
		// NRF_LOG_INFO("Clock value");
		// WE HAVE A CLOCK VALUE
                memcpy(m_clock_buffer,m_rx_buffer,8);
                app_sched_event_put(NULL,1,&get_clock);
		m_rx_buffer_index = 0;
	        }
            break;
            }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    // NRF_LOG_INFO("* user ev handler");
    // NRF_LOG_FLUSH();
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            // NRF_LOG_INFO("* evt_drv_suspend");
            // NRF_LOG_FLUSH();
            break;
        case APP_USBD_EVT_DRV_RESUME:
            // NRF_LOG_INFO("* evt_drv_resume");
            // NRF_LOG_FLUSH();
            break;
        case APP_USBD_EVT_STARTED:
            // NRF_LOG_INFO("* evt_drv_started");
            // NRF_LOG_FLUSH();
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            // NRF_LOG_INFO("* USB power detected");
            if (!nrf_drv_usbd_is_enabled()) {
              app_usbd_enable();
              }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // NRF_LOG_INFO("* USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            // NRF_LOG_INFO("* USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

void hdlc_send(uint8_t * p, uint8_t size) {
    ret_code_t err_code;
    char outbuff[2*NRF_DRV_USBD_EPSIZE]; // 2 * sixty-four bytes
    uint8_t cursor = 0;
    UNUSED_VARIABLE(err_code);
    // ref: https://en.wikipedia.org/wiki/High-Level_Data_Link_Control
    if (size == 0) return;   
    outbuff[cursor++] = 0x7e;
    while (size > 0) {
      if (*p == 0x7e) { 
         outbuff[cursor++] = 0x7d;
	 outbuff[cursor++] = 0x5e;
         }
      else if (*p == 0x7d) { 
         outbuff[cursor++] = 0x7d;
	 outbuff[cursor++] = 0x5d;
         }
      else 
         outbuff[cursor++] = *p; 
      p++;
      size--;
      if (cursor > sizeof(outbuff)-2) return;  
      }
    outbuff[cursor++] = 0x7e;
    if (cursor <= NRF_DRV_USBD_EPSIZE) {
       err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm,outbuff,cursor);
       return;
       }
    err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm,outbuff,NRF_DRV_USBD_EPSIZE);
    err_code = app_usbd_cdc_acm_write(&m_app_cdc_acm,outbuff+NRF_DRV_USBD_EPSIZE,
		       cursor-NRF_DRV_USBD_EPSIZE);
    }

void get_clock(void * parameter, uint16_t size) {
    uint8_t buffer[4];
    uint32_t newclock; 
    uint32_t i;
    // we expect first four zeroes (not a valid clock)
    for (i=0; i<4; i++) if (m_clock_buffer[i] != 0) return; 
    // then four clock digits 
    for (i=0; i<4; i++) buffer[i] = m_clock_buffer[4+i]; 
    // convert to host mode  
    newclock = 0;
    for (i=0; i<4; i++) {
       newclock *= 256;
       newclock += buffer[i];
       }
    clock = newclock;
    // NRF_LOG_INFO("got clock %d",clock);
    clock_bcast();
    }

void nrf_802154_transmitted_raw(const uint8_t *p_frame,
                uint8_t *p_ack, int8_t power, uint8_t lqi) {
    nrf_802154_receive();
    app_sched_event_put(NULL,1,&read_usb);
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
    }

void motedump(void * parameter, uint16_t size) {
    recpacket_t * k;
    // motepacket_t * m;
    // mote_report_t * p;
    // mote_received_t * q;
    for (int n=0; n<PACKET_POOL_SIZE; n++) {
       if (!packetpool[n].occupied) continue;
       k = &packetpool[n];
       // m = &k->packet;
       // p = (mote_report_t *)m->data;
       hdlc_send((uint8_t*)k,sizeof(recpacket_t));

       /*
       // NRF_LOG_INFO("Packet count %d time %d rssi %d",
		    // packet_count, packetpool[n].time, packetpool[n].rssi);
       // NRF_LOG_INFO("mote %d (time %d)",p->mote_id,p->time);
       for (int j=0; j<p->num_reports; j++) {
	   q = &p->reports[j];
	   //  NRF_LOG_INFO("  heard other mote %d (rssi %d)",
	   //    q->mote_id, q->rssi);
           }
       */
       packetpool[n].occupied = false;
       packet_count++;
       }
    // NRF_LOG_FLUSH();
    }

void set_mote_receive(void * parameter, uint16_t size) { 
    bsp_board_led_invert(0);
    nrf_802154_receive(); 
    }
void nrf_802154_received_timestamp_raw(uint8_t * p_data, int8_t power, uint8_t lqi, uint32_t time) {
    motepacket_t * m = (motepacket_t *) p_data;
    uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t);
    uint32_t n;
    UNUSED_PARAMETER(lqi);
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
    }

void motesetup() {
    nrf_802154_init();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    // uint8_t state = nrf_802154_state_get();
    // NRF_LOG_INFO("Launch with 802.15.4 in state %d.",state)
    // NRF_LOG_FLUSH();
    }

/*
void nrf_802154_tx_started (const uint8_t *p_frame) {
    NRF_LOG_INFO("802154 tx started");
    NRF_LOG_FLUSH();
    }
    */

static void mote_timeout_handler(void * p_context) {
    // app_sched_event_put(NULL,1,&motetx);
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
    // nrf_drv_clock_lfclk_request(NULL);
    // while(!nrf_drv_clock_lfclk_is_running()) { }
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    }

void usb_init(void) {
    ret_code_t err_code;
    static app_usbd_config_t const usbd_config = {
       .ev_state_proc = usbd_user_ev_handler
       };
    app_usbd_serial_num_generate();
    err_code = nrf_drv_clock_init();  // but earlier there is nrf_drv_clock_request??
    APP_ERROR_CHECK(err_code);
    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);
    app_usbd_class_inst_t const * class_cdc_acm = 
       app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    err_code = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(err_code);
    }
int main(void) {
    ret_code_t err_code;
    log_init();
    if (false) {  // unclear if this will be compatible with USB and 802.15.4
       nrf_drv_clock_lfclk_request(NULL);
       while(!nrf_drv_clock_lfclk_is_running()) { }
       }
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    timers_init();
    leds_init();
    pool_init();
    power_management_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    usb_init();
    err_code = app_usbd_power_events_enable();
    APP_ERROR_CHECK(err_code);
    // NRF_LOG_INFO("after usb init");
    // NRF_LOG_FLUSH();
    motesetup();

    err_code = app_timer_create(&mote_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                mote_timeout_handler);
    APP_ERROR_CHECK(err_code);

    // Enter main loop.
    for (;;) {
      // NRF_LOG_FLUSH();
      while (app_usbd_event_queue_process()) { }
      app_sched_execute();
      nrf_pwr_mgmt_run();
      }
    }

/* ??? what does this do I wonder 
    ret = app_usbd_cdc_acm_serial_state_notify(&m_app_cdc_acm,
              APP_USBD_CDC_ACM_SERIAL_STATE_BREAK,
              false);
    UNUSED_VARIABLE(ret);
    */


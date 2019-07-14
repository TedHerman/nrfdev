#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_assert.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "bsp.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "motebadge.h"


extern void neopixel_write(uint8_t* rgb);
extern void mem_init();
extern uint32_t mem_cursor();
extern void mem_rewind();
extern void mem_undo();
extern uint8_t* mem_fetch();
extern uint8_t* mem_access(uint16_t index);
extern uint8_t* mem_search(uint16_t index);
extern uint16_t mem_check();
extern void hdlc_callback();
extern void hdlc_init();
extern void hdlc_stop();
extern bool hdlc_send(uint8_t* message, uint8_t length);
extern uint8_t* hdlc_push(uint8_t c);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

// need forward ref for data structure macro
static void cdc_acm_user_ev_handler(
  app_usbd_class_inst_t const * p_inst,
  app_usbd_cdc_acm_user_event_t event);
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
  cdc_acm_user_ev_handler,
  CDC_ACM_COMM_INTERFACE,
  CDC_ACM_DATA_INTERFACE,
  CDC_ACM_COMM_EPIN,
  CDC_ACM_DATA_EPIN,
  CDC_ACM_DATA_EPOUT,
  APP_USBD_CDC_COMM_PROTOCOL_AT_V250
  );
const app_usbd_cdc_acm_t * remote_ref_cdc_acm = &m_app_cdc_acm;

APP_TIMER_DEF(usbd_clock_id);
uint32_t clock;          // clock incremented every second during xfer session
uint32_t activity_time;  // last observed activity time

uint8_t rx_buff[16];     // single buffer
bool rx_discard;         // flag because why? strange nordic api
bool command_active;     // to prevent overflow on command processing
bool xfer_active;        // to wait until transfer complete
bool streaming;          // for bulk transfer, make this true, else false
uint8_t sync_message[] = {'K','O','K'};
void usbd_cmd_process(void * parameter, uint16_t size);
void usbd_cmd_reset(void * parameter, uint16_t size); 

// we run a clock to know when it's time to tell user 
// that the situation is one of idleness
void usbd_clock_handler(void * p_context) {
  clock++;
  if (clock - activity_time > 15) {
    app_sched_event_put(NULL,1,&usbd_cmd_reset);
    return;
    }
  }

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event) {
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    switch (event) {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
	    bsp_board_led_off(0);
            NRF_LOG_INFO("port open");
	    NRF_LOG_FLUSH();
	    app_usbd_cdc_acm_write(p_cdc_acm,&sync_message,3);
            break;
            }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
	    bsp_board_led_on(0);
            NRF_LOG_INFO("port close");
	    NRF_LOG_FLUSH();
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
	    activity_time = clock;
            hdlc_callback();
            app_usbd_cdc_acm_read(p_cdc_acm,rx_buff,1);  
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
            size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm); // actually read from prev read
            size_t stored = app_usbd_cdc_acm_bytes_stored(p_cdc_acm); // available
	    uint8_t* command = NULL;

	    if (!rx_discard) {
	       // NRF_LOG_INFO("RX_DONE size %d stored %d rxbuff %d",size,stored,rx_buff[0]);
	       command = hdlc_push(rx_buff[0]);
	       }
	    rx_discard = false;
            if (stored > 0 && stored < 16) {  
	       app_usbd_cdc_acm_read_any(p_cdc_acm,rx_buff,stored);
	       rx_discard = true;  // avoid double-read
               size = app_usbd_cdc_acm_rx_size(p_cdc_acm); // actually read from prev read
               stored = app_usbd_cdc_acm_bytes_stored(p_cdc_acm); // available
	       for (int i=0; i<size; i++) command = hdlc_push(rx_buff[i]);
	       } 
	    app_usbd_cdc_acm_read(p_cdc_acm,rx_buff,1);  

	    if (command == NULL) break;
	    // NOTE in current imlementation command size is known 
	    // as a convention of the protocol (check first byte, etc)
	    if (!command_active) {
	      activity_time = clock;
	      command_active = true;
	      app_sched_event_put(&command,4,&usbd_cmd_process);
	      }
	    break;
	 default: break;
         }
    }
 }

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
    switch (event) {
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
            NRF_LOG_INFO("* USB power detected %d",nrf_drv_usbd_is_enabled());
            if (!nrf_drv_usbd_is_enabled()) { app_usbd_enable(); }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            // NRF_LOG_INFO("* USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("* USB ready");
            app_usbd_start();
            break;
        default:
            break;
        }
    }

// fetch and transfer first/next block
void usbd_stream(void * parameter, uint16_t size) {
  uint8_t* block;
  if (!streaming) return;
  // request for "next block" in dump of transfered memory
  block = mem_fetch();
  if (block != NULL && xfer_active) {
     // NOTE possible danger, if hdlc_send failed (busy), not checked here
     hdlc_send(block,DATA_LENGTH_MAX);
     return; // callback will tell when the send finished
     }
  else {
     streaming = false;
     hdlc_send(sync_message,sizeof(sync_message)); 
     }
  }

// callback after hdlc_send finishes
void hdlc_send_callback() {
  if (streaming) app_sched_event_put(NULL,1,&usbd_stream); // could use timer here?
  }

// called when input from USB device was recognized as
// hdlc and decoded, and this means work to do 
void usbd_cmd_process(void * parameter, uint16_t size) { 
  uint8_t* command = *(uint8_t**)parameter; 
  uint16_t blocknum;
  uint8_t* block;
  // NRF_LOG_HEXDUMP_INFO(command,8);
  // NRF_LOG_FLUSH();
  switch (command[0]) {
    case 'K':
      // synchronization command, so reply in kind
      hdlc_send(sync_message,sizeof(sync_message));  
      break;
    case '>': 
      streaming = true;
      app_sched_event_put(NULL,1,&usbd_stream);
      break;
    case '<':
      // request to rewind memory, for another scan (maybe)
      mem_rewind();
      hdlc_send(sync_message,sizeof(sync_message)); 
      break;
    case '*':
      // request to clear memory for complete restart
      hdlc_stop();
      mem_init();
      hdlc_init();
      break;
    case '?':
      // fetch block request
      blocknum = *(uint16_t*)&command[1];     
      block = mem_search(blocknum);
      NRF_LOG_INFO("fetch %d got %d",blocknum,block);
      NRF_LOG_HEXDUMP_INFO(block,8);
      NRF_LOG_FLUSH();
      if (block != NULL && xfer_active) hdlc_send(block,DATA_LENGTH_MAX);
      else hdlc_send(sync_message,sizeof(sync_message)); 
      break;
    default:
      break;
    }
  command_active = false;
  }	

void usbd_start() {
    // reference code shows this being called after advertising_start() 
    ret_code_t err_code;
    err_code = app_usbd_power_events_enable();
    APP_ERROR_CHECK(err_code);
    hdlc_init();
    mem_init();
    }

void usbd_init(void) {
    // reference code in examples/peripheral/usbd_ble_uart show this 
    // code being executed *before* enabling ble, or sdh_enable
    ret_code_t err_code;
    static app_usbd_config_t const usbd_config = {
       .ev_state_proc = usbd_user_ev_handler
       };
    app_usbd_class_inst_t const * class_cdc_acm = 
       app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    app_usbd_serial_num_generate();
    err_code = nrf_drv_clock_init(); 
    err_code = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(err_code);
    err_code = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&usbd_clock_id,
              APP_TIMER_MODE_REPEATED,
              usbd_clock_handler);
    APP_ERROR_CHECK(err_code);
    }

// called externally, integrated into main loop
bool usbd_process() {
    return app_usbd_event_queue_process();
    }

// scheduled when clock on activity lapses
void usbd_cmd_reset(void * parameter, uint16_t size) {
    uint8_t rgb[3] = {0x33,0x55,0x00};
    command_active = false;
    streaming = false;
    app_timer_stop(usbd_clock_id);
    hdlc_stop();
    mem_init();
    neopixel_write(rgb);
    }

void usbd_xfer() {
    ret_code_t err_code;
    uint8_t rgb[] = {0xff,0x00,0x33};
    neopixel_write(rgb);
    mem_rewind();
    clock = 0;
    command_active = false;
    xfer_active = true;
    streaming = false;
    err_code = app_timer_start(usbd_clock_id,APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    }

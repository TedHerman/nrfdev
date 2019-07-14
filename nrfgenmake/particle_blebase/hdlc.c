#include "nrf_drv_usbd.h"
#include "nrf_assert.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define HDLC_INDATA_MAX 16

extern const app_usbd_cdc_acm_t * remote_ref_cdc_acm;
extern void hdlc_send_callback(void);
void hdlc_loop(void * parameter, uint16_t size);
uint8_t hdlc_decode(uint8_t* frame);
uint8_t decode_buffer[HDLC_INDATA_MAX];
uint8_t outbuff[8*NRF_DRV_USBD_EPSIZE]; // 8 * sixty-four bytes = 512 bytes
uint8_t rx_pipe[2*HDLC_INDATA_MAX];  // pipe for raw input storing bytes
uint8_t rx_cursor;
uint8_t rx_state;
uint32_t outbuff_cursor;
uint32_t outbuff_reader;

bool callback_expected;
bool hdlc_send_busy;

APP_TIMER_DEF(hdlc_id);

// convert binary into hdlc, with output placed in outbuff
// note restriction, maybe at most 255 bytes per hdcl_send
// (output is 512 bytes, so actual input max unknown)
// ref: https://en.wikipedia.org/wiki/High-Level_Data_Link_Control
void hdlc_prep(uint8_t * p, uint8_t size) { 
  // NRF_LOG_INFO("hdlc_prep input");
  // NRF_LOG_HEXDUMP_INFO(p,8);
  // NRF_LOG_FLUSH();
  outbuff_cursor = 0;
  if (size == 0) return;   
  outbuff[outbuff_cursor++] = 0x7e;
  while (size > 0) {
    if (*p == 0x7e) { 
      outbuff[outbuff_cursor++] = 0x7d;
      outbuff[outbuff_cursor++] = 0x5e;
      p++;
      size--;
      }
    else if (*p == 0x7d) { 
      outbuff[outbuff_cursor++] = 0x7d;
      outbuff[outbuff_cursor++] = 0x5d;
      p++;
      size--;
      }
    else { 
      outbuff[outbuff_cursor++] = *p; 
      p++;
      size--;
      if (outbuff_cursor > sizeof(outbuff)-2) return;  
      }
    }
  outbuff[outbuff_cursor++] = 0x7e;
  outbuff_reader = 0;
  // should be all set for hdlc_loop
  // NRF_LOG_INFO("hdlc_prep output, size %d",outbuff_cursor);
  // NRF_LOG_HEXDUMP_INFO(outbuff,8);
  // NRF_LOG_HEXDUMP_INFO(&outbuff[outbuff_cursor-8],8);
  // NRF_LOG_FLUSH();
  }

// note: hdlc_decode is for short inputs only 
// (output has to fit in HDLC_INDATA_MAX bytes) 
// return value is size of output byte string
// with zero for invalid or overflowing input
uint8_t hdlc_decode(uint8_t* frame) {
  // output will be in decode_buffer, but return 0 if there's a problem
  // normally the length is the return value
  uint8_t* outptr = decode_buffer;
  uint8_t* outlimit = decode_buffer + HDLC_INDATA_MAX;
  uint8_t* cursor = frame;
  uint8_t state = 0;
  // NRF_LOG_INFO("decoding");
  // NRF_LOG_HEXDUMP_INFO(frame,8);
  while (outptr < outlimit) {
    switch (state) {
      case 0: 
	if (*cursor != 0x7e) return 0;
	cursor++;
	state = 1;
	break;
      case 1: 
	if (*cursor == 0x7e) { 
           // NRF_LOG_INFO("decoded");
           // NRF_LOG_HEXDUMP_INFO(decode_buffer,8);
	   return (outptr-decode_buffer);
	   } 
	if (*cursor != 0x7d) {
	   *outptr++ = *cursor++;
	   break;
	   }
	// 0x7d is an escape char, skip it
	cursor++;
	state = 2;
	break;
      case 2:
	if (*cursor == 0x5e) {
	   *outptr++ = 0x7e;
	   cursor++;
	   state = 1;
	   break;
	   }
	if (*cursor == 0x5d) {
	   *outptr++ = 0x7d;
	   cursor++;
	   state = 1;
	   break;
	   }
      default:
	return 0;
        }
     }
  return 0;
  }

// loop based on outbuff and outbuff_cursor, using 
// timer to pause after cdc_acm_write, for a callback/signal
// see "callback_expected" for mediation between TX event
// handler and invoking this hdlc_loop 
void hdlc_loop(void * parameter, uint16_t size) { 
  ret_code_t err_code;
  if (outbuff_cursor - outbuff_reader >= NRF_DRV_USBD_EPSIZE) {
    // NRF_LOG_INFO("full cdc_acm_write");
    // NRF_LOG_HEXDUMP_INFO(&outbuff[outbuff_reader],8);
    // NRF_LOG_FLUSH();
    err_code = app_usbd_cdc_acm_write(remote_ref_cdc_acm,
		       &outbuff[outbuff_reader],
		       NRF_DRV_USBD_EPSIZE);
    if (err_code == NRF_ERROR_BUSY) {
       err_code = app_timer_start(hdlc_id,APP_TIMER_TICKS(10), NULL);
       APP_ERROR_CHECK(err_code);
       return;
       }
    APP_ERROR_CHECK(err_code);
    callback_expected = true;  // callback to continue loop
    outbuff_reader += NRF_DRV_USBD_EPSIZE;
    return;
    }
  if (outbuff_cursor - outbuff_reader > 0) {
    // NRF_LOG_INFO("partial cdc_acm_write %d",outbuff_cursor-outbuff_reader);
    // NRF_LOG_HEXDUMP_INFO(&outbuff[outbuff_reader],8);
    // NRF_LOG_FLUSH();
    err_code = app_usbd_cdc_acm_write(remote_ref_cdc_acm,
		       &outbuff[outbuff_reader],
		       outbuff_cursor - outbuff_reader);
    if (err_code == NRF_ERROR_BUSY) {
       err_code = app_timer_start(hdlc_id,APP_TIMER_TICKS(10), NULL);
       APP_ERROR_CHECK(err_code);
       }
    APP_ERROR_CHECK(err_code);
    outbuff_reader = outbuff_cursor;
    callback_expected = true;  // callback to continue loop
    return;
    }
  // Here, all the sending is finished for this block
  hdlc_send_callback();  // notify of completion
  hdlc_send_busy = false;
  }

// called externally from cdc_acm event handler 
void hdlc_callback() {
  if (!callback_expected) return;
  callback_expected = false;
  app_sched_event_put(NULL,1,&hdlc_loop);
  }

// called by timer expiration
void hdlc_timeout_handler(void * p_context) {
  app_sched_event_put(NULL,1,&hdlc_loop);
  }

// invoke here to send a message hdlc-encoded, which 
// can fail if the output buffer is busy; hence return
// true/false depending on whether the send started
bool hdlc_send(uint8_t* message, uint8_t length) {
  if (hdlc_send_busy) return false;
  hdlc_prep(message,length);
  hdlc_send_busy = true;
  app_sched_event_put(NULL,1,&hdlc_loop);
  return true;
  }

// input processing state machine: calculates a decoded
// version of input, which is fed asynchronously one byte
// at a time, continously attempting hdlc recognition 
uint8_t* hdlc_push(uint8_t c) {
  // called to consume c, perhaps return decoded accumulation
  uint8_t r;
  if (rx_cursor > sizeof(rx_pipe)-1) rx_cursor = 0;  // crazy but safe
  switch (rx_state) {
    case 0:
      if (c != 0x7e) return NULL;  // just consume stray chars
      rx_pipe[0] = c;
      rx_cursor = 1;
      rx_state = 1;
      break;
    case 1:
      if (c != 0x7e) {
         rx_pipe[rx_cursor++] = c;
         break;
         }
      // mini resynchronization here
      if (c == 0x7e && rx_cursor > 0 && rx_pipe[rx_cursor-1]==c) {
         rx_cursor = 0;
         rx_pipe[rx_cursor++] = c;
         break;
         }
      if (c == 0x7e && rx_cursor > 1) {
         rx_pipe[rx_cursor] = c;
         r = hdlc_decode(rx_pipe);
         rx_state = 0;
         rx_cursor = 0;
         if (r > 0) return decode_buffer;
         else return NULL;
         }
    default:
      break;
    }
  return NULL;
  }

void hdlc_init() {
  ret_code_t err_code;
  err_code = app_timer_create(&hdlc_id,
              APP_TIMER_MODE_SINGLE_SHOT,
              hdlc_timeout_handler);
  APP_ERROR_CHECK(err_code);
  callback_expected = false;
  hdlc_send_busy = false;
  outbuff_cursor = 0;
  outbuff_reader = 0;
  rx_cursor = 0;
  rx_state = 0;
  }

void hdlc_stop() {
  app_timer_stop(hdlc_id);
  }

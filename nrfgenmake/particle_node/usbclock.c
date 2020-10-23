#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern void clock_set(uint32_t newclock);

void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
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

uint32_t clock_via_usb = 0;

void read_usb(void * parameter, uint16_t size) {
  m_rx_buffer_index = 0;
  app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, 1);
  m_rx_buffer_index++;
  }

void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event) {
  app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
  switch (event) {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
      /*Setup first transfer*/

      // NRF_LOG_INFO("port open");
      m_rx_buffer_index = 0;
      ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, 1);
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
        app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_rx_buffer[m_rx_buffer_index], 1);
        break;
        }
      app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_rx_buffer[m_rx_buffer_index], size);
      size_t size2 = app_usbd_cdc_acm_rx_size(p_cdc_acm);
      // NRF_LOG_INFO("read size = %d, read = %x",
      //      size2,m_rx_buffer[m_rx_buffer_index]);
      if (m_rx_buffer_index <= 3 &&
                memcmp(m_rx_buffer,zeros,m_rx_buffer_index+1) != 0) {
        m_rx_buffer_index = 0;
        app_usbd_cdc_acm_read(&m_app_cdc_acm, &m_rx_buffer[m_rx_buffer_index], 1);
        break;
        }
      m_rx_buffer_index += size2;
      // NRF_LOG_INFO("buffer so far has %d bytes",m_rx_buffer_index); 
      // NRF_LOG_RAW_HEXDUMP_INFO(m_rx_buffer,m_rx_buffer_index);
      if (m_rx_buffer_index >= 8 &&
                memcmp(m_rx_buffer,zeros,4) == 0) {
        uint32_t v,w;
	w = m_rx_buffer[4];
	v = w << 24;
	w = m_rx_buffer[5];
        v |= (w << 16);
	w = m_rx_buffer[6];
        v |= (w << 8);
	w = m_rx_buffer[7];
        v |= w;	
	// NRF_LOG_INFO("Clock proposed %d",v);
	if (v < 1890000000 && v > 1570000000) {
	   clock_via_usb = v;  // save for external access
	   clock_set(clock_via_usb);
	   NRF_LOG_INFO("Clock set %d",v);
	   NRF_LOG_FLUSH();
	   NVIC_SystemReset();
	   }
        m_rx_buffer_index = 0;
        }
      break;
      }
    default:
      break;
    }
  }

static void usbd_user_ev_handler(app_usbd_event_type_t event) {
  // NRF_LOG_INFO("* user ev handler");
  // NRF_LOG_FLUSH();
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
    else outbuff[cursor++] = *p;
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


void usb_init(void) {
  ret_code_t err_code;
  static app_usbd_config_t const usbd_config = {
    .ev_state_proc = usbd_user_ev_handler
    };
  app_usbd_serial_num_generate();
  err_code = nrf_drv_clock_init();
     // but earlier there is nrf_drv_clock_request??
  APP_ERROR_CHECK(err_code);
  err_code = app_usbd_init(&usbd_config);
  APP_ERROR_CHECK(err_code);
  app_usbd_class_inst_t const * class_cdc_acm =
  app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
  err_code = app_usbd_class_append(class_cdc_acm);
  APP_ERROR_CHECK(err_code);
  clock_via_usb = 0;   
  }


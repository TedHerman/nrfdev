#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_atomic.h"
#include "nrf_drv_twi.h"

#include "boards.h"
#include "bsp.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "boron.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

#define BORON_ADDR 0x04 

extern bool i2c_setaddr(uint8_t addr);
extern ret_code_t i2c_rx(uint8_t* buffer, uint8_t size);
extern ret_code_t i2c_tx(uint8_t* buffer, uint8_t size, bool cont);
extern void boronapi_read_done( boronstate_t * p );
extern void boronapi_write_done( bool worked );

// global variables of this module
uint8_t i2c_buffer[64];

void boron_read(void * parameter, uint16_t size) {
  ret_code_t err_code;
  // NRF_LOG_INFO("trying to set address to boron");
  // NRF_LOG_FLUSH();
  if (!i2c_setaddr(BORON_ADDR)) {
    boronapi_read_done(NULL);
    return;
    }
  memset(i2c_buffer,0,sizeof(i2c_buffer));
  err_code = i2c_rx(i2c_buffer, sizeof(boronstate_t));
  if (err_code != NRF_SUCCESS) { 
    boronapi_read_done(NULL);
    return;
    }
  boronapi_read_done((boronstate_t *)i2c_buffer);
  } 

void boron_write(void * parameter, uint16_t size) { 
  ret_code_t err_code;
  if (!i2c_setaddr(BORON_ADDR)) {
    NRF_LOG_INFO("boron not found");
    boronapi_write_done(false);
    return;
    }
  err_code = i2c_tx(i2c_buffer,sizeof(bps_reading_t),false);
  if (err_code != NRF_SUCCESS) {
    boronapi_write_done(false);
    return;
    }
  boronapi_write_done(true);
  }

void boronapi_write(bps_reading_t * p) {
  memcpy(i2c_buffer,(uint8_t*)p,sizeof(bps_reading_t));
  NRF_LOG_INFO("boronapi write called") 
  app_sched_event_put(NULL,1,&boron_write); 
  }

// initiates read, completed by callback
void boronapi_read() {
  app_sched_event_put(NULL,1,&boron_read); 
  }

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

#define TWI_INSTANCE_ID 1

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
typedef void (*twi_handler_t)(nrf_drv_twi_evt_t const * p_event, void * p_context);
twi_handler_t auxhandler; 
uint8_t twi_address;

/*
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context) {
  NRF_LOG_INFO("twi event with handler %p",auxhandler);
  if (auxhandler != NULL) (*auxhandler)(p_event,p_context);
  }
*/

void i2c_init () {
  ret_code_t err_code;
  auxhandler = NULL;
  twi_address = 0;

  const nrf_drv_twi_config_t twi_boron_config = {
    .scl                = ARDUINO_SCL_PIN,
    .sda                = ARDUINO_SDA_PIN,
    .frequency          = NRF_DRV_TWI_FREQ_100K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false 
    };

  // err_code = nrf_drv_twi_init(&m_twi, &twi_boron_config, twi_handler, NULL);
  err_code = nrf_drv_twi_init(&m_twi, &twi_boron_config, NULL, NULL);  // blocking mode
  APP_ERROR_CHECK(err_code);
  nrf_drv_twi_enable(&m_twi);
  }

/*
bool i2c_sethandler(uint8_t addr, twi_handler_t f) {
  if (f == NULL) {
    // implicit request to unset handler
    auxhandler = NULL;
    twi_address = 0;
    return false;  // does not matter for the NULL case
    }
  auxhandler = f;  // optimistic setting (could be wrong)
  twi_address = addr;
  err_code = nrf_drv_twi_rx(&m_twi, addr, buffer, sizeof(buffer));
  if (err_code != NRF_SUCCESS) {
    auxhandler = NULL;
    twi_address = 0
    return false;  // false means address not found
    }
  NRF_LOG_INFO("i2c sethandler %p",auxhandler);
  return true;
  }
  */

uint8_t i2c_getaddr() { return twi_address; }
bool i2c_setaddr(uint8_t addr) {
  ret_code_t err_code;
  uint8_t buffer[sizeof(boronstate_t)];
  if (twi_address == addr) return true;  // presumably this is OK
  twi_address = addr;
  err_code = nrf_drv_twi_rx(&m_twi, addr, buffer, sizeof(buffer));
  if (err_code != NRF_SUCCESS) {  // any kind of error means TWI not working
    twi_address = 0;
    return false;  // false means address not found
    }
  return true;
  }
bool i2c_isbusy() {
  return nrf_drv_twi_is_busy(&m_twi);
  }

ret_code_t i2c_rx(uint8_t* buffer, uint8_t size) {
  // NRF_LOG_INFO("attempt rx on addr %d",twi_address);
  return nrf_drv_twi_rx(&m_twi,twi_address,buffer,size); 
  }	

ret_code_t i2c_tx(uint8_t* buffer, uint8_t size, bool cont) {
  // NRF_LOG_INFO("attempt tx on addr %d",twi_address);
  return nrf_drv_twi_tx(&m_twi,twi_address,buffer,size,cont);
  }


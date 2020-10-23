#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

typedef struct {
  uint8_t addr[6]; 
  } master_addr_t;

master_addr_t master_addr_list[] = 
  { { {0xff,0xca,0xb5,0xf5,0xc6,0x81} },   // mote 1024 
    { {0xd3,0x9b,0x09,0xe1,0x6e,0x11} },
      };

int16_t get_node_id() {
  int16_t i;
  uint8_t mac_addr[6];
  memcpy(mac_addr+2,(uint8_t*)&NRF_FICR->DEVICEADDR[0],4);  // low 4 bytes
  memcpy(mac_addr,(uint8_t*)&NRF_FICR->DEVICEADDR[1],2);    // hi 2 bytes
  mac_addr[0] |= 0xc0;  // force first two bits to be 1 per the standard
  for (i=0; i<sizeof(master_addr_list); i++) {
    if (memcmp(master_addr_list[i].addr,mac_addr,6) == 0) break;
    }
  if (i >= sizeof(master_addr_list)) {
    NRF_LOG_INFO("mac address, not in table, is");
    NRF_LOG_HEXDUMP_INFO((uint8_t*)mac_addr,6);
    NRF_LOG_FLUSH();
    APP_ERROR_CHECK(1);
    return -1;
    };
  return i+1024;
  }

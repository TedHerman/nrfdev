#include <string.h>
#include "ble_gap.h" 

typedef struct {
  uint8_t addr[BLE_GAP_ADDR_LEN]; 
  } master_addr_t;

master_addr_t master_addr_list[] = 
  { { {0xfe,0xe0,0x17,0x9c,0xd9,0xe3} },   // mote 00
    { {0x46,0x8d,0xb6,0x76,0x66,0xfe} },
    { {0x28,0xc1,0x6c,0xde,0xa6,0xf4} },
    { {0xfc,0x2d,0x34,0x39,0x2a,0xd5} } };

int16_t find_mac_addr(ble_gap_addr_t ble_addr) {
  int16_t i;
  for (i=0; i<sizeof(master_addr_list); i++) {
    if (memcmp(master_addr_list[i].addr,
               (uint8_t*)&ble_addr.addr,
               BLE_GAP_ADDR_LEN) == 0) break;
    }
  if (i >= sizeof(master_addr_list)) return -1;
  return i;
  }

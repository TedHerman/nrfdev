#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_802154.h"
#include "nrf_802154_types.h"
#include "boards.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_802154_clock.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "IEEE802154.h"

#define MAX_MESSAGE_SIZE 127
#define CHANNEL 26 
#define PACKET_MAX_PAYLOAD 26 

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

ret_code_t err_code;

uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * p = (motepacket_t *)&buffer;

static volatile uint32_t rx_counter;

uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t short_address[]    = {0x06, 0x07};
uint8_t pan_id[]           = {0x04, 0x05};

//int main(int argc, char *argv[])
int main(void) {
    // initialize the logging
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    bsp_board_init(BSP_INIT_LEDS);
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_init();
    nrf_802154_init();
    // if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();

    // these three are for filtering, which we don't use (probably these could be deleted) 
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);

    // promiscuous mode appears to be necessary
    nrf_802154_promiscuous_set(true);

    nrf_802154_channel_set(CHANNEL);

    // this puts the stack into permanent receive mode
    nrf_802154_receive();
    if (nrf_802154_state_get() == NRF_802154_STATE_RECEIVE) 
       NRF_LOG_INFO("radio in receive state");
    while (true) {
	NRF_LOG_FLUSH();
        nrf_pwr_mgmt_run();
        }
    return 0;
    }

void nrf_802154_received_raw(uint8_t * p_data, int8_t power, uint8_t lqi) {
    (void) power;
    (void) lqi;
    uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t); 
    bsp_board_led_invert(1);
    NRF_LOG_INFO("Packet %d",rx_counter);
    NRF_LOG_HEXDUMP_INFO(p_data, 40);

    memcpy(buffer,p_data+1,size);
    rx_counter++;

    nrf_802154_buffer_free_raw(p_data);

    //nrf_802154_receive();
    }

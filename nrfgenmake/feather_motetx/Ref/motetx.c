#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_802154.h"
#include "nrf_802154_types.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "IEEE802154.h"

#define MAX_MESSAGE_SIZE 37 
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

bool m_tx_in_progress;
uint8_t seqno = 0; 
uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * p = (motepacket_t *)&buffer;

//int main(int argc, char *argv[])
int main(void) {
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(0);

    p->fcf = 0x0000;
    // the following FCF lines are copied from TinyOS CC2420 driver
    p->fcf &= 1 << IEEE154_FCF_ACK_REQ;
    p->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    p->data_seq_no = 0;
    p->pan = 0x22;     // default group
    p->dest = 0xFFFF;  // dest addr 
    p->src = 99;       // src addr 
    p->iframe = 0x3f;  // 6LowPan designator is 0x3f
    p->type = 0x37;    // message type 
    p->length =  offsetof(motepacket_t,data) + PACKET_MAX_PAYLOAD + 2 - 1;
    NRF_LOG_INFO("packet length = %d",p->length);

    nrf_802154_init();
    //nrf_802154_short_address_set(short_address);
    //nrf_802154_extended_address_set(extended_address);
    //nrf_802154_pan_id_set(pan_id);
    //nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

    while (1) {
        p->data_seq_no = seqno++;
        for (int i=0; i<PACKET_MAX_PAYLOAD; i++) p->data[i] = seqno+i;
        bsp_board_led_invert(0);
	while (m_tx_in_progress) {
           nrf_delay_ms(200);
	   if (nrf_802154_state_get() != NRF_802154_STATE_TRANSMIT) {
              m_tx_in_progress = false;
              bsp_board_led_invert(1);
              }		   
           }
	m_tx_in_progress = nrf_802154_transmit_raw(buffer, true);
	NRF_LOG_INFO("transmit raw in progress = %d",m_tx_in_progress);
        nrf_delay_ms(2000);
        }

    return 0;
}

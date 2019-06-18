#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_802154.h"
#include "boards.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "IEEE802154.h"

#define MAX_MESSAGE_SIZE 37
#define CHANNEL 26
#define PACKET_MAX_PAYLOAD 26

static uint8_t counter = 0;
static uint32_t msgnum = 0;

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

uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * packetptr = (motepacket_t *)buffer;

// define scheduler parameters
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE            10
APP_TIMER_DEF(main_timer_id);

// int8_t  powervalues[] = {-30,-20,-16,-12,-8,-4,0,4};
uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
uint8_t short_address[]    = {0x06, 0x07};
uint8_t pan_id[]           = {0x04, 0x05};

void setupMsg() {
    packetptr->fcf = 0x0000;
    packetptr->fcf &= 1 << IEEE154_FCF_ACK_REQ;
    packetptr->fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    packetptr->data_seq_no = counter++;
    packetptr->pan = 0x22;     // GROUP
    packetptr->dest = 0xFFFF;  // BCAST_ADDR
    packetptr->src = 99;       // ID of sender
    packetptr->iframe = 0x3f;  // 6LowPan designator
    packetptr->type = 0x37;    // (bogus for now, could have meaning later)
    // -1 for length, +2 for unknown reason - seems to sort of work,
    // perhaps because of the CRC bytes in the fcs?
    packetptr->length =  offsetof(motepacket_t,data) 
	    + 2 + PACKET_MAX_PAYLOAD - 1;
    for (int i=0; i<PACKET_MAX_PAYLOAD; i++) packetptr->data[i] = counter+i;
    } 

// xmit message 
void triggered(void * parameter, uint16_t size) {
    NRF_LOG_INFO("transmit %d",msgnum++);
    NRF_LOG_FLUSH();
    bsp_board_led_invert(0);
    setupMsg();
    nrf_802154_transmit_raw(buffer,false);
    }

// callback from timer expiration
static void timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&triggered);
    }

//int main(int argc, char *argv[])
int main(void) {
    uint32_t err_code;
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(0);

    // use power management later in main loop, so init here
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    // initialize app scheduler and app timer, with initial timer
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&main_timer_id,
                 APP_TIMER_MODE_REPEATED,
                 timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(main_timer_id,
               APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

    nrf_802154_init();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_tx_power_set(4);  // 4 is the supposed maximum power
       // but if it has bad results try 0 and see how that works
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

     for (;;) {
        app_sched_execute();
        nrf_pwr_mgmt_run();
        }

    }

void nrf_802154_transmitted_raw(const uint8_t * p_frame, uint8_t * p_ack, int8_t power, uint8_t lqi)
{
    bsp_board_led_invert(2);
    if (p_ack != NULL) nrf_802154_buffer_free_raw(p_ack);
    nrf_802154_receive();
}

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_802154.h"
#include "boards.h"

#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define MAX_MESSAGE_SIZE 127
#define CHANNEL          26 

//static uint8_t m_message[MAX_MESSAGE_SIZE];

static volatile uint32_t rx_counter;

int main(int argc, char *argv[])
{
    (void) argc;
    (void) argv;

    uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
    uint8_t short_address[]    = {0x06, 0x07};
    uint8_t pan_id[]           = {0x04, 0x05};
    uint8_t state;

    // initialize the logging
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(0);

    nrf_pwr_mgmt_init();

    nrf_802154_init();

    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);

    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    state = nrf_802154_state_get();

    NRF_LOG_INFO("Launch with 802.15.4 in state %d.",state)
    NRF_LOG_FLUSH();

    while (1) {
       nrf_pwr_mgmt_run();
       }

    return 0;
}

void nrf_802154_received_raw(uint8_t * p_data, int8_t power, uint8_t lqi)
{
    (void) power;
    (void) lqi;

    NRF_LOG_INFO("Packet");
    NRF_LOG_HEXDUMP_INFO(p_data, 40);
    NRF_LOG_FLUSH();

    rx_counter++;

    nrf_802154_buffer_free_raw(p_data);
    bsp_board_led_invert(1);

    //nrf_802154_receive();
    return;
}

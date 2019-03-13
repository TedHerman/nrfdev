#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf_802154.h"
#include "boards.h"
#include "nrf_delay.h"


#define MAX_MESSAGE_SIZE 40 
#define CHANNEL          26

static volatile bool m_tx_in_progress;
static volatile bool m_tx_done;
static uint8_t counter = 0;
uint8_t message[MAX_MESSAGE_SIZE];

void setupMsg() {
    for (uint8_t i = 0; i < MAX_MESSAGE_SIZE; i++) message[i] = counter+i; 
    counter++;
    message[0] = MAX_MESSAGE_SIZE;
    message[1] = 0x41;                // Set MAC header: short addresses, no ACK
    message[2] = 0x98;                // Set MAC header
    } 

//int main(int argc, char *argv[])
int main(void)
{
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_led_invert(0);

    m_tx_in_progress = false;
    m_tx_done        = false;

    nrf_802154_init();
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();

    while (1)
    {
        bsp_board_led_invert(0);
        setupMsg();
        // if (m_tx_done)
        // {
        //     m_tx_in_progress = false;
        //     m_tx_done        = false;
        // }

        // if (!m_tx_in_progress)
        // {
            //tx_buffer_fill(message, sizeof(message));

            //m_tx_in_progress = nrf_802154_transmit(message, sizeof(message), true);
            m_tx_in_progress = nrf_802154_transmit_raw(message, false);
            if (m_tx_in_progress) {
                bsp_board_led_invert(1);
            }
        // }
        nrf_delay_ms(3000);

    }

    return 0;
}

void nrf_802154_transmitted(const uint8_t * p_frame, uint8_t * p_ack, uint8_t length, int8_t power, int8_t lqi)
{
    (void) p_frame;
    (void) length;
    (void) power;
    (void) lqi;

    m_tx_done = true;
             bsp_board_led_invert(2);

    if (p_ack != NULL)
    {
        //nrf_802154_buffer_free(p_ack);
        nrf_802154_buffer_free_raw(p_ack);
    }
}

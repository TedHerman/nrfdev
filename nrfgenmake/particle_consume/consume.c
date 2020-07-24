#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_atomic.h"
#include "nrfx_wdt.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "app_button.h"
#include "app_gpiote.h"
#include "nrf_802154.h"
#include "nrf_radio.h"
#include "nrf_802154_types.h"
#include "nrf_802154_clock.h"
#include "boards.h"
#include "IEEE802154.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "ble_gap.h"
#include "bsp.h"
#include "motebadge.h"

#define CHANNEL 26 // for 802.15.4 radio 
#define BEACON_PERIOD 80 

#define APP_BLE_CONN_CFG_TAG 1 // identifying the SoftDevice BLE configuration
#define APP_BLE_OBSERVER_PRIO 3 
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

#define APP_TIMER_PRESCALER     0  // Value of the RTC1 PRESCALER register.
#define APP_TIMER_MAX_TIMERS    1  // Maximum number of simultaneously created timers.
#define APP_TIMER_OP_QUEUE_SIZE 3  // Size of timer operation queues.
#define BUTTON_DEBOUNCE_DELAY   50 // Delay from a GPIOTE event until a button is reported as pushed.
#define APP_GPIOTE_MAX_USERS    1  // Maximum number of users of the GPIOTE handler.

uint32_t random_write_delay(uint32_t bound);
static void mote_timeout_handler(void * p_context);
void timers_start(void);
void moterestart(void); 
void motesetup(void);

/****** Timers *********************************************************/
APP_TIMER_DEF(clock_id);       // 1 sec tick / atomic increment of clock
APP_TIMER_DEF(mote_timer_id);  // 8 sec tick / consolidated report/record
APP_TIMER_DEF(transmit_timer_id); // in (0,8) sec / randomized report send
APP_TIMER_DEF(rejuvenate_id);  // 60 sec tick / rejuvenate 802.15.4 stack

// global variables of this module
static nrf_atomic_u32_t clock;
uint16_t local_mote_id;
uint16_t data_seq_no;
motepacket_t prepare_report; 
mote_report_t consolidated;
mote_received_t observation; 
uint32_t randvalue;
uint8_t seqno = 0; 
bool xfer_active;
bool is802154running;
uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * packetptr = (motepacket_t *)buffer;
uint8_t buffer[sizeof(motepacket_t)];
motepacket_t * p = (motepacket_t *)&buffer;

// const int8_t  powervalues[] = {-30,-20,-16,-12,-8,-4,0,4};
const uint8_t extended_address[] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef};
const uint8_t short_address[]    = {0x06, 0x07};
const uint8_t pan_id[]           = {0x04, 0x05};

// colors for blinks
uint16_t colorCount;
uint8_t rgb0[3] = {0x11,0x00,0x11};
uint8_t rgb1[3] = {0x11,0x11,0x00};
uint8_t rgb2[3] = {0x00,0x11,0x11};
uint8_t rgb3[3] = {0x00,0x33,0x00};

// trap for assert failures
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name) {
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
    }

// background clock tick 
static void clock_timeout_handler(void * p_context) { 
    nrf_atomic_u32_fetch_add(&clock,1);
    }

// force regular restarts of 802.15.4 stack just so that 
// the 120-second limit on RAAL doesn't shut down receiving 
void do_moterestart(void * parameter, uint16_t size) { moterestart(); }
static void rejuvenation_handler(void * p_context) { 
    if (xfer_active) return;
    app_sched_event_put(NULL,1,&do_moterestart);
    }

void report_init(mote_report_t * input) {
    mote_report_t * p = (mote_report_t*)prepare_report.data;
    memset((uint8_t*)&prepare_report,0,sizeof(motepacket_t));
    prepare_report.fcf = 0x0000;
    prepare_report.fcf &= 1 << IEEE154_FCF_ACK_REQ;
    prepare_report.fcf |= ( ( IEEE154_TYPE_DATA << IEEE154_FCF_FRAME_TYPE )
                  |  ( 1 << IEEE154_FCF_INTRAPAN )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                  |  ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) );
    prepare_report.data_seq_no = data_seq_no++;
    prepare_report.pan = 0x22;     // GROUP
    prepare_report.dest = 0xFFFF;  // BCAST_ADDR
    prepare_report.src = local_mote_id; // ID of sender
    prepare_report.iframe = 0x3f;  // 6LowPan designator
    prepare_report.type = 0x37;    // (bogus for now, could have meaning later)
    // -1 for length, +2 for unknown reason - seems to sort of work,
    // perhaps because of the CRC bytes in the fcs?
    prepare_report.length =  offsetof(motepacket_t,data) + 2 + sizeof(mote_report_t) - 1;
    p->time = clock;
    p->mote_id = local_mote_id;
    p->num_reports = 0;
    if (input != NULL) memcpy((uint8_t*)p,(uint8_t*)input,sizeof(mote_report_t));
    }

void ble_stack_init(void) {
    ret_code_t err_code;
    ble_gap_addr_t ble_addr;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // read the device MAC address
    sd_ble_gap_addr_get(&ble_addr);
    local_mote_id = 5;
    }

void nrf_802154_received_timestamp_raw(uint8_t * p_data, int8_t power, uint8_t lqi, uint32_t time) {
    uint8_t size = (p_data[0]<sizeof(motepacket_t)) ? p_data[0] : sizeof(motepacket_t);
    motepacket_t * q = (motepacket_t *)p_data;
    UNUSED_VARIABLE(size);  // may be tested in future revisions of this code
    if (q->type == 0x36) {
       nrf_atomic_u32_store(&clock,*(uint32_t *)q->data);
       nrfx_wdt_feed();
       nrf_802154_buffer_free_raw(p_data);
       return;
       }
    if (q->type == 0x37) {
       mote_report_t * s =  (mote_report_t *)q->data;
       observation.mote_id = s->mote_id;
       observation.rssi = power;
       nrf_802154_buffer_free_raw(p_data);
       nrfx_wdt_feed();
       // bsp_board_led_invert(1);
       return;
       }
    if (q->type == 0x38) {
       }
    nrf_802154_buffer_free_raw(p_data);
    }

// called periodically to refresh stack
void moterestart(void) {
    ret_code_t err_code;
    NRF_LOG_INFO("mote restart");
    if (is802154running) {
      nrf_802154_deinit();
      is802154running = false;
      }
    /**** coming back from xfer, nrf_sdh is already disabled ****/
    if (nrf_sdh_is_enabled()) {
       err_code = nrf_sdh_disable_request();
       APP_ERROR_CHECK(err_code);
       }
    ble_stack_init();
    nrf_drv_clock_init();
    nrf_802154_init();
    is802154running = true;
    if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    }

// called once, for each reboot
void motesetup() {
    ble_stack_init();
    nrf_drv_clock_init();
    nrf_802154_init();
    is802154running = true;
    if (!nrf_802154_clock_hfclk_is_running()) nrf_802154_clock_hfclk_start();
    nrf_802154_short_address_set(short_address);
    nrf_802154_extended_address_set(extended_address);
    nrf_802154_pan_id_set(pan_id);
    nrf_802154_promiscuous_set(true);
    nrf_802154_channel_set(CHANNEL);
    nrf_802154_receive();
    }

// handler called regularly in each beacon period to produce consolidate
// report of observations, recording in long-term storage
static void mote_timeout_handler(void * p_context) {
    ret_code_t err_code;
    if (xfer_active) return;
    report_init(&consolidated);  
    err_code = app_timer_start(transmit_timer_id,
               APP_TIMER_TICKS(random_write_delay(10)), NULL);
    APP_ERROR_CHECK(err_code);
    }

void nrf_802154_transmitted_raw(const uint8_t *p_frame,
                uint8_t *p_ack, int8_t power, uint8_t lqi) {
    nrf_802154_receive();
    nrf_gpio_cfg_output(LED_PRIMARY_PIN);
    nrf_gpio_cfg_output(LED_RGB_RED_PIN);
    nrf_gpio_cfg_output(LED_RGB_GREEN_PIN);
    nrf_gpio_cfg_output(LED_RGB_BLUE_PIN);
    if (colorCount % 4 == 0) nrf_gpio_pin_toggle(LED_PRIMARY_PIN);
    if (colorCount % 4 == 1) nrf_gpio_pin_toggle(LED_RGB_RED_PIN);
    if (colorCount % 4 == 2) nrf_gpio_pin_toggle(LED_RGB_GREEN_PIN);
    if (colorCount % 4 == 3) nrf_gpio_pin_toggle(LED_RGB_BLUE_PIN);
    colorCount++;
    }
void nrf_802154_transmit_failed (const uint8_t *p_frame, nrf_802154_tx_error_t error) {
    nrf_802154_receive();
    }
void mote_transmit(void * parameter, uint16_t size) { 
    nrf_802154_transmit_raw((uint8_t*)&prepare_report,true);
    nrfx_wdt_feed();
    // bsp_board_led_invert(0);
    }
static void transmit_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&mote_transmit);
    }

void board_init(void) {
    bsp_board_init(BSP_INIT_LEDS);
    }

void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    }

void power_management_init(void) {
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    }


void timers_init(void) {
    ret_code_t err_code;
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    // report generation & restart 
    err_code = app_timer_create(&mote_timer_id,
		                APP_TIMER_MODE_REPEATED,
                                mote_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // report/probe transmission
    err_code = app_timer_create(&transmit_timer_id,
		                APP_TIMER_MODE_SINGLE_SHOT,
                                transmit_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // clock tick
    err_code = app_timer_create(&clock_id,
		                APP_TIMER_MODE_REPEATED,
                                clock_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // rejuvenation timer - for bug in 802.15.4 stack
    err_code = app_timer_create(&rejuvenate_id,
		                APP_TIMER_MODE_REPEATED,
                                rejuvenation_handler);
    APP_ERROR_CHECK(err_code);
    }

void timers_start(void) {
    ret_code_t err_code;
    // regular background tasks: clock tick and rejuvenation
    err_code = app_timer_start(clock_id,APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(rejuvenate_id,APP_TIMER_TICKS(60000), NULL);
    APP_ERROR_CHECK(err_code);
    // kick off first cycle, report and restart 
    err_code = app_timer_start(mote_timer_id, APP_TIMER_TICKS(BEACON_PERIOD), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(transmit_timer_id,
               APP_TIMER_TICKS(random_write_delay(BEACON_PERIOD)), NULL);
    APP_ERROR_CHECK(err_code);
    }

uint32_t random_write_delay(uint32_t bound) {
    return 10;
    }

int main(void) {
    log_init();
    board_init();
    power_management_init();
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    NRF_LOG_INFO("Initialization starts ...");
    NRF_LOG_FLUSH();
    motesetup();
    report_init(NULL);   // first-time empty report 
    timers_init();
    timers_start();

    // Enter main loop.
    for (;;) {
	NRF_LOG_FLUSH();
	app_sched_execute();
  	sd_app_evt_wait();
        // nrf_pwr_mgmt_run();
        }
    }


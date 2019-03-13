#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rng.h"

// define scheduler parameters
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE            10
// define timer for blinking
APP_TIMER_DEF(blink_timer_id);      

// define universal err_code variable once
ret_code_t err_code;

// random number generated here
uint32_t randvalue;

// flip led and reset timer randomly
void flipled(void * parameter, uint16_t size) {
    uint8_t * p = (uint8_t*)&randvalue;	
    bsp_board_led_invert(0);
    err_code = nrf_drv_rng_rand(p,4);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(blink_timer_id,
               APP_TIMER_TICKS(randvalue%501), NULL);
    APP_ERROR_CHECK(err_code);
    }

// callback from timer expiration
static void blink_timeout_handler(void * p_context) {
    app_sched_event_put(NULL,1,&flipled);
    }

void initialize() {
    // configure board for leds, set up clock and rng 
    bsp_board_init(BSP_INIT_LEDS);
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
    err_code = nrf_drv_rng_init(NULL);
    APP_ERROR_CHECK(err_code);
    // use power management later in main loop, so init here
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    // initialize app scheduler and app timer, with initial timer
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&blink_timer_id,
               APP_TIMER_MODE_SINGLE_SHOT, 
               blink_timeout_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(blink_timer_id,
               APP_TIMER_TICKS(500), NULL);
    APP_ERROR_CHECK(err_code);
    }

int main(void) {
    initialize();
    for (;;) {
        app_sched_execute();
        nrf_pwr_mgmt_run();
        }
    }

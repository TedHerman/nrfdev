#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_pwm.h"

#define LED_RGB_RED   1
#define LED_RGB_BLUE  2
#define LED_RGB_GREEN 3
#define LED_PRIMARY 0

static uint16_t led_duty_cycles[PWM0_CH_NUM] = { 0 };
static uint32_t _systick_count = 0;
static uint32_t primary_cycle_length = 300;

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle);
void neopixel_write (uint8_t *pixels);

void led_tick() {
    uint32_t millis = _systick_count;
    uint32_t cycle = millis % primary_cycle_length;
    uint32_t half_cycle = primary_cycle_length / 2;
    if (cycle > half_cycle) {
        cycle = primary_cycle_length - cycle;
    }
    uint16_t duty_cycle = 0x4f * cycle / half_cycle;
    #if LED_STATE_ON == 1
    duty_cycle = 0xff - duty_cycle;
    #endif
    led_pwm_duty_cycle(LED_PRIMARY, duty_cycle);

    #ifdef LED_SECONDARY_PIN
    cycle = millis % secondary_cycle_length;
    half_cycle = secondary_cycle_length / 2;
    if (cycle > half_cycle) {
        cycle = secondary_cycle_length - cycle;
    }
    duty_cycle = 0x8f * cycle / half_cycle;
    #if LED_STATE_ON == 1
    duty_cycle = 0xff - duty_cycle;
    #endif
    led_pwm_duty_cycle(LED_SECONDARY, duty_cycle);
    #endif
}


void SysTick_Handler(void) {
  _systick_count++;
  led_tick();
}


void led_pwm_init(uint32_t led_index, uint32_t led_pin)
{
  NRF_PWM_Type* pwm    = NRF_PWM0;

  pwm->ENABLE = 0;

  nrf_gpio_cfg_output(led_pin);
  nrf_gpio_pin_write(led_pin, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[led_index] = led_pin;

  pwm->MODE            = PWM_MODE_UPDOWN_Up;
  pwm->COUNTERTOP      = 0xff;
  pwm->PRESCALER       = PWM_PRESCALER_PRESCALER_DIV_16;
  pwm->DECODER         = PWM_DECODER_LOAD_Individual;
  pwm->LOOP            = 0;

  pwm->SEQ[0].PTR      = (uint32_t) (led_duty_cycles);
  pwm->SEQ[0].CNT      = 4; // default mode is Individual --> count must be 4
  pwm->SEQ[0].REFRESH  = 0;
  pwm->SEQ[0].ENDDELAY = 0;

  pwm->ENABLE = 1;
  pwm->EVENTS_SEQEND[0] = 0;
  //  pwm->TASKS_SEQSTART[0] = 1;
  }

void pwm_teardown(NRF_PWM_Type* pwm ) {
  pwm->TASKS_SEQSTART[0] = 0;
  pwm->ENABLE            = 0;

  pwm->PSEL.OUT[0] = 0xFFFFFFFF;
  pwm->PSEL.OUT[1] = 0xFFFFFFFF;
  pwm->PSEL.OUT[2] = 0xFFFFFFFF;
  pwm->PSEL.OUT[3] = 0xFFFFFFFF;

  pwm->MODE        = 0;
  pwm->COUNTERTOP  = 0x3FF;
  pwm->PRESCALER   = 0;
  pwm->DECODER     = 0;
  pwm->LOOP        = 0;
  pwm->SEQ[0].PTR  = 0;
  pwm->SEQ[0].CNT  = 0;
  }

void led_pwm_duty_cycle(uint32_t led_index, uint16_t duty_cycle) {
  led_duty_cycles[led_index] = duty_cycle;
  nrf_pwm_event_clear(NRF_PWM0, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(NRF_PWM0, NRF_PWM_TASK_SEQSTART0);
  }

void neopixel_init(void)
{
  led_pwm_init(LED_RGB_RED, LED_RGB_RED_PIN);
  led_pwm_init(LED_RGB_GREEN, LED_RGB_GREEN_PIN);
  led_pwm_init(LED_RGB_BLUE, LED_RGB_BLUE_PIN);
}

void neopixel_teardown(void)
{
  uint8_t grb[3] = { 0, 0, 0 };
  neopixel_write(grb);
  nrf_gpio_cfg_default(LED_RGB_RED_PIN);
  nrf_gpio_cfg_default(LED_RGB_GREEN_PIN);
  nrf_gpio_cfg_default(LED_RGB_BLUE_PIN);
}

// write 3 bytes color to a built-in neopixel
void neopixel_write (uint8_t *pixels)
{
  led_pwm_duty_cycle(LED_RGB_RED, pixels[2]);
  led_pwm_duty_cycle(LED_RGB_GREEN, pixels[1]);
  led_pwm_duty_cycle(LED_RGB_BLUE, pixels[0]);
}


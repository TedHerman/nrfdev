#include <stdbool.h>
#include <stdint.h>
#include "boards.h"
#include "nrf_assert.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_pwm.h"

// WS2812B (rev B) timing is 0.4 and 0.8 us
#define MAGIC_T0H               6UL | (0x8000) // 0.375us
#define MAGIC_T1H              13UL | (0x8000) // 0.8125us
#define CTOPVAL                20UL            // 1.25us

#define NEO_NUMBYTE  3

static uint16_t pixels_pattern[NEO_NUMBYTE * 8 + 2];

// write 3 bytes color to a built-in neopixel
void neopixel_write (uint8_t *pixels) {
  uint8_t grb[NEO_NUMBYTE] = {pixels[1], pixels[2], pixels[0]};
  uint16_t pos = 0;    // bit position
  for ( uint16_t n = 0; n < NEO_NUMBYTE; n++ ) {
    uint8_t pix = grb[n];
    for ( uint8_t mask = 0x80; mask > 0; mask >>= 1 ) {
      pixels_pattern[pos] = (pix & mask) ? MAGIC_T1H : MAGIC_T0H;
      pos++;
      }
    }
  // Zero padding to indicate the end of sequence
  pixels_pattern[pos++] = 0 | (0x8000);    // Seq end
  pixels_pattern[pos++] = 0 | (0x8000);    // Seq end

  NRF_PWM_Type* pwm = NRF_PWM1;

  nrf_pwm_seq_ptr_set(pwm, 0, pixels_pattern);
  nrf_pwm_seq_cnt_set(pwm, 0, sizeof(pixels_pattern)/2);
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
  nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);

  // blocking wait for sequence complete
  while( !nrf_pwm_event_check(pwm, NRF_PWM_EVENT_SEQEND0) ) {}
  nrf_pwm_event_clear(pwm, NRF_PWM_EVENT_SEQEND0);
  }

void pwm_init() {
  uint16_t led_duty_cycles[PWM1_CH_NUM] = { 0 };
  NRF_PWM_Type* pwm    = NRF_PWM1;
  pwm->ENABLE = 0;
  nrf_gpio_cfg_output(LED_NEOPIXEL);
  nrf_gpio_pin_write(LED_NEOPIXEL, 1 - LED_STATE_ON);

  pwm->PSEL.OUT[LED_NEOPIXEL] = LED_NEOPIXEL;

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
  // pwm->TASKS_SEQSTART[0] = 1;
  }

// use PWM1 for neopixel
void neopixel_init(void) {
  // To support both the SoftDevice + Neopixels we use the EasyDMA
  // feature from the NRF25. However this technique implies to
  // generate a pattern and store it on the memory. The actual
  // memory used in bytes corresponds to the following formula:
  //              totalMem = numBytes*8*2+(2*2)
  // The two additional bytes at the end are needed to reset the
  // sequence.
  NRF_PWM_Type* pwm = NRF_PWM1;

  // Set the wave mode to count UP
  // Set the PWM to use the 16MHz clock
  // Setting of the maximum count
  // but keeping it on 16Mhz allows for more granularity just
  // in case someone wants to do more fine-tuning of the timing.
  nrf_pwm_configure(pwm, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, CTOPVAL);

  // Disable loops, we want the sequence to repeat only once
  nrf_pwm_loop_set(pwm, 0);
  // On the "Common" setting the PWM uses the same pattern for the
  // for supported sequences. The pattern is stored on half-word of 16bits
  nrf_pwm_decoder_set(pwm, PWM_DECODER_LOAD_Common, PWM_DECODER_MODE_RefreshCount);

  // The following settings are ignored with the current config.
  nrf_pwm_seq_refresh_set(pwm, 0, 0);
  nrf_pwm_seq_end_delay_set(pwm, 0, 0);

  // The Neopixel implementation is a blocking algorithm. DMA
  // allows for non-blocking operation. To "simulate" a blocking
  // operation we enable the interruption for the end of sequence
  // and block the execution thread until the event flag is set by
  // the peripheral.
  //    pwm->INTEN |= (PWM_INTEN_SEQEND0_Enabled<<PWM_INTEN_SEQEND0_Pos);

  // PSEL must be configured before enabling PWM
  nrf_pwm_pins_set(pwm, 
    (uint32_t[] ) { LED_NEOPIXEL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL });

  // Enable the PWM
  nrf_pwm_enable(pwm);
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

void neopixel_teardown(void) {
  uint8_t grb[3] = { 0, 0, 0 };

  NRFX_DELAY_US(50);  // wait for previous write is complete

  neopixel_write(grb);
  NRFX_DELAY_US(50);  // wait for this write

  // pwm_teardown(NRF_PWM1);
  }

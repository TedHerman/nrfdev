#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

const uint8_t leds_list[5] = {18,19,20,21,22};

int main(void) {
  uint8_t i;
  uint8_t counter = 0;
  uint32_t pins;
  for (i=0;i<5;i++) nrf_gpio_cfg_output(leds_list[i]);  
  while (true) {
    nrf_delay_ms(1000);
    counter++;
    for (i=0;i<5;i++) {
      pins = NRF_GPIO->OUT; 
      if (counter & (1<<i)) {
         pins = ~pins;
         NRF_GPIO->OUTCLR = 1 << leds_list[i]; 
         NRF_GPIO->OUTSET = 0; 
         }
      else {
         NRF_GPIO->OUTSET = 1 << leds_list[i];
         NRF_GPIO->OUTCLR = 0;
         }
      }
    }
  }

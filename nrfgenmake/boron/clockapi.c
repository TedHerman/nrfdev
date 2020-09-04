#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "app_error.h"
#include "app_util.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_atomic.h"
#include "nrf_drv_twi.h"

#include "boards.h"
#include "bsp.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE
#define SCHED_QUEUE_SIZE 256

#define DS3231_ADDR (0xD0U >> 1)

// typedef void (*clockapi_twi_handler_t)(nrf_drv_twi_evt_t const * p_event, void * p_context);
extern bool i2c_setaddr(uint8_t addr);
extern bool i2c_isbusy();
extern ret_code_t i2c_rx(uint8_t* buffer, uint8_t size);
extern ret_code_t i2c_tx(uint8_t* buffer, uint8_t size, bool cont);

extern void auxiliary_tick_handler();

// global variables of this module
static nrf_atomic_u32_t clock;
static uint32_t fresh_epoch_clock;
uint8_t clock_buffer[4];
uint8_t ds3231clock[6]; // for sec,min,hour,day,month,year
uint8_t ds3231clock_raw[7];
static volatile uint8_t twi_mode = 0;  // 0 => idle; 1 => phase1; 2 => phase2; etc

// constant for date -> epoch time conversion
// from 2020 through 2037 (37-20+1 = 18)
const uint8_t depoch[18*12][7] = {   
   // y,m,d,t,t,t,t where tttt is the epoch time
   {20,1,1,94,12,53,96},
   {20,2,1,94,53,19,224},
   {20,3,1,94,91,79,96},
   {20,4,1,94,132,31,208},
   {20,5,1,94,171,172,208},
   {20,6,1,94,212,139,80},
   {20,7,1,94,252,24,80},
   {20,8,1,95,36,246,208},
   {20,9,1,95,77,213,80},
   {20,10,1,95,117,98,80},
   {20,11,1,95,158,64,208},
   {20,12,1,95,197,219,224},
   {21,1,1,95,238,186,96},
   {21,2,1,96,23,152,224},
   {21,3,1,96,60,130,224},
   {21,4,1,96,101,83,80},
   {21,5,1,96,140,224,80},
   {21,6,1,96,181,190,208},
   {21,7,1,96,221,75,208},
   {21,8,1,97,6,42,80},
   {21,9,1,97,47,8,208},
   {21,10,1,97,86,149,208},
   {21,11,1,97,127,116,80},
   {21,12,1,97,167,15,96},
   {22,1,1,97,207,237,224},
   {22,2,1,97,248,204,96},
   {22,3,1,98,29,182,96},
   {22,4,1,98,70,134,208},
   {22,5,1,98,110,19,208},
   {22,6,1,98,150,242,80},
   {22,7,1,98,190,127,80},
   {22,8,1,98,231,93,208},
   {22,9,1,99,16,60,80},
   {22,10,1,99,55,201,80},
   {22,11,1,99,96,167,208},
   {22,12,1,99,136,66,224},
   {23,1,1,99,177,33,96},
   {23,2,1,99,217,255,224},
   {23,3,1,99,254,233,224},
   {23,4,1,100,39,186,80},
   {23,5,1,100,79,71,80},
   {23,6,1,100,120,37,208},
   {23,7,1,100,159,178,208},
   {23,8,1,100,200,145,80},
   {23,9,1,100,241,111,208},
   {23,10,1,101,24,252,208},
   {23,11,1,101,65,219,80},
   {23,12,1,101,105,118,96},
   {24,1,1,101,146,84,224},
   {24,2,1,101,187,51,96},
   {24,3,1,101,225,110,224},
   {24,4,1,102,10,63,80},
   {24,5,1,102,49,204,80},
   {24,6,1,102,90,170,208},
   {24,7,1,102,130,55,208},
   {24,8,1,102,171,22,80},
   {24,9,1,102,211,244,208},
   {24,10,1,102,251,129,208},
   {24,11,1,103,36,96,80},
   {24,12,1,103,75,251,96},
   {25,1,1,103,116,217,224},
   {25,2,1,103,157,184,96},
   {25,3,1,103,194,162,96},
   {25,4,1,103,235,114,208},
   {25,5,1,104,18,255,208},
   {25,6,1,104,59,222,80},
   {25,7,1,104,99,107,80},
   {25,8,1,104,140,73,208},
   {25,9,1,104,181,40,80},
   {25,10,1,104,220,181,80},
   {25,11,1,105,5,147,208},
   {25,12,1,105,45,46,224},
   {26,1,1,105,86,13,96},
   {26,2,1,105,126,235,224},
   {26,3,1,105,163,213,224},
   {26,4,1,105,204,166,80},
   {26,5,1,105,244,51,80},
   {26,6,1,106,29,17,208},
   {26,7,1,106,68,158,208},
   {26,8,1,106,109,125,80},
   {26,9,1,106,150,91,208},
   {26,10,1,106,189,232,208},
   {26,11,1,106,230,199,80},
   {26,12,1,107,14,98,96},
   {27,1,1,107,55,64,224},
   {27,2,1,107,96,31,96},
   {27,3,1,107,133,9,96},
   {27,4,1,107,173,217,208},
   {27,5,1,107,213,102,208},
   {27,6,1,107,254,69,80},
   {27,7,1,108,37,210,80},
   {27,8,1,108,78,176,208},
   {27,9,1,108,119,143,80},
   {27,10,1,108,159,28,80},
   {27,11,1,108,199,250,208},
   {27,12,1,108,239,149,224},
   {28,1,1,109,24,116,96},
   {28,2,1,109,65,82,224},
   {28,3,1,109,103,142,96},
   {28,4,1,109,144,94,208},
   {28,5,1,109,183,235,208},
   {28,6,1,109,224,202,80},
   {28,7,1,110,8,87,80},
   {28,8,1,110,49,53,208},
   {28,9,1,110,90,20,80},
   {28,10,1,110,129,161,80},
   {28,11,1,110,170,127,208},
   {28,12,1,110,210,26,224},
   {29,1,1,110,250,249,96},
   {29,2,1,111,35,215,224},
   {29,3,1,111,72,193,224},
   {29,4,1,111,113,146,80},
   {29,5,1,111,153,31,80},
   {29,6,1,111,193,253,208},
   {29,7,1,111,233,138,208},
   {29,8,1,112,18,105,80},
   {29,9,1,112,59,71,208},
   {29,10,1,112,98,212,208},
   {29,11,1,112,139,179,80},
   {29,12,1,112,179,78,96},
   {30,1,1,112,220,44,224},
   {30,2,1,113,5,11,96},
   {30,3,1,113,41,245,96},
   {30,4,1,113,82,197,208},
   {30,5,1,113,122,82,208},
   {30,6,1,113,163,49,80},
   {30,7,1,113,202,190,80},
   {30,8,1,113,243,156,208},
   {30,9,1,114,28,123,80},
   {30,10,1,114,68,8,80},
   {30,11,1,114,108,230,208},
   {30,12,1,114,148,129,224},
   {31,1,1,114,189,96,96},
   {31,2,1,114,230,62,224},
   {31,3,1,115,11,40,224},
   {31,4,1,115,51,249,80},
   {31,5,1,115,91,134,80},
   {31,6,1,115,132,100,208},
   {31,7,1,115,171,241,208},
   {31,8,1,115,212,208,80},
   {31,9,1,115,253,174,208},
   {31,10,1,116,37,59,208},
   {31,11,1,116,78,26,80},
   {31,12,1,116,117,181,96},
   {32,1,1,116,158,147,224},
   {32,2,1,116,199,114,96},
   {32,3,1,116,237,173,224},
   {32,4,1,117,22,126,80},
   {32,5,1,117,62,11,80},
   {32,6,1,117,102,233,208},
   {32,7,1,117,142,118,208},
   {32,8,1,117,183,85,80},
   {32,9,1,117,224,51,208},
   {32,10,1,118,7,192,208},
   {32,11,1,118,48,159,80},
   {32,12,1,118,88,58,96},
   {33,1,1,118,129,24,224},
   {33,2,1,118,169,247,96},
   {33,3,1,118,206,225,96},
   {33,4,1,118,247,177,208},
   {33,5,1,119,31,62,208},
   {33,6,1,119,72,29,80},
   {33,7,1,119,111,170,80},
   {33,8,1,119,152,136,208},
   {33,9,1,119,193,103,80},
   {33,10,1,119,232,244,80},
   {33,11,1,120,17,210,208},
   {33,12,1,120,57,109,224},
   {34,1,1,120,98,76,96},
   {34,2,1,120,139,42,224},
   {34,3,1,120,176,20,224},
   {34,4,1,120,216,229,80},
   {34,5,1,121,0,114,80},
   {34,6,1,121,41,80,208},
   {34,7,1,121,80,221,208},
   {34,8,1,121,121,188,80},
   {34,9,1,121,162,154,208},
   {34,10,1,121,202,39,208},
   {34,11,1,121,243,6,80},
   {34,12,1,122,26,161,96},
   {35,1,1,122,67,127,224},
   {35,2,1,122,108,94,96},
   {35,3,1,122,145,72,96},
   {35,4,1,122,186,24,208},
   {35,5,1,122,225,165,208},
   {35,6,1,123,10,132,80},
   {35,7,1,123,50,17,80},
   {35,8,1,123,90,239,208},
   {35,9,1,123,131,206,80},
   {35,10,1,123,171,91,80},
   {35,11,1,123,212,57,208},
   {35,12,1,123,251,212,224},
   {36,1,1,124,36,179,96},
   {36,2,1,124,77,145,224},
   {36,3,1,124,115,205,96},
   {36,4,1,124,156,157,208},
   {36,5,1,124,196,42,208},
   {36,6,1,124,237,9,80},
   {36,7,1,125,20,150,80},
   {36,8,1,125,61,116,208},
   {36,9,1,125,102,83,80},
   {36,10,1,125,141,224,80},
   {36,11,1,125,182,190,208},
   {36,12,1,125,222,89,224},
   {37,1,1,126,7,56,96},
   {37,2,1,126,48,22,224},
   {37,3,1,126,85,0,224},
   {37,4,1,126,125,209,80},
   {37,5,1,126,165,94,80},
   {37,6,1,126,206,60,208},
   {37,7,1,126,245,201,208},
   {37,8,1,127,30,168,80},
   {37,9,1,127,71,134,208},
   {37,10,1,127,111,19,208},
   {37,11,1,127,151,242,80},
   {37,12,1,127,191,141,96}};

uint32_t epoch_fromclock() {
  // implicit input is in ds3231clock
  // for sec,min,hour,day,month,year
  //      0   1    2   3    4    5
  uint32_t R = 0; 
  uint8_t i,j;
  for (i=0; i<18*12; i++) {
    if (ds3231clock[5]==depoch[i][0]) break;
    }
  for (j=i; j<i+12; j++) {
    if (ds3231clock[4]==depoch[j][1]) break;
    } 
  // get epoch time at start of month
  R  = (uint32_t)depoch[j][3] << 24;
  R += (uint32_t)depoch[j][4] << 16;
  R += (uint32_t)depoch[j][5] << 8;
  R += (uint32_t)depoch[j][6];  
  R += 86400U * (uint32_t)(ds3231clock[3]-1);  // add seconds per day * day of month offset
  R += 3600 * (uint32_t)ds3231clock[2]; // add seconds per hour * hour
  R += 60 * (uint32_t)ds3231clock[1];   // seconds per minute * min
  R += (uint32_t)ds3231clock[0];
  return R;
  }

uint32_t loadword(uint8_t * p) {
  // load a word from a byte address, regardless of alignment
  uint32_t R = p[3];
  R |= ((uint32_t)p[2]) << 8;
  R |= ((uint32_t)p[1]) << 16;
  R |= ((uint32_t)p[0]) << 24;
  return R;
  }

void clock_fromepoch(uint32_t etime) {
  // implicit output is in ds3231clock
  // for sec,min,hour,day,month,year
  //      0   1    2   3    4    5
  uint8_t i;
  uint32_t r,s,t,w;
  for (i=0; i<18*12; i++) {
    w = loadword((uint8_t*)&depoch[i][3]);
    if (etime <= w) break;
    // if (etime > w)  continue;
    }
  if (etime < w) i--;
  // NRF_LOG_INFO("Hit %d vs %d",w,etime);
  ds3231clock[5] = depoch[i][0];  // copy year
  ds3231clock[4] = depoch[i][1];  // month
  w = loadword((uint8_t*)&depoch[i][3]);
  s = etime - w;  // second offset into month
  r = 1 + (s / 86400U); // whole days into month, but start at 1
  ds3231clock[3] = (uint8_t)r;
  t = s%86400U;   // second offset into day
  ds3231clock[2] = (uint8_t)(t / 3600); // calculate hour
  t = t%3600;     // second offset into hour
  ds3231clock[1] = (uint8_t)(t / 60);   // calculate minute
  ds3231clock[0] = (uint8_t)(t % 60);   // set second 
  // NRF_LOG_INFO("Computed year = %d month = %d day = %d",ds3231clock[5],ds3231clock[4],r);
  // NRF_LOG_INFO("  hour = %d minute = %d second = %d",ds3231clock[2],ds3231clock[1],
  //		  ds3231clock[0]);
  }

void ds3231_raw_normal() {
  // input: ds3231clock_raw (7 bytes) output: ds3231clock (6 bytes)
  // convert raw clock into non-BCD version 
  // output is in ds3231clock
  // for sec,min,hour,day,month,year
  //      0   1    2   3    4    5
  ds3231clock[0] = (ds3231clock_raw[0]>>4)*10 + (ds3231clock_raw[0]&0xf);
  ds3231clock[1] = (ds3231clock_raw[1]>>4)*10 + (ds3231clock_raw[1]&0xf);
  ds3231clock[2] = ((ds3231clock_raw[2]>>4)&0x3)*10 + (ds3231clock_raw[2]&0xf);
  ds3231clock[3] = (ds3231clock_raw[4]>>4)*10 + (ds3231clock_raw[4]&0xf);
  ds3231clock[4] = (ds3231clock_raw[5]>>4)*10 + (ds3231clock_raw[5]&0xf);
  ds3231clock[5] = (ds3231clock_raw[6]>>4)*10 + (ds3231clock_raw[6]&0xf);
  }
void ds3231_normal_raw() {
  // input: ds3231clock (6 bytes) output: ds3231clock_raw (7 bytes)
  // convert clock into BCD version 
  // output is in ds3231clock_raw
  // for sec,min,hour,weekday,day,month,year
  //      0   1    2    3      4    5    6
  ds3231clock_raw[0] = (ds3231clock[0]/10)<<4 | (ds3231clock[0]%10);
  ds3231clock_raw[1] = (ds3231clock[1]/10)<<4 | (ds3231clock[1]%10);
  ds3231clock_raw[2] = (ds3231clock[2]/10)<<4 | (ds3231clock[2]%10);
  ds3231clock_raw[4] = (ds3231clock[3]/10)<<4 | (ds3231clock[3]%10);
  ds3231clock_raw[5] = (ds3231clock[4]/10)<<4 | (ds3231clock[4]%10);
  ds3231clock_raw[6] = (ds3231clock[5]/10)<<4 | (ds3231clock[5]%10);
  }

void ds3231_read() { 
  ret_code_t err_code;
  uint32_t R;
  uint8_t reg[1] = {0};
  if (!i2c_setaddr(DS3231_ADDR)) return;
  err_code = i2c_tx(reg, sizeof(reg), true);
  APP_ERROR_CHECK(err_code);
  err_code = i2c_rx(ds3231clock_raw, sizeof(ds3231clock_raw));
  APP_ERROR_CHECK(err_code);
  ds3231_raw_normal();
  R = epoch_fromclock();
  nrf_atomic_u32_store(&clock,*(uint32_t *)&R);
  // NRF_LOG_INFO("clock read %d",R);
  // NRF_LOG_RAW_HEXDUMP_INFO(ds3231clock_raw,8);
  }

void ds3231_write() {
  ret_code_t err_code;
  uint8_t reg[8];
  reg[0] = 0; memcpy(&reg[1],ds3231clock_raw,7);  // register + values all in one call
  err_code = i2c_tx(reg, sizeof(reg), false);
  APP_ERROR_CHECK(err_code);
  }

uint32_t get_clock() {
  return nrf_atomic_u32_add(&clock,0); 
  }

void message_clock_set(void * parameter, uint16_t size) {
  // copy of clock data in clock_buffer
  fresh_epoch_clock  = (uint32_t)clock_buffer[3] << 24;
  fresh_epoch_clock += (uint32_t)clock_buffer[2] << 16;
  fresh_epoch_clock += (uint32_t)clock_buffer[1] << 8;
  fresh_epoch_clock += (uint32_t)clock_buffer[0];
  clock_fromepoch(fresh_epoch_clock);  // convert into ds3231clock
  ds3231_normal_raw();                 // convert/copy to ds3231clock_raw 
  twi_mode = 3;
  if (!i2c_setaddr(DS3231_ADDR)) return;
  ds3231_write();
  }

// handler called regularly for timer tick 
APP_TIMER_DEF(second_tick_id);
static void second_tick_handler(void * p_context) {
  uint32_t t = nrf_atomic_u32_add(&clock,1); // increment current clock and read
  if (t % 60 == 0) { 
    if (!i2c_setaddr(DS3231_ADDR)) return;
    ds3231_read();
    return;
    }
  auxiliary_tick_handler(t);
  }

void clockapi_init() {
  ret_code_t err_code;
  // NOTE: we assume that timers_init() has been done previously, before this call!
  err_code = app_timer_create(&second_tick_id,APP_TIMER_MODE_REPEATED,second_tick_handler);
  APP_ERROR_CHECK(err_code);
  err_code = app_timer_start(second_tick_id,APP_TIMER_TICKS(1000), NULL);
  APP_ERROR_CHECK(err_code);
  i2c_setaddr(DS3231_ADDR); // NOTE ERROR NOT TOLERATED HERE
  ds3231_read();
  }


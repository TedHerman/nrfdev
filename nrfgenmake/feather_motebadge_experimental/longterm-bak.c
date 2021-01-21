#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf_log.h"
#include "motebadge.h"

extern void pool_init();
extern bool pool_pop();
extern mote_received_t packetpool_spare; 
extern uint16_t local_mote_id;

// define memchunk pointer (memchunk_p) and a pointer array into it
memchunk_t __attribute__((section(".noinit"))) memchunk_p;
static uint8_t* p_memchunks[MEMCHUNKS_POOL];  // pointers into data section

// initialize the memchunk area and pointers
void mem_init(bool force) {
  uint32_t area_index = 0;
  char * fence = "fnce";
  if (force) memset(memchunk_p.memchunk_area,0,DATA_LENGTH_MAX*MEMCHUNKS_POOL);
  if (    memcmp(memchunk_p.prefence,fence,4) != 0
       || memcmp(memchunk_p.postfence,fence,4) != 0  ) {
     memcpy(memchunk_p.prefence,fence,4);
     memcpy(memchunk_p.postfence,fence,4);
     memchunk_p.memchunk_index = 0;
     memchunk_p.batch_offset = 0;
     }
  for (int32_t i=0; i<MEMCHUNKS_POOL; i++) {
     p_memchunks[i] = memchunk_p.memchunk_area+area_index;
     area_index += DATA_LENGTH_MAX;
     }
  }

// get sample of top rssi observations for current period and 
// save them into the memchunks pool
void mem_batch(uint32_t clock, mote_report_t * r) { 
  batch_digest_t * t;
  batch_t * s; 
  mote_received_t * v;
  mote_received_t * w;
  int size_needed = offsetof(batch_t,reports) + 6*sizeof(mote_received_t);
  r->num_reports = 0;
  r->mote_id = local_mote_id;
  r->time = clock;
  if (memchunk_p.memchunk_index >= MEMCHUNKS_POOL) return;
  if (memchunk_p.batch_offset + size_needed >= DATA_LENGTH_MAX) {
    memchunk_p.memchunk_index++;
    memchunk_p.batch_offset = offsetof(batch_digest_t,batches);
    } 
  if (memchunk_p.memchunk_index >= MEMCHUNKS_POOL) return;
  t = (batch_digest_t *)p_memchunks[memchunk_p.memchunk_index];
  s = (batch_t *)((uint8_t*)t + memchunk_p.batch_offset); 
  s->num_reports = 0;
  memcpy((uint8_t*)&s->time,(uint8_t*)&r->time,4);
  t->id = local_mote_id;  // could well be redundant, won't hurt 
  memchunk_p.batch_offset += offsetof(batch_t,reports);
  v = (mote_received_t *)s->reports;
  w = (mote_received_t *)r->reports;
  for (int i=0; i<6; i++) {
    pool_pop();  // attempt to get largest rssi value in heap
    if (packetpool_spare.rssi == 0) break; 
    v->mote_id = packetpool_spare.mote_id; 
    v->rssi = packetpool_spare.rssi;
    w->mote_id = packetpool_spare.mote_id; 
    w->rssi = packetpool_spare.rssi;
    s->num_reports++;
    r->num_reports++;
    v = (mote_received_t*)( sizeof(mote_received_t) + (uint8_t*) v );
    w = (mote_received_t*)( sizeof(mote_received_t) + (uint8_t*) w );
    memchunk_p.batch_offset += sizeof(mote_received_t);
    }
  pool_init();  // clear for next period of observations
  }

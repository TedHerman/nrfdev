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
memchunk_t memchunk_p;   // warning, this is around 720KB
static uint8_t* p_memchunks[MEMCHUNKS_POOL];  // pointers into data section

// initialize the memchunk area and pointers
void mem_init(bool force) {
  uint32_t area_index = 0;
  char * fence = "fnce";
  for (int i=0; i<MEMCHUNKS_POOL; i++) {
     batch_digest_t * b;
     p_memchunks[i] = memchunk_p.area+area_index;
     b = (batch_digest_t *) p_memchunks[i];
     if (force) {
	 uint8_t* q = p_memchunks[i];
	 for (int j=0; j<DATA_LENGTH_MAX; j++) *q++ = 0; 
         } 
     b->record_number = i;
     area_index += DATA_LENGTH_MAX;
     }
  if ( force || memcmp(memchunk_p.prefence,fence,4) != 0
       || memcmp(memchunk_p.postfence,fence,4) != 0  ) {
     memcpy(memchunk_p.prefence,fence,4);
     memcpy(memchunk_p.postfence,fence,4);
     memchunk_p.cursor = 0;
     memchunk_p.high_water = 0;
     memchunk_p.batch_offset = 0;
     memchunk_p.empty = true;
     }
  }

uint32_t mem_cursor() {
  return memchunk_p.cursor;
  }

void mem_restore() { 
  memchunk_p.cursor = memchunk_p.high_water;
  }

void mem_undo() {
  if (memchunk_p.cursor > 0) memchunk_p.cursor--;
  }

bool mem_empty() {
  return memchunk_p.empty;
  }

uint8_t* mem_access(uint16_t blocknum) {
  return p_memchunks[blocknum];  // trusting caller
  }

// record first/next block of data (size up to DATA_LENGTH_MAX)
// into p_memchunks[memchunk_p.cursor] and increment the
// cursor for future call 
void mem_record(uint8_t* p, uint8_t size) { 
  memchunk_p.empty = false;
  if (memchunk_p.cursor >= MEMCHUNKS_POOL) return;
  if (size >= DATA_LENGTH_MAX) size = DATA_LENGTH_MAX;
  memcpy(p_memchunks[memchunk_p.cursor++],p,size);
  if (memchunk_p.cursor > memchunk_p.high_water) memchunk_p.high_water = memchunk_p.cursor;
  }

// fetch first/next block of data (size assumed to be DATA_LENGTH_MAX)
// based on memchunk_p.cursor and increment cursor for the 
// next call; return NULL if the cursor is out of specified limit
uint8_t* mem_fetch() {
  uint8_t* p;
  int limit = memchunk_p.high_water + 1;
  if (limit > MEMCHUNKS_POOL) limit = MEMCHUNKS_POOL;
  // DEBUG HERE
limit = MEMCHUNKS_POOL;
  if (memchunk_p.cursor >= limit) return NULL;
  if (memchunk_p.empty) return NULL;
  p = p_memchunks[memchunk_p.cursor++];  
  return p;
  }

// reset memchunk_p.cursor to zero
void mem_rewind() { 
  memchunk_p.cursor = 0;
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
  memchunk_p.empty = false;
  if (memchunk_p.cursor >= MEMCHUNKS_POOL) return;
  if (memchunk_p.batch_offset + size_needed >= DATA_LENGTH_MAX) {
    memchunk_p.cursor++;
    if (memchunk_p.cursor > memchunk_p.high_water) memchunk_p.high_water = memchunk_p.cursor;
    memchunk_p.batch_offset = offsetof(batch_digest_t,batches);
    } 
  if (memchunk_p.cursor >= MEMCHUNKS_POOL) return;
  t = (batch_digest_t *)p_memchunks[memchunk_p.cursor];
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

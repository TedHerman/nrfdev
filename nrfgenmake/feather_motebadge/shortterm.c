#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>
#include "nrf_log.h"
#include "motebadge.h"

// this packet pool is for the recording period of accumulating
// packets from other badges
mote_received_t packetpool[PACKET_POOL_SIZE]; 
mote_received_t packetpool_spare;  // used as return value below
int packetpool_index;  // number of entries used in packetpool
void pool_init() { 
  memset((uint8_t*)packetpool,0,PACKET_POOL_SIZE*sizeof(mote_received_t));
  packetpool_index = 0;
  }
// swap two packetpool items if second > first, return true if swap done
bool poolswap(int i,int j) {
  mote_received_t k;
  // consider a mote_received_t which is all zero not worth swapping
  if (packetpool[j].rssi == 0 && packetpool[j].mote_id == 0) return false; 
  if (packetpool[j].rssi <= packetpool[i].rssi) return false;  // order by rssi value
  k.mote_id = packetpool[i].mote_id;
  k.rssi = packetpool[i].rssi;
  packetpool[i] = packetpool[j]; 
  packetpool[j] = k;
  return true;
  }
// insert new mote_received item into packetpool (as a max-heap)
void pool_insert(mote_received_t* p) {
  int child, parent; 
  child = packetpool_index;
  if (packetpool_index >= PACKET_POOL_SIZE) return;
  packetpool[child].mote_id = p->mote_id;
  packetpool[child].rssi = p->rssi;
  packetpool_index++;
  if (child < 1) return;  // base case is empty heap
  while (true) {
    parent = ((child+1)/2)-1;
    if (parent < 0 || !(parent < child)) return;
    if (!poolswap(parent,child)) return;
    child = parent;
    }
  return;
  }
// heap pop of pool into packetspare 
void pool_pop() {
  // claim: if heap is empty, then postcondition is
  // that packetpool_spare will have both id and rssi zero
  int root, childL, childR; 
  root = 0;
  packetpool_spare.mote_id = packetpool[0].mote_id; // that was easy
  packetpool_spare.rssi = packetpool[0].rssi; // that was easy
  // loop goal: reduce heap size by one, maintaining heap 
  // property that each node is larger than or equal than child values
  while (true) {
    // invariant of loop should be to replace root with largest child
    if (packetpool[root].rssi == 0) return;
    childL = 2*(root+1)-1;
    childR = 2*(root+1); 
    // case: exceed heap size (childR index > childL index)
    if (childL >= PACKET_POOL_SIZE) return; 
    // case: no right child, so promote left child value
    else if (childR >= PACKET_POOL_SIZE) {
	packetpool[root].mote_id = packetpool[childL].mote_id;
	packetpool[root].rssi = packetpool[childL].rssi;
	root = childL; 
        }
    // case: no left child, promote right child value
    else if (packetpool[childL].rssi == 0) {
	packetpool[root].mote_id = packetpool[childR].mote_id;
	packetpool[root].rssi = packetpool[childR].rssi;
	root = childR; 
        }
    // case: no right child, promote left child value
    else if (packetpool[childR].rssi == 0) {
	packetpool[root].mote_id = packetpool[childL].mote_id;
	packetpool[root].rssi = packetpool[childL].rssi;
	root = childL; 
        }
    // case: left child >= right child, promote left child
    // case: both left and right child have zero rssi
    else if ( packetpool[childL].rssi >= packetpool[childR].rssi ) {
	packetpool[root].mote_id = packetpool[childL].mote_id;
	packetpool[root].rssi = packetpool[childL].rssi;
	root = childL; 
	}
    // case: right child > left child, promote right child
    else {
	packetpool[root].mote_id = packetpool[childR].mote_id;
	packetpool[root].rssi = packetpool[childR].rssi;
	root = childR; 
        }
    }
  }

/*
 * Project particledemo
 * Description:
 * Author:
 * Date:
 */
#include "boron.h"

boronstate_t boronstate = {
  .fence = {0xaa,0xbb,0xcc,0xdd},
  .clock = 0   
  };

#define BPSQUEUESIZE 16
bps_reading_t readingQueue[BPSQUEUESIZE];
uint8_t readingQueueNum;
uint8_t readingQueueHead;


int counter = 0;
int numPublished = 0;
bool connected;
bool timesynced;
// int led1 = D0;
int led2 = D7;
CellularSignal rssi;

void extColorPat(uint8_t pattern) {
  switch (pattern) {
    case 0:  // 1 second, increasing, blue+green
      for (int i=50; i<100; i += 5) {
        analogWrite(A1,i,500);
        analogWrite(A2,i,500);
        delay(100);
        }
      break;
    case 1: // 1 second, decreasing, blue+green
      for (int i=100; i>50; i -= 5) {
        analogWrite(A1,i,500);
        analogWrite(A2,i,500);
        delay(100);
        }
    default:
      break;
    }
  }

void i2cWandler() {
  Serial.println("i2cWandler called");
  uint8_t buffer[32];
  memcpy(buffer,(uint8_t*)&boronstate,sizeof(boronstate));
  Wire.write(buffer,sizeof(boronstate));
  }

void i2cHandler(int amount) {
  Serial.printlnf("i2cHandler called with %d bytes",amount);
  uint8_t buffer[32];
  uint8_t cursor = 0;
  if (amount <= 0) return;
  if ((uint)amount > sizeof(buffer)) {
    while (amount > 0) { Wire.read(); amount--; }
    return;
    }
  while (cursor < amount) {
    buffer[cursor] = Wire.read();
    cursor++;
    }
  if (readingQueueNum < BPSQUEUESIZE-1) {
    uint8_t i = (readingQueueHead + readingQueueNum)%BPSQUEUESIZE;
    memcpy(&readingQueue[i],buffer,sizeof(bps_reading_t));
    readingQueueNum++;
    }; 
  }

// setup() runs once, when the device is first turned on.
void setup() {
  readingQueueNum = 0;
  readingQueueHead = 0;
  Serial.begin(115200);
  Serial.printlnf("Serial started");
  if (Cellular.connecting()) Serial.println("Cellular connecting");
  // Cellular.on();
  Serial.printlnf("Cellular on");
  BLE.off();
  // Cellular.connect();
  Serial.printlnf("Cellular connected");
  Particle.connect();
  waitUntil(Particle.connected);
  rssi = Cellular.RSSI();
  Wire.setSpeed(CLOCK_SPEED_100KHZ);
  Wire.stretchClock(true);
  Wire.begin(4);
  Wire.reset();
  Wire.onReceive(i2cHandler);
  Wire.onRequest(i2cWandler);
  Wire.reset();
  // pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  Particle.syncTime();
  waitUntil(Particle.syncTimeDone);
  boronstate.clock = Time.now();
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  }

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // post event if possible
  char databuf[122]; 
  memset(databuf,0,sizeof(databuf));
  if (Particle.connected() && readingQueueNum > 0) {
    uint8_t i = readingQueueHead;
    bool r;
    sprintf(databuf,"%d,%d,%d,%d,%d,%d",
      (int)boronstate.clock,(int)rssi.getStrength(),
      (int)readingQueue[i].clock,(int)readingQueue[i].systolic,
      (int)readingQueue[i].diastolic,(int)readingQueue[i].pulse);
    r = Particle.publish("boomerangtest",databuf,PRIVATE);
    if (!r) Serial.printlnf("publish response: failure",r); 
    else { 
      numPublished++;
      readingQueueNum--;
      readingQueueHead = (readingQueueHead+1) % BPSQUEUESIZE;
      Serial.printlnf("published reading %d",numPublished);
      }
    }

  // in each iteration, blink one led  
  // digitalWrite(led2, HIGH);
  extColorPat(0);
  extColorPat(1);
  boronstate.clock = Time.now();
  counter += 2;
  // Serial.printlnf("clock = %d",boronstate.clock);
  if (counter > 3600*3) {
    Particle.syncTime();
    Particle.publishVitals();
    counter = 0;
    }
  }
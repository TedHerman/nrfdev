#include "Particle.h"
#include "boron.h"
#include "PublishQueueAsyncRK.h"

SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

// use the command particle library add PublishQueueAsyncRK to add in the 
// the PublishQueueAsyncRK library

retained uint8_t publishQueueRetainedBuffer[1024];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));
uint8_t lastpostday;
uint8_t lastpostcount;

ApplicationWatchdog* wd;

boronstate_t boronstate = {
  .fence = {0xaa,0xbb,0xcc,0xdd},
  .clock = 0,
  .isdst = false,
  .connected = false,
  .capped = false,
  .clinic = false
  };

#define BPSQUEUESIZE 16
bps_reading_t readingQueue[BPSQUEUESIZE];
uint8_t readingQueueNum;
uint8_t readingQueueHead;

uint32_t morningreading;
uint32_t eveningreading;
uint32_t counter = 0;
int numPublished = 0;
bool timesynced;
// int led1 = D0;
int led2 = D7;
CellularSignal rssi;

void watchdogHandler() {
  System.reset();
  }

void extColorPat(uint8_t pattern) {
  switch (pattern) {
    case 0:  // 2.5 second, increasing, blue+green
      for (int i=0; i<=100; i+=2) {
        analogWrite(A0,0,500);
        analogWrite(A1,i,500);
        analogWrite(A2,i,500);
        delay(50);
        }
      break;
    case 1: // 2.5 second, decreasing, blue+green
      for (int i=100; i>=0; i-=2) {
        analogWrite(A0,0,500);
        analogWrite(A1,i,500);
        analogWrite(A2,i,500);
        delay(50);
        }
        break;
    case 2: // 2.5 second, increasing green+red
      for (int i=0; i<=100; i+=2) {
        int j = (4*i)/5;
        if (i>0 && j==0) j = 1;
        analogWrite(A0,i,500);  
        analogWrite(A1,j,500);
        analogWrite(A2,0,500);
        delay(50);
        }
      break;
     case 3: // 2.5 second, decreasing green+red
      for (int i=100; i>=0; i-=2) {
        int j = (4*i)/5;
        analogWrite(A0,i,500);  
        analogWrite(A1,j,500);
        analogWrite(A2,0,500);
        delay(50);
        }
      break;
    case 9: // 5 seconds, blinky blue
      analogWrite(A0,0,25);
      analogWrite(A1,0,25);
      analogWrite(A2,100,25);
      delay(5000);
      break;
    case 10: // purple
      analogWrite(A0,50,500);
      analogWrite(A1,0,500);
      analogWrite(A2,100,500);
      break;
    default:
      break;
    }
  }

void i2cWandler() {
  Serial.println("i2cWandler called");
  uint8_t buffer[32];
  boronstate.connected = Particle.connected();
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
  pinMode(A0,OUTPUT);
  pinMode(A1,OUTPUT);
  pinMode(A2,OUTPUT);
  readingQueueNum = 0;
  readingQueueHead = 0;
  Serial.begin(115200);
  Serial.printlnf("Serial started");
  BLE.off();
  Cellular.on();
  Cellular.connect();
  extColorPat(10);
  waitUntil(Cellular.ready);
  Particle.connect();
  waitUntil(Particle.connected);
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
  wd = new ApplicationWatchdog(300000, watchdogHandler, 1536);
  }

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // post event if possible
  char databuf[122]; 
  uint8_t curday = Time.day();
  // feed the watchdog, hopefully every minute
  wd->checkin(); 
  // in each iteration, ensure cloud connection
  boronstate.connected = Particle.connected();
  if (!boronstate.connected) {
    Cellular.on();
    Cellular.connect();
    extColorPat(10);
    waitUntil(Cellular.ready);
    Particle.connect();
    waitUntil(Particle.connected);
    }
  boronstate.isdst = Time.isDST(); // THIS IS BOGUS - SEE DOCS
  if (curday != lastpostday) {
    morningreading = 0;
    eveningreading = 0;
    lastpostcount = 0;
    }
  if (curday == lastpostday && lastpostcount > 5) {
    boronstate.capped = true;
    }
  else boronstate.capped = false;
  memset(databuf,0,sizeof(databuf));
  rssi = Cellular.RSSI();
  if (Particle.connected() && readingQueueNum > 0) {
    uint8_t i = readingQueueHead;
    sprintf(databuf,"%d,%d,%d,%d,%d,%d",
      (int)boronstate.clock,(int)rssi.getStrength(),
      (int)readingQueue[i].clock,(int)readingQueue[i].systolic,
      (int)readingQueue[i].diastolic,(int)readingQueue[i].pulse);
    publishQueue.publish("boomerangtest",databuf,PRIVATE,WITH_ACK);
    numPublished++;
    readingQueueNum--;
    readingQueueHead = (readingQueueHead+1) % BPSQUEUESIZE;
    lastpostday = curday;
    if (lastpostcount == 0 && morningreading == 0) 
      morningreading = Time.now();
    if (lastpostcount == 0 && morningreading > 0 
      && eveningreading == 0) 
      eveningreading = Time.now();
    lastpostcount++;
    Serial.printlnf("scheduled publish for reading %d",numPublished);
    extColorPat(9);
    }

  if (lastpostcount == 0) {
    extColorPat(2);
    extColorPat(3);
    }
  else {
    extColorPat(0);
    extColorPat(1);
    }
  boronstate.clock = Time.now();
  if (eveningreading == 0 
      && (boronstate.clock - morningreading > 8*60*60)) {
    lastpostcount = 0;  // refresh counter for afternoon
    }
  // Serial.printlnf("clock = %d",boronstate.clock);
  if (Time.second()-counter > 3600*3) {
    Particle.syncTime();
    Particle.publishVitals();
    counter = Time.second();
    }
  }

Wiring on the Particle Xenon:

for debugging on an FTDI module
  D9,D10 (uart)

for the SD card module which uses SPI, 
  D11,D12,D13,D14 which are named
  on the xenon-pinout diagram as 
  MI,MO,SCK,A5

for the DS3231 clock module which uses I2C,
  which is called "TWI" in Nordicland, 
  D0,D1 which are named SDA,SCL on the
  xenon-pinout diagram

For V3.3 and GND, both SD card module and
clock module need these, so a junction or
fan-out connection is needed.

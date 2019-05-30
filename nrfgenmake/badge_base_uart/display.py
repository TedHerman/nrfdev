#!/usr/bin/env python2.7

import serial

S = serial.Serial(baudrate=115200,port="/dev/ttyACM0",timeout=1.0) 

def getpacket():
  # initially scan for start of frame:
  while True:
    b = S.read(1)
    if len(b) < 1: return None
    if ord(b[0]) == 0x7E: break 
  # we have just read a 0x7E
  packet = list() 
  b = S.read(1) # check for special case
  if len(b) < 1: return None
  if ord(b[0]) == 0x7E: # empty frame means we got fooled
     b = S.read(1) # now *this* should be the real start
     if len(b) < 1: return None
  # b is the first character of the frame   
  # now after start of frame, scan for end of frame
  while True:
    if ord(b[0]) == 0x7D:  # special character check
       b = S.read(1) 
       if len(b) < 1: return None
       if ord(b[0]) not in (0x5E,0x5D): return None
       if ord(b[0]) == 0x5E: b = chr(0x7E)
       elif ord(b[0]) == 0x5D: b = chr(0x7D)
    elif ord(b[0]) == 0x7E:
       return list(map(ord,packet)) 
    else:
       packet.append(b[0])
       b = S.read(1) 
       if len(b) < 1: return None
    continue

def showraw(p):
  print list(map(hex,p))
    
def showpacket(p): 
  time = p[2]*256*256*256 + p[3]*256*256 + p[1]*256 + p[0]
  moteid = p[4]
  result = "Mote {0} at time {1}".format(moteid,time)
  numrep = p[5]
  replist = p[6:]
  for i in range(numrep):
     oid,rssi,replist = replist[0],replist[1],replist[2:]
     result += " saw {0} with rssi {1}".format(oid,rssi)
  print(result)

while True:
  T = getpacket()
  if T == None: continue
  showpacket(T)

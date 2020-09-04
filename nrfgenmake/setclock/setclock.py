import time, calendar, serial, sys

S = serial.Serial(baudrate=115200,port="/dev/ttyUSB0",timeout=1.0)
S.flush()

def attempt():
  utc = calendar.timegm(time.gmtime())
  setstr = "{0:08x}".format(utc) # gmtime -> epoch -> ascii unicode hex
  S.write(bytes(setstr,'utf-8'))
  accum = ''
  while True:
    b = S.read(1)
    if len(b) < 1: break 
    c = chr(ord(b))
    if c not in ">?\r\n": accum += c
  S.flush()
  return setstr == accum

def main():
  for i in range(3):
    if attempt():
      sys.stdout.write("Clock set to UTC\n")
      S.close()
      sys.exit(0)
      break
  sys.stdout.write("Clock could not be set\n")
  S.close()

main()

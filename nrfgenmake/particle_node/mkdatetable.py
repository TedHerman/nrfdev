import time, datetime

# NOTE - needs Python3

def molist():
  "make a list of datetime objects that are the start of all future months"
  dlist = list()
  for Yr in range(2020,2038):
    for Mo in range(1,13):
      dlist.append(datetime.datetime(Yr,Mo,1,tzinfo=datetime.timezone.utc))
  return dlist

def hexoflist(L):
  "make cryptic bytes list from list of datetime objects"
  "format is b'YYMMDDtttttttt' where tttttttt is epoch time"
  tlist = list()
  for D in L:
    B = bytearray(7)
    B[0] = D.year-2000
    B[1] = D.month
    B[2] = 1 
    C = "{0:08x}".format(int(D.strftime('%s')))
    for i in range(4): B[3+i] = eval("0x{0}".format(C[2*i:2*i+2])) 
    # debug with these
    # x = (256*256*256*B[-4])+(256*256*B[-3])+(256*B[-2])+B[-1]
    # print(datetime.datetime.fromtimestamp(x))
    tlist.append(B)
  return tlist

def formatC(L): 
  "produce C source to encode cryptic byte arrays"
  S = "{"
  for item in L:
    S += "   {" 
    for digit in item: S += "{0:d},".format(digit)
    S = S[:-1]+"},\n"
  S = S[:-2] + "};"
  print(S)

formatC(hexoflist(molist()))

     


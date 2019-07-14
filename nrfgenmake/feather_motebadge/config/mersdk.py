import os, sys, subprocess, pprint

with open("sdk_config.h") as Base, open("other_config.h") as Other:
    A = [ line for line in Base if line.startswith("#define") and "ENABLED" in line ]
    B = [ line for line in Other if line.startswith("#define") and "ENABLED" in line ]

Asets,Bsets = dict(),dict()

def entry(D,S):
  for line in S:
    R = line.strip().split()  
    if len(R) < 3: D[R[1]] = "?"
    else:          D[R[1]] = R[2]

entry(Asets,A)
entry(Bsets,B)

# which are only in Asets
for k in sorted(Asets.keys()):
  if k not in Bsets:
     sys.stdout.write("{0} uniquely in A\n".format(k)) 

# which are only in Bsets
for k in sorted(Bsets.keys()):
  if k not in Asets:
     sys.stdout.write("{0} uniquely in B\n".format(k)) 

# which disagree on values between Asets and Bsets
for k in sorted(list(set(Asets.keys()) & set(Bsets.keys()))):
  if Asets[k] != Bsets[k]:  
     sys.stdout.write("{0} disagrees between A and B\n".format(k)) 

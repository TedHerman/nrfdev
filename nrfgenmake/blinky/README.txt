This is an example of the "blank" (no softdevice) stack   
in a program that blinks LEDs. The main thing to look at
is the config subdirectory, which nrfgenmake reads to 
get Makefile.in, options.json, Makefile.posix, and the 
linker specification files (the ".ld" files).  

Two things you need to change: options.json and 
Makefile.posix. Change options.json by adding/removing
names of header (".h") files and C source files. For
both options.json and Makefile.posix, you will need 
to change the location of the Nordic software tree. 

When the changes are completed, go ahead and run 
nrfgenmake to create your Makefile. Then try "make" 
to build blinky.

The normal process for a project is to guess at the 
inclusions and exclusions to put in options.json. 

1. Start with a minimal set of includes and excludes.
2. If you can, copy the options.json from an working 
   existing project that resembles yours.
3. You might have to iterate through     
     nrfgenmake -> make -> edit options -> (repeat)
   quite a few times.
4. It can help to study examples in the Nordic tree.
5. Remember to try unix search commands:        
     find /opt/nordic -name somefile.h 
     egrep -r "nrf_device" /opt/nordic       
     egrep -r "nrf_device" /opt/nordic | more
     egrep -r "nrf_device" /opt/nordic > found.txt
   (these are just hypothetical examples)             

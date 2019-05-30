This is a project using open-source tools to develop applications for
Nordic nRFx based platforms. 

nrftool is a python script that uploads a file, like nrf51422.bin, 
using bus pirate, to an nRF51422 (tested with a BLE400 board). In 
principle, this can be extended to other nRF51 series modules, and
hopefully even to an nRF52 some day. Since nrftool depends on some
bus pirate support (a python library "pyBusPirateLite"), a requirement
to get this working is to download and install that dependency (or
just make a symbolic link).            

nrfgenmake has other python scripts; it reads 
files of a project and generates a Makefile 
suitable to GCC arm (tested with arm-none-eabi). The goal is to 
automate building new nRF52 projects.  This is very 
incomplete, only a work-in-progress. Some example projects are 
included in the nrfgenmake directory. The difficulty of automation
is due to the way configuration data are distributed in a project:
source files, SDK libraries, source, and header files; Makefile 
parameters; the sdk\_config.h file. When something doesn't compile,
or does compile but doesn't work, where is the problem? Could be 
the Makefile, could be the sdk\_config.h file, could be what was 
imported, and so it goes. 

Here are the files in nrfgenmake:
 * BLEclock - a program which combines some BLE functions with a clock,
written for the nRF51422 platform 
 * blinky - standard led hello world made complicated by using all
the Nordic conventions for application timer, application scheduling,
random number generation, low-power idling (but eventually you 
probably need to use these anyway)
 * hfclk - a slightly tweaked copy of 
SDK15.3/examples/ble\_peripheral/ble\_app\_hrs, trying to ask Softdevice 
to turn on the high frequency clock and get a response; but it fails
 * hrsmotetx - another modified copy of ble\_app\_hrs, which manages
to implement 802.15.4 transmissions (multiprotocol) along with BLE; it
depends on a modification to Nordic's 802.15.4 stack from github, just
a few changes to the rsch code.
 * motetx - a standalone 802.15.4 tranmission program which attempts 
to set up headers and payload for compatibility with mote-style packets 
 * octavrx - a standalone 802.15.4 receiving/logging program  
 * octavxx - a standalone 802.15.4 transmitting program  
 * mersdk - a crude python script to compare two sdk\_config.h files,
which turns out to be handy guessing at what options to set
 * nrfgenmake51 - the program which constructs a new Makefile, using files
inside the config directory: Makefile.in, Makefile.posix, options.json; also
sdk\_config.h, blank.ld or s110.ld (linker files). This version was created
some years ago for the nRF51832 or similar.
 * nrfgenmake52 - a newer version of nrfgenmake51, now for the nrf52840.

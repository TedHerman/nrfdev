All of these examples are based on a modified Nordic SDK.

The base is just SDK15.3; I put this in ~/Desktop/nRF52840/SDK15.3

There needs to be an environment variable NORDIC_BASE referring to the SDK.

I put these two lines at the end of my .bashrc
 
  export PATH=$PATH:/opt/nrfjprog
  export NORDIC_BASE=/home/herman/Desktop/nRF52840/SDK15.3

(You will need to install nrfjprog somewhere, I put in /opt, you can 
choose otherwise; I downloaded nrfjprog release 9.8.1 on my Linux system)

The SDK15.3 has to be modified:

  1. Remove the existing components/802_15_4 directory 
  2. Check out this modified 802.15.4 protocol stack:
     https://github.com/UI-tedherman/nRF-IEEE-802.15.4-radio-driver/tree/802BLEmulti
  3. Move the 802BLEmulti to SDK15.3/components/802.15.4 

------------------------------------------------------------------------------

Operation of examples.

The nrfgenmake directory has three commands and a ton of examples. The
examples use various boards, such as the Nordic NRF52840DK (big board)
that emulates a JLink SEGGER. Also I modified SDK15.3/components/boards 
to add new boards: Sparkfun's nRF52840 Mini, Adafruit's Feather nRF52840
Express, and Particle's nRF52840 Xenon. Look in SDKmods to see the changes,
mainly just adding other boards to boards.h 

The three commands are nrfgenmake51, nrfgenmake52, and mersdk -- they are 
Python2 scripts. Would be easy to upgrade to Python3, but for now, Python
means Python2. Forget about nrfgenmake51 - it is obsolete nowadays.

nrfgenmake52 is supposed to build a Makefile from information in source
files and a config directory. Suppose you are in the blinky directory. 
Then ../nrfgenmake52 should build the Makefile for you. The command looks
for config/Makefile.in, config/options.json, the Nordic SDK (as assumed
from environment variable NORDIC_BASE), and other files. You should modify
config/Makefile.in to suit your board with the -DBOARD_PCA10056 or whatever
(in two places). Then nrfgenmake52 will *try* to scan your source file(s),
guess from the includes what Nordic sources you need, supplemented by 
directives in config/options.json, to create the Makefile. If all is well,
you can just do "make", "make flash", and so on after that. 

mersdk is another command for debugging. Two very important files in 
the config directory for your project as the ld file (loader) and the 
sdk_config file, which the Makefile will use to specify lots of compile
options. The mersdk command takes two input sdk_config files, say yours
and one from the Nordic example projects (you have to specify the full
paths) and it generates a report of which ENABLED variables are different.

Debugging

You'll probaby want to have SEGGER software on your development system.
I downloaded all of that into /opt/SEGGER, then I have a symbolic link
from /usr/bin/JLinkEXE to /opt/SEGGER/JLink_V642e/JLinkExe (but you could
just set up your PATH is another idea). Similarly I set up JLinkRTTClient
in the same fashion.

Normally, after "make flash" completes, you program is running on the 
board. Then, in another shell, start "$ JLinkExe -if SWD" and connect.
Launch yet another shell and run JLinkRTTClient to interact with 
your program. For this to work, you need to plan ahead: set up the 
compile-time option NRF_LOG_BACKEND_RTT_ENABLED so that statements
like NRF_LOG_INFO("Hello World") will output to the JLinkEXE connection.

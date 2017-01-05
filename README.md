This is a project using open-source tools to develop applications for
Nordic nRFx based platforms. 

nrftool is a python script that uploads a file, like nrf51422.bin, 
using bus pirate, to an nRF51422 (tested with a BLE400 board). In 
principle, this can be extended to other nRF51 series modules, and
hopefully even to an nRF52 some day. Since nrftool depends on some
bus pirate support (a python library "pyBusPirateLite"), a requirement
to get this working is to download and install that dependency (or
just make a symbolic link).            

nrfgenmake is another python script; it's goal is to read the 
source files of a project and automatically generate a Makefile 
suitable to GCC arm (tested with arm-none-eabi). This is very 
incomplete, only a work-in-progress. Two example projects are 
included in the nrfgenmake directory.

PROJECT_NAME := pca10028

NORDIC_PATH = /arm/opt/nordic

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = $(NORDIC_PATH)/components/toolchain/gcc
include config/Makefile.posix

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
/arm/opt/nordic/components/ble/common/ble_srv_common.c \
/arm/opt/nordic/components/libraries/util/app_util_platform.c \
/arm/opt/nordic/components/ble/ble_advertising/ble_advertising.c \
/arm/opt/nordic/components/drivers_nrf/twi_master/nrf_drv_twi.c \
/arm/opt/nordic/components/ble/ble_services/ble_bas/ble_bas.c \
/arm/opt/nordic/components/drivers_nrf/delay/nrf_delay.c \
/arm/opt/nordic/components/drivers_nrf/uart/nrf_drv_uart.c \
/arm/opt/nordic/components/libraries/uart/retarget.c \
/arm/opt/nordic/components/libraries/util/app_error.c \
/arm/opt/nordic/components/toolchain/system_nrf51.c \
/arm/opt/nordic/components/libraries/util/nrf_assert.c \
/arm/opt/nordic/components/softdevice/common/softdevice_handler/softdevice_handler.c \
/arm/opt/nordic/components/libraries/trace/app_trace.c \
/arm/opt/nordic/components/ble/ble_services/ble_dis/ble_dis.c \
/arm/opt/nordic/components/ble/common/ble_advdata.c \
/arm/opt/nordic/components/libraries/button/app_button.c \
/arm/opt/nordic/components/libraries/timer/app_timer.c \
/arm/opt/nordic/components/drivers_nrf/pstorage/pstorage.c \
/arm/opt/nordic/components/libraries/uart/app_uart.c \
/arm/opt/nordic/components/drivers_nrf/common/nrf_drv_common.c \
/arm/opt/nordic/components/ble/device_manager/device_manager_peripheral.c \
/arm/opt/nordic/examples/bsp/bsp.c \
/arm/opt/nordic/components/ble/common/ble_conn_params.c \
/arm/opt/nordic/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
demo.c
#assembly files common to all targets
ASM_SOURCE_FILES  = $(NORDIC_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s

#includes common to all targets
INC_PATHS  = -Iconfig
INC_PATHS += -I/arm/opt/nordic/examples/bsp 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/pstorage 
INC_PATHS += -I/arm/opt/nordic/components/softdevice/s120/headers 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/common 
INC_PATHS += -I/arm/opt/nordic/components/libraries/util 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/gpiote 
INC_PATHS += -I/arm/opt/nordic/components/libraries/uart 
INC_PATHS += -I/arm/opt/nordic/components/ble/common 
INC_PATHS += -I/arm/opt/nordic/components/softdevice/common/softdevice_handler 
INC_PATHS += -I/arm/opt/nordic/components/toolchain/gcc 
INC_PATHS += -I/arm/opt/nordic/components/ble/ble_services/ble_dis 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/uart 
INC_PATHS += -I/arm/opt/nordic/components/ble/ble_advertising 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/hal 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/delay 
INC_PATHS += -I/arm/opt/nordic/components/libraries/button 
INC_PATHS += -I/arm/opt/nordic/components/ble/device_manager 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/config 
INC_PATHS += -I/arm/opt/nordic/components/drivers_nrf/twi_master 
INC_PATHS += -I/arm/opt/nordic/components/libraries/trace 
INC_PATHS += -I/arm/opt/nordic/components/ble/ble_services/ble_bas 
INC_PATHS += -I/arm/opt/nordic/components/toolchain 
INC_PATHS += -I/arm/opt/nordic/components/device 
INC_PATHS += -I/arm/opt/nordic/components/libraries/timer 

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DBOARD_PCA10028
CFLAGS += -DNRF51
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DS110
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -Werror -O3
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L$(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DNRF51
ASMFLAGS += -DSWI_DISABLE0
#default target - first one defined
default: clean nrflink

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422

#target for printing all targets
help:
	@echo following targets are available:
	@echo   flash	
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrflink: OUTPUT_FILENAME := nrf51422_xxaa
nrflink: LINKER_SCRIPT := config/s110.ld
nrflink: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.bin
	nrftool --address s110 --progfile $(OUTPUT_BINARY_DIRECTORY)/nrf51422_xxaa.bin program /dev/ttyUSB0

flash_softdevice:
	@echo Flashing: softdevice
	nrftool --address 0 --progfile /arm/opt/nordic/components/softdevice/s110/hex/s110_nrf51_8.0.0_softdevice.bin program /dev/ttyUSB0

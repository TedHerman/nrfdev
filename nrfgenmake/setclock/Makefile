PROJECT_NAME := pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := config/blank.ld

NORDIC_PATH = $(NORDIC_BASE)

TEMPLATE_PATH = $(NORDIC_PATH)/components/toolchain/gcc
include config/Makefile.posix

#source common to all targets
SRC_FILES += \
 $(NORDIC_PATH)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
 $(NORDIC_PATH)/components/libraries/bsp/bsp.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_str_formatter.c \
 $(NORDIC_PATH)/components/libraries/util/app_error.c \
 $(NORDIC_PATH)/components/libraries/experimental_section_vars/nrf_section_iter.c \
 $(NORDIC_PATH)/components/libraries/timer/app_timer.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_default_backends.c \
 $(NORDIC_PATH)/components/libraries/uart/app_uart_fifo.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf_format.c \
 $(NORDIC_PATH)/components/boards/boards.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_rtt.c \
 $(NORDIC_PATH)/components/libraries/memobj/nrf_memobj.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_twim.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT_printf.c \
 $(NORDIC_PATH)/components/libraries/balloc/nrf_balloc.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_uart.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_serial.c \
 $(NORDIC_PATH)/components/libraries/atomic/nrf_atomic.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_clock.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
 $(NORDIC_PATH)/components/libraries/ringbuf/nrf_ringbuf.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_rng.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_systick.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_rng.c \
 $(NORDIC_PATH)/modules/nrfx/soc/nrfx_atomic.c \
 $(NORDIC_PATH)/components/libraries/cli/nrf_cli.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_twi.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf.c \
 $(NORDIC_PATH)/components/libraries/button/app_button.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_twi.c \
 $(NORDIC_PATH)/components/libraries/util/app_util_platform.c \
 $(NORDIC_PATH)/components/libraries/strerror/nrf_strerror.c \
 $(NORDIC_PATH)/components/libraries/util/app_error_handler_gcc.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_frontend.c \
 $(NORDIC_PATH)/components/libraries/util/nrf_assert.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_clock.c \
 $(NORDIC_PATH)/components/libraries/scheduler/app_scheduler.c \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
 $(NORDIC_PATH)/components/libraries/fifo/app_fifo.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uarte.c \
 $(NORDIC_PATH)/components/libraries/queue/nrf_queue.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uart.c \
 $(NORDIC_PATH)/modules/nrfx/mdk/system_nrf52840.c \
 $(NORDIC_PATH)/components/softdevice/common/nrf_sdh.c \
 clockapi.c \
 app_error_weak.c \
setclock.c

#includes common to all targets
INC_FOLDERS += config \
 $(NORDIC_PATH)/components/toolchain/cmsis/include \
 $(NORDIC_PATH)/components/drivers_nrf/nrf_soc_nosd \
 $(NORDIC_PATH)/components/libraries/experimental_section_vars \
 $(NORDIC_PATH)/modules/nrfx/drivers/include \
 $(NORDIC_PATH)/modules/nrfx \
 $(NORDIC_PATH)/integration/nrfx \
 $(NORDIC_PATH)/components/boards \
 $(NORDIC_PATH)/modules/nrfx/mdk \
 $(NORDIC_PATH)/components/libraries/timer \
 $(NORDIC_PATH)/components/libraries/cli \
 $(NORDIC_PATH)/components/libraries/atomic \
 $(NORDIC_PATH)/components/libraries/mutex \
 $(NORDIC_PATH)/components/libraries/delay \
 $(NORDIC_PATH)/components/libraries/bootloader/dfu \
 $(NORDIC_PATH)/components/libraries/util \
 $(NORDIC_PATH)/modules/nrfx/soc \
 $(NORDIC_PATH)/components/libraries/queue \
 $(NORDIC_PATH)/components/libraries/log/src \
 $(NORDIC_PATH)/components/libraries/fds \
 $(NORDIC_PATH)/components/libraries/bsp \
 $(NORDIC_PATH)/components/libraries/log \
 $(NORDIC_PATH)/components/libraries/ringbuf \
 $(NORDIC_PATH)/components/libraries/memobj \
 $(NORDIC_PATH)/external/fnmatch \
 $(NORDIC_PATH)/external/segger_rtt \
 $(NORDIC_PATH)/components/libraries/uart \
 $(NORDIC_PATH)/components/softdevice/common \
 $(NORDIC_PATH)/components/libraries/button \
 $(NORDIC_PATH)/modules/nrfx/hal \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs \
 $(NORDIC_PATH)/components/libraries/fifo \
 $(NORDIC_PATH)/external/fprintf \
 $(NORDIC_PATH)/components/libraries/strerror \
 $(NORDIC_PATH)/integration/nrfx/legacy \
 $(NORDIC_PATH)/components/libraries/balloc \
 $(NORDIC_PATH)/components/libraries/experimental_section_vars \
 $(NORDIC_PATH)/components/libraries/scheduler \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt \

# Libraries common to all targets
LIB_FILES += \

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

#flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PARTICLE_XENON
# CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DDEBUG
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(NORDIC_PATH)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

# Assembler flags
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PARTICLE_XENON
# ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

#default target - first one defined
default: nrf52840_xxaa 

#target for printing all targets
help:
	@echo following targets are available:
	@echo           nrf52840_xxaa
	@echo           flash      - flashing binary

include $(TEMPLATE_PATH)/Makefile.common

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

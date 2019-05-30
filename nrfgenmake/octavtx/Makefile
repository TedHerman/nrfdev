PROJECT_NAME := pca10056
TARGETS      := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := config/blank.ld

NORDIC_PATH = /home/herman/Desktop/nRF52840/SDK15.3

TEMPLATE_PATH = $(NORDIC_PATH)/components/toolchain/gcc
include config/Makefile.posix

#source common to all targets
SRC_FILES += \
 $(NORDIC_PATH)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_notification_direct.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_request_direct.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_str_formatter.c \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
 $(NORDIC_PATH)/components/libraries/util/app_error.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_priority_drop_direct.c \
 $(NORDIC_PATH)/components/libraries/timer/app_timer.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_default_backends.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_uart.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/temperature/nrf_802154_temperature_none.c \
 $(NORDIC_PATH)/components/libraries/button/app_button.c \
 $(NORDIC_PATH)/components/boards/boards.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_core_hooks.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_rtt.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/hp_timer/nrf_802154_hp_timer.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_rssi.c \
 $(NORDIC_PATH)/components/libraries/memobj/nrf_memobj.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_core.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT_printf.c \
 $(NORDIC_PATH)/components/libraries/balloc/nrf_balloc.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_timer_coord.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_pib.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_rx_buffer.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/nrf_802154_rsch_crit_sect.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf_format.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uarte.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_debug.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_serial.c \
 $(NORDIC_PATH)/components/libraries/util/app_error_weak.c \
 $(NORDIC_PATH)/components/libraries/atomic/nrf_atomic.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/clock/nrf_802154_clock_nodrv.c \
 $(NORDIC_PATH)/components/libraries/uart/retarget.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_imm_ack_generator.c \
 $(NORDIC_PATH)/components/libraries/bsp/bsp.c \
 $(NORDIC_PATH)/components/libraries/uart/app_uart_fifo.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT.c \
 $(NORDIC_PATH)/components/libraries/ringbuf/nrf_ringbuf.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/nrf_802154_rsch.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_csma_ca.c \
 $(NORDIC_PATH)/components/libraries/cli/nrf_cli.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_critical_section.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_ack_data.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_frame_parser.c \
 $(NORDIC_PATH)/components/libraries/util/app_util_platform.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_ack_pending_bit.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_uart.c \
 $(NORDIC_PATH)/components/libraries/strerror/nrf_strerror.c \
 $(NORDIC_PATH)/components/libraries/util/app_error_handler_gcc.c \
 $(NORDIC_PATH)/components/drivers_nrf/radio_config/radio_config.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_revision.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_filter.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_gpiote.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_frontend.c \
 $(NORDIC_PATH)/components/libraries/util/nrf_assert.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_enh_ack_generator.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_precise_ack_timeout.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/lp_timer/nrf_802154_lp_timer_nodrv.c \
 $(NORDIC_PATH)/components/libraries/scheduler/app_scheduler.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_ack_generator.c \
 $(NORDIC_PATH)/components/libraries/fifo/app_fifo.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_delayed_trx.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uart.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/raal/single_phy/single_phy.c \
 $(NORDIC_PATH)/components/802.15.4/src/timer_scheduler/nrf_802154_timer_sched.c \
 $(NORDIC_PATH)/modules/nrfx/mdk/system_nrf52840.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_clock.c \
 $(NORDIC_PATH)/components/softdevice/common/nrf_sdh.c \
main.c
#includes common to all targets
INC_FOLDERS += config \
 $(NORDIC_PATH)/components/toolchain/cmsis/include \
 $(NORDIC_PATH)/components/drivers_nrf/nrf_soc_nosd \
 $(NORDIC_PATH)/modules/nrfx \
 $(NORDIC_PATH)/integration/nrfx \
 $(NORDIC_PATH)/modules/nrfx/drivers/include \
 $(NORDIC_PATH)/components/boards \
 $(NORDIC_PATH)/modules/nrfx/mdk \
 $(NORDIC_PATH)/components/libraries/timer \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/raal \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features \
 $(NORDIC_PATH)/components/libraries/cli \
 $(NORDIC_PATH)/components/libraries/atomic \
 $(NORDIC_PATH)/components/libraries/mutex \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt \
 $(NORDIC_PATH)/components/libraries/delay \
 $(NORDIC_PATH)/components/libraries/bootloader/dfu \
 $(NORDIC_PATH)/components/libraries/log \
 $(NORDIC_PATH)/components/libraries/util \
 $(NORDIC_PATH)/components/libraries/log/src \
 $(NORDIC_PATH)/components/802.15.4/src/rsch \
 $(NORDIC_PATH)/components/libraries/fds \
 $(NORDIC_PATH)/components/libraries/bsp \
 $(NORDIC_PATH)/components/802.15.4/src \
 $(NORDIC_PATH)/components/802.15.4/src/platform/lp_timer \
 $(NORDIC_PATH)/components/libraries/ringbuf \
 $(NORDIC_PATH)/components/libraries/memobj \
 $(NORDIC_PATH)/external/fnmatch \
 $(NORDIC_PATH)/components/802.15.4/src/platform/temperature \
 $(NORDIC_PATH)/external/segger_rtt \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator \
 $(NORDIC_PATH)/components/libraries/uart \
 $(NORDIC_PATH)/components/softdevice/common \
 $(NORDIC_PATH)/components/libraries/fifo \
 $(NORDIC_PATH)/modules/nrfx/hal \
 $(NORDIC_PATH)/components/802.15.4/src/platform/clock \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs \
 $(NORDIC_PATH)/modules/nrfx/drivers/include \
 $(NORDIC_PATH)/components/802.15.4/src/timer_scheduler \
 $(NORDIC_PATH)/external/fprintf \
 $(NORDIC_PATH)/components/libraries/button \
 $(NORDIC_PATH)/components/libraries/strerror \
 $(NORDIC_PATH)/integration/nrfx/legacy \
 $(NORDIC_PATH)/components/libraries/balloc \
 $(NORDIC_PATH)/components/802.15.4/src/platform/hp_timer \
 $(NORDIC_PATH)/components/libraries/experimental_section_vars \
 $(NORDIC_PATH)/components/libraries/scheduler \
 $(NORDIC_PATH)/components/drivers_nrf/radio_config \

# Libraries common to all targets
LIB_FILES += -lc -lnosys -lm -lstdc++

# Optimization flags
OPT = -O3 -g3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DBSP_UART_SUPPORT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums
# flags specific to 802.15.4
CFLAGS += -DRAAL_SINGLE_PHY=1
CFLAGS += -DNRF_CLOCK_ENABLED
CFLAGS += -Dnrf52840_tx

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DBSP_UART_SUPPORT
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -Dnrf52840_tx
ASMFLAGS += -DSWI_DISABLE0

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(NORDIC_PATH)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

nrf52840_xxaa: CFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: CFLAGS += -D__STACK_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__HEAP_SIZE=8192
nrf52840_xxaa: ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(NORDIC_PATH)/components/toolchain/gcc

include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall


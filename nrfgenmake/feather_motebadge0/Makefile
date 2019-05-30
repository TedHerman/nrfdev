PROJECT_NAME := pca10056
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := config/s140.ld

NORDIC_PATH = /home/herman/Desktop/nRF52840/SDK15.3

TEMPLATE_PATH = $(NORDIC_PATH)/components/toolchain/gcc
include config/Makefile.posix

#source common to all targets
SRC_FILES += \
 $(NORDIC_PATH)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_ecdh.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_default_backends.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_ack_pending_bit.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_error.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_rtt.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/clock/nrf_802154_clock_sdk.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_core.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/raal/softdevice/nrf_raal_softdevice.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_serial.c \
 $(NORDIC_PATH)/components/libraries/util/app_error_weak.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_hkdf.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_imm_ack_generator.c \
 $(NORDIC_PATH)/components/libraries/uart/app_uart.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_revision.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_rssi.c \
 $(NORDIC_PATH)/components/drivers_nrf/radio_config/radio_config.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_gpiote.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/lp_timer/nrf_802154_lp_timer_nodrv.c \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
 $(NORDIC_PATH)/components/libraries/fifo/app_fifo.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_delayed_trx.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_aead.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154.c \
 $(NORDIC_PATH)/components/libraries/queue/nrf_queue.c \
 $(NORDIC_PATH)/modules/nrfx/mdk/system_nrf52840.c \
 $(NORDIC_PATH)/components/softdevice/common/nrf_sdh.c \
 $(NORDIC_PATH)/components/libraries/fds/fds.c \
 $(NORDIC_PATH)/components/libraries/bsp/bsp.c \
 $(NORDIC_PATH)/components/libraries/experimental_section_vars/nrf_section_iter.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_ecdsa.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_chacha_poly_aead.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_mutex.c \
 $(NORDIC_PATH)/components/libraries/memobj/nrf_memobj.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_request_swi.c \
 $(NORDIC_PATH)/components/libraries/atomic_flags/nrf_atflags.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_aes_shared.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_hash.c \
 $(NORDIC_PATH)/components/libraries/fstorage/nrf_fstorage_sd.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_clock.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
 $(NORDIC_PATH)/components/libraries/ringbuf/nrf_ringbuf.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_eddsa.c \
 $(NORDIC_PATH)/components/libraries/fstorage/nrf_fstorage.c \
 $(NORDIC_PATH)/components/libraries/balloc/nrf_balloc.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_rng.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_backend_uart.c \
 $(NORDIC_PATH)/components/libraries/util/app_error_handler_gcc.c \
 $(NORDIC_PATH)/components/libraries/util/nrf_assert.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_clock.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_shared.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf_format.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_eddsa.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uarte.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_init.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_uart.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_ecdh.c \
 $(NORDIC_PATH)/components/802.15.4/src/timer_scheduler/nrf_802154_timer_sched.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_wdt.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_str_formatter.c \
 $(NORDIC_PATH)/components/libraries/util/app_error.c \
 $(NORDIC_PATH)/components/libraries/timer/app_timer.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_uart.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_core_hooks.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/hp_timer/nrf_802154_hp_timer.c \
 $(NORDIC_PATH)/components/ble/common/ble_conn_state.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_notification_swi.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT_printf.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_aes_aead.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_rx_buffer.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/nrf_802154_rsch_crit_sect.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_pib.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_debug.c \
 $(NORDIC_PATH)/components/libraries/atomic/nrf_atomic.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_init.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_aes.c \
 $(NORDIC_PATH)/components/ble/common/ble_conn_params.c \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/nrf_802154_rsch.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_csma_ca.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_ack_data.c \
 $(NORDIC_PATH)/components/softdevice/common/nrf_sdh_ble.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_shared.c \
 $(NORDIC_PATH)/components/boards/boards.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_filter.c \
 $(NORDIC_PATH)/components/libraries/log/src/nrf_log_frontend.c \
 $(NORDIC_PATH)/components/libraries/atomic_fifo/nrf_atfifo.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_ecc.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_hmac.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_hash.c \
 $(NORDIC_PATH)/components/ble/peer_manager/pm_buffer.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_swi.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_ecc.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_ecdsa.c \
 $(NORDIC_PATH)/components/ble/peer_manager/peer_data_storage.c \
 $(NORDIC_PATH)/components/libraries/uart/app_uart_fifo.c \
 $(NORDIC_PATH)/components/libraries/crypto/nrf_crypto_rng.c \
 $(NORDIC_PATH)/components/802.15.4/src/platform/temperature/nrf_802154_temperature_none.c \
 $(NORDIC_PATH)/components/libraries/button/app_button.c \
 $(NORDIC_PATH)/components/ble/peer_manager/peer_database.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_timer_coord.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_aes.c \
 $(NORDIC_PATH)/components/libraries/strerror/nrf_strerror.c \
 $(NORDIC_PATH)/components/libraries/uart/retarget.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_priority_drop_swi.c \
 $(NORDIC_PATH)/external/segger_rtt/SEGGER_RTT.c \
 $(NORDIC_PATH)/integration/nrfx/legacy/nrf_drv_rng.c \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/nrfx_rng.c \
 $(NORDIC_PATH)/external/fprintf/nrf_fprintf.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_frame_parser.c \
 $(NORDIC_PATH)/components/libraries/util/app_util_platform.c \
 $(NORDIC_PATH)/components/802.15.4/src/nrf_802154_critical_section.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/nrf_802154_precise_ack_timeout.c \
 $(NORDIC_PATH)/components/libraries/scheduler/app_scheduler.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_ack_generator.c \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310/cc310_backend_hmac.c \
 $(NORDIC_PATH)/components/libraries/cli/nrf_cli.c \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features/ack_generator/nrf_802154_enh_ack_generator.c \
 $(NORDIC_PATH)/components/ble/peer_manager/peer_id.c \
 $(NORDIC_PATH)/components/ble/common/ble_srv_common.c \
 motebadge0.c \
 idlist.c \
app_error_weak.c

#includes common to all targets
INC_FOLDERS += config \
 $(NORDIC_PATH)/components/toolchain/cmsis/include \
 $(NORDIC_PATH)/modules/nrfx \
 $(NORDIC_PATH)/integration/nrfx \
 $(NORDIC_PATH)/integration/nrfx/legacy \
 $(NORDIC_PATH)/external/segger_rtt \
 $(NORDIC_PATH)/modules/nrfx/drivers/include \
 $(NORDIC_PATH)/components/802.15.4/src \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/raal \
 $(NORDIC_PATH)/components/802.15.4/external/softdevice/s140/headers/nrf52 \
 $(NORDIC_PATH)/components/802.15.4/external/softdevice/s140/headers \
 $(NORDIC_PATH)/components/softdevice/s140/headers/nrf52 \
 $(NORDIC_PATH)/components/softdevice/s140/headers \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310 \
 $(NORDIC_PATH)/external/nrf_cc310/include \
 $(NORDIC_PATH)/external/cifra_AES128-EAX \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310_bl \
 $(NORDIC_PATH)/components/libraries/crypto/backend/mbedtls \
 $(NORDIC_PATH)/components/libraries/crypto/backend/oberon \
 $(NORDIC_PATH)/components/libraries/crypto/backend/micro_ecc \
 $(NORDIC_PATH)/components/libraries/crypto/backend/nrf_sw \
 $(NORDIC_PATH)/components/libraries/crypto/backend/nrf_hw \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cifra \
 $(NORDIC_PATH)/components/boards \
 $(NORDIC_PATH)/modules/nrfx/mdk \
 $(NORDIC_PATH)/components/libraries/timer \
 $(NORDIC_PATH)/components/802.15.4/src/platform/lp_timer \
 $(NORDIC_PATH)/components/802.15.4/src/mac_features \
 $(NORDIC_PATH)/components/802.15.4/src/rsch/raal/softdevice \
 $(NORDIC_PATH)/components/libraries/atomic_fifo \
 $(NORDIC_PATH)/components/libraries/crypto/backend/cc310 \
 $(NORDIC_PATH)/components/drivers_nrf/radio_config \
 $(NORDIC_PATH)/components/libraries/fstorage \
 $(NORDIC_PATH)/components/libraries/crypto \
 $(NORDIC_PATH)/components/libraries/mutex \
 $(NORDIC_PATH)/modules/nrfx/drivers/src/prs \
 $(NORDIC_PATH)/components/libraries/stack_info \
 $(NORDIC_PATH)/components/libraries/pwr_mgmt \
 $(NORDIC_PATH)/components/ble/common \
 $(NORDIC_PATH)/components/libraries/delay \
 $(NORDIC_PATH)/components/libraries/bootloader/dfu \
 $(NORDIC_PATH)/components/libraries/atomic_flags \
 $(NORDIC_PATH)/components/libraries/util \
 $(NORDIC_PATH)/external/nrf_cc310/include \
 $(NORDIC_PATH)/components/libraries/log/src \
 $(NORDIC_PATH)/components/802.15.4/src/rsch \
 $(NORDIC_PATH)/components/libraries/fds \
 $(NORDIC_PATH)/components/libraries/bsp \
 $(NORDIC_PATH)/components/802.15.4/src \
 $(NORDIC_PATH)/components/libraries/crc16 \
 $(NORDIC_PATH)/components/ble/peer_manager \
 $(NORDIC_PATH)/components/libraries/log \
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
 $(NORDIC_PATH)/components/libraries/cli \
 $(NORDIC_PATH)/components/libraries/queue \
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
 $(NORDIC_PATH)/components/libraries/atomic \

# Libraries common to all targets
LIB_FILES += \
 $(NORDIC_PATH)/external/nrf_cc310/lib/cortex-m4/hard-float/libnrf_cc310_0.9.12.a \
 $(NORDIC_PATH)/external/nrf_oberon/lib/cortex-m4/hard-float/liboberon_2.0.7.a \

# Optimization flags
#OPT = -O3 -g3
OPT = -O3
# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBOARD_FEATHER_NRF52840_EXPRESS
CFLAGS += -DDEBUG
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DMBEDTLS_CONFIG_FILE=\"nrf_crypto_mbedtls_config.h\"
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DRAAL_SOFTDEVICE=1
#CFLAGS += -DENABLE_DEBUG_LOG=1
CFLAGS += -DuECC_ENABLE_VLI_API=0
CFLAGS += -DuECC_OPTIMIZATION_LEVEL=3
CFLAGS += -DuECC_SQUARE_FUNC=0
CFLAGS += -DuECC_SUPPORT_COMPRESSED_POINT=0
CFLAGS += -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums -Wno-packed-bitfield-compat

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBOARD_FEATHER_NRF52840_EXPRESS
ASMFLAGS += -DDEBUG
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_CRYPTO_MAX_INSTANCE_COUNT=1
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DRAAL_SOFTDEVICE=1
ASMFLAGS += -DuECC_ENABLE_VLI_API=0
ASMFLAGS += -DuECC_OPTIMIZATION_LEVEL=3
ASMFLAGS += -DuECC_SQUARE_FUNC=0
ASMFLAGS += -DuECC_SUPPORT_COMPRESSED_POINT=0
ASMFLAGS += -DuECC_VLI_NATIVE_LITTLE_ENDIAN=1

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
LIB_FILES += -lc -lnosys -lm -lstdc++


.PHONY: default help

# Default target - first one defined
default: nrf52840_xxaa

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		nrf52840_xxaa
	@echo		flash_softdevice
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(NORDIC_PATH)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash flash_softdevice erase

# Flash the program
flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex
	nrfjprog -f nrf52 --program $(OUTPUT_DIRECTORY)/nrf52840_xxaa.hex --sectorerase
	nrfjprog -f nrf52 --reset

# Flash softdevice
flash_softdevice:
	@echo Flashing: s140_nrf52_6.1.1_softdevice.hex
	nrfjprog -f nrf52 --program $(NORDIC_PATH)/components/softdevice/s140/hex/s140_nrf52_6.1.1_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

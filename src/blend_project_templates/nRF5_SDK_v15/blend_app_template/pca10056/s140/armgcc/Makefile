PROJECT_NAME     := ble_app_uart_c_pca10056_s140
TARGETS          := nrf52840_xxaa
OUTPUT_DIRECTORY := _build

PROJ_ROOT := ../../../../../../..
ZIP_EXCLUDE = $(wildcard $(PROJ_ROOT)/sdk/*.zip)
SDK_DIR := $(filter-out $(ZIP_EXCLUDE), $(wildcard $(PROJ_ROOT)/sdk/nRF5_SDK_15*))
LOCAL_SRC_DIR := ../../..
LOCAL_SRCS := $(wildcard $(LOCAL_SRC_DIR)/*.c) 


$(OUTPUT_DIRECTORY)/nrf52840_xxaa.out: \
  LINKER_SCRIPT  := ble_app_uart_c_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(LOCAL_SRCS) \
  $(SDK_DIR)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_DIR)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_DIR)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_DIR)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_DIR)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_DIR)/components/libraries/log/src/nrf_log_str_formatter.c \
  $(SDK_DIR)/components/libraries/button/app_button.c \
  $(SDK_DIR)/components/libraries/util/app_error.c \
  $(SDK_DIR)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_DIR)/components/libraries/util/app_error_weak.c \
  $(SDK_DIR)/components/libraries/fifo/app_fifo.c \
  $(SDK_DIR)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_DIR)/components/libraries/timer/app_timer.c \
  $(SDK_DIR)/components/libraries/uart/app_uart_fifo.c \
  $(SDK_DIR)/components/libraries/util/app_util_platform.c \
  $(SDK_DIR)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_DIR)/components/libraries/util/nrf_assert.c \
  $(SDK_DIR)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_DIR)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_DIR)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_DIR)/external/fprintf/nrf_fprintf.c \
  $(SDK_DIR)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_DIR)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_DIR)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_DIR)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_DIR)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_DIR)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_DIR)/components/libraries/uart/retarget.c \
  $(SDK_DIR)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_DIR)/components/boards/boards.c \
  $(SDK_DIR)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_DIR)/integration/nrfx/legacy/nrf_drv_uart.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/nrfx_power_clock.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/nrfx_uart.c \
  $(SDK_DIR)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_DIR)/components/libraries/bsp/bsp.c \
  $(SDK_DIR)/components/libraries/bsp/bsp_btn_ble.c \
  $(SDK_DIR)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_DIR)/external/segger_rtt/SEGGER_RTT_Syscalls_GCC.c \
  $(SDK_DIR)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_DIR)/components/ble/common/ble_advdata.c \
  $(SDK_DIR)/components/ble/ble_db_discovery/ble_db_discovery.c \
  $(SDK_DIR)/components/ble/common/ble_srv_common.c \
  $(SDK_DIR)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_DIR)/components/ble/nrf_ble_scan/nrf_ble_scan.c \
  $(SDK_DIR)/external/utf_converter/utf.c \
  $(SDK_DIR)/components/ble/ble_services/ble_nus_c/ble_nus_c.c \
  $(SDK_DIR)/components/softdevice/common/nrf_sdh.c \
  $(SDK_DIR)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_DIR)/components/softdevice/common/nrf_sdh_soc.c \

# Include folders common to all targets
INC_FOLDERS += \
  ../config \
  $(SDK_DIR)/components/nfc/ndef/generic/message \
  $(SDK_DIR)/components/nfc/t2t_lib \
  $(SDK_DIR)/components/nfc/t4t_parser/hl_detection_procedure \
  $(SDK_DIR)/components/ble/ble_services/ble_ancs_c \
  $(SDK_DIR)/components/ble/ble_services/ble_ias_c \
  $(SDK_DIR)/components/libraries/pwm \
  $(SDK_DIR)/components/libraries/usbd/class/cdc/acm \
  $(SDK_DIR)/components/libraries/usbd/class/hid/generic \
  $(SDK_DIR)/components/libraries/usbd/class/msc \
  $(SDK_DIR)/components/libraries/usbd/class/hid \
  $(SDK_DIR)/modules/nrfx/hal \
  $(SDK_DIR)/components/nfc/ndef/conn_hand_parser/le_oob_rec_parser \
  $(SDK_DIR)/components/libraries/log \
  $(SDK_DIR)/components/ble/ble_services/ble_gls \
  $(SDK_DIR)/components/libraries/fstorage \
  $(SDK_DIR)/components/nfc/ndef/text \
  $(SDK_DIR)/components/libraries/mutex \
  $(SDK_DIR)/components/libraries/gfx \
  $(SDK_DIR)/components/libraries/bootloader/ble_dfu \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/common \
  $(SDK_DIR)/components/libraries/fifo \
  $(SDK_DIR)/components/boards \
  $(SDK_DIR)/components/nfc/ndef/generic/record \
  $(SDK_DIR)/components/nfc/t4t_parser/cc_file \
  $(SDK_DIR)/components/ble/ble_advertising \
  $(SDK_DIR)/external/utf_converter \
  $(SDK_DIR)/components/ble/ble_services/ble_bas_c \
  $(SDK_DIR)/modules/nrfx/drivers/include \
  $(SDK_DIR)/components/libraries/experimental_task_manager \
  $(SDK_DIR)/components/ble/ble_services/ble_hrs_c \
  $(SDK_DIR)/components/softdevice/s140/headers/nrf52 \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/le_oob_rec \
  $(SDK_DIR)/components/libraries/queue \
  $(SDK_DIR)/components/libraries/pwr_mgmt \
  $(SDK_DIR)/components/ble/ble_dtm \
  $(SDK_DIR)/components/toolchain/cmsis/include \
  $(SDK_DIR)/components/ble/ble_services/ble_rscs_c \
  $(SDK_DIR)/components/ble/common \
  $(SDK_DIR)/components/ble/ble_services/ble_lls \
  $(SDK_DIR)/components/libraries/bsp \
  $(SDK_DIR)/components/ble/ble_db_discovery \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/ac_rec \
  $(SDK_DIR)/components/ble/ble_services/ble_bas \
  $(SDK_DIR)/components/libraries/mpu \
  $(SDK_DIR)/components/libraries/experimental_section_vars \
  $(SDK_DIR)/components/ble/ble_services/ble_ans_c \
  $(SDK_DIR)/components/libraries/slip \
  $(SDK_DIR)/components/libraries/delay \
  $(SDK_DIR)/components/libraries/mem_manager \
  $(SDK_DIR)/components/libraries/csense_drv \
  $(SDK_DIR)/components/libraries/memobj \
  $(SDK_DIR)/components/ble/ble_services/ble_nus_c \
  $(SDK_DIR)/components/softdevice/common \
  $(SDK_DIR)/components/ble/ble_services/ble_ias \
  $(SDK_DIR)/components/libraries/usbd/class/hid/mouse \
  $(SDK_DIR)/components/libraries/ecc \
  $(SDK_DIR)/components/ble/nrf_ble_scan \
  $(SDK_DIR)/components/nfc/ndef/conn_hand_parser/ble_oob_advdata_parser \
  $(SDK_DIR)/components/ble/ble_services/ble_dfu \
  $(SDK_DIR)/external/fprintf \
  $(SDK_DIR)/components/libraries/svc \
  $(SDK_DIR)/components/libraries/atomic \
  $(SDK_DIR)/components \
  $(SDK_DIR)/components/libraries/scheduler \
  $(SDK_DIR)/components/libraries/cli \
  $(SDK_DIR)/components/ble/ble_services/ble_lbs \
  $(SDK_DIR)/components/ble/ble_services/ble_hts \
  $(SDK_DIR)/components/libraries/crc16 \
  $(SDK_DIR)/components/nfc/t4t_parser/apdu \
  $(SDK_DIR)/components/libraries/util \
  $(SDK_DIR)/components/libraries/usbd/class/cdc \
  $(SDK_DIR)/components/libraries/csense \
  $(SDK_DIR)/components/libraries/balloc \
  $(SDK_DIR)/components/libraries/low_power_pwm \
  $(SDK_DIR)/components/libraries/hardfault \
  $(SDK_DIR)/components/ble/ble_services/ble_cscs \
  $(SDK_DIR)/components/libraries/uart \
  $(SDK_DIR)/components/libraries/hci \
  $(SDK_DIR)/components/libraries/usbd/class/hid/kbd \
  $(SDK_DIR)/components/libraries/timer \
  $(SDK_DIR)/components/softdevice/s140/headers \
  $(SDK_DIR)/integration/nrfx \
  $(SDK_DIR)/components/nfc/t4t_parser/tlv \
  $(SDK_DIR)/components/libraries/sortlist \
  $(SDK_DIR)/components/libraries/spi_mngr \
  $(SDK_DIR)/components/libraries/led_softblink \
  $(SDK_DIR)/components/nfc/ndef/conn_hand_parser \
  $(SDK_DIR)/components/libraries/sdcard \
  $(SDK_DIR)/components/nfc/ndef/parser/record \
  $(SDK_DIR)/modules/nrfx/mdk \
  $(SDK_DIR)/components/ble/ble_services/ble_cts_c \
  $(SDK_DIR)/components/ble/ble_services/ble_nus \
  $(SDK_DIR)/components/libraries/twi_mngr \
  $(SDK_DIR)/components/ble/ble_services/ble_hids \
  $(SDK_DIR)/components/libraries/strerror \
  $(SDK_DIR)/components/libraries/crc32 \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/ble_oob_advdata \
  $(SDK_DIR)/components/nfc/t2t_parser \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/ble_pair_msg \
  $(SDK_DIR)/components/libraries/usbd/class/audio \
  $(SDK_DIR)/components/nfc/t4t_lib/hal_t4t \
  $(SDK_DIR)/components/nfc/t4t_lib \
  $(SDK_DIR)/components/ble/peer_manager \
  $(SDK_DIR)/components/drivers_nrf/usbd \
  $(SDK_DIR)/components/libraries/ringbuf \
  $(SDK_DIR)/components/ble/ble_services/ble_tps \
  $(SDK_DIR)/components/nfc/ndef/parser/message \
  $(SDK_DIR)/components/ble/ble_services/ble_dis \
  $(SDK_DIR)/components/nfc/ndef/uri \
  $(SDK_DIR)/components/ble/nrf_ble_gatt \
  $(SDK_DIR)/components/ble/nrf_ble_qwr \
  $(SDK_DIR)/components/libraries/gpiote \
  $(SDK_DIR)/components/libraries/button \
  $(SDK_DIR)/modules/nrfx \
  $(SDK_DIR)/components/libraries/twi_sensor \
  $(SDK_DIR)/integration/nrfx/legacy \
  $(SDK_DIR)/components/libraries/usbd \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/ep_oob_rec \
  $(SDK_DIR)/external/segger_rtt \
  $(SDK_DIR)/components/libraries/atomic_fifo \
  $(SDK_DIR)/components/ble/ble_services/ble_lbs_c \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/ble_pair_lib \
  $(SDK_DIR)/components/libraries/crypto \
  $(SDK_DIR)/components/ble/ble_racp \
  $(SDK_DIR)/components/libraries/fds \
  $(SDK_DIR)/components/nfc/ndef/launchapp \
  $(SDK_DIR)/components/ble/ble_services/ble_hrs \
  $(SDK_DIR)/components/ble/ble_services/ble_rscs \
  $(SDK_DIR)/components/nfc/ndef/connection_handover/hs_rec \
  $(SDK_DIR)/components/nfc/t2t_lib/hal_t2t \
  $(SDK_DIR)/components/nfc/ndef/conn_hand_parser/ac_rec_parser \
  $(SDK_DIR)/components/libraries/stack_guard \
  $(SDK_DIR)/components/libraries/log/src \

# Libraries common to all targets
LIB_FILES += \

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
CFLAGS += -DNRF_SD_BLE_API_VERSION=6
CFLAGS += -DS140
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DSWI_DISABLE0
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=6
ASMFLAGS += -DS140
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DSWI_DISABLE0

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_DIR)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
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
	@echo		flash_softdevice
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_DIR)/components/toolchain/gcc


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
	@echo Flashing: s140_nrf52_6.1.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_DIR)/components/softdevice/s140/hex/s140_nrf52_6.1.0_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset

erase:
	nrfjprog -f nrf52 --eraseall

SDK_CONFIG_FILE := ../config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_DIR)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)

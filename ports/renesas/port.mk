UF2_FAMILY_ID = 0x7be8976d
CROSS_COMPILE = arm-none-eabi-

FSP_RA = lib/mcu/renesas/fsp/ra/fsp
CMSIS_5 = lib/CMSIS_5

# Don't include options setting in .bin file since it create unnecessary large file due to padding
OBJCOPY_BIN_OPTION = --only-section .text --only-section .data --only-section .rodata --only-section .bss

# Default to port 0 fullspeed, board with port 1 highspeed should override this in board.mk
BOARD_TUD_RHPORT ?= 0

CFLAGS += \
	-flto \
	-DCFG_TUSB_MCU=OPT_MCU_RAXXX \
	-DBOARD_TUD_RHPORT=$(BOARD_TUD_RHPORT) \
	-Wno-error=undef \
	-Wno-error=strict-prototypes \
	-Wno-error=cast-align \
	-Wno-error=cast-qual \
	-Wno-error=unused-but-set-variable \
	-Wno-error=unused-variable \
	-nostdlib \
	-nostartfiles \
	-ffreestanding

ifeq ($(PORT), 1)
  CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_HIGH_SPEED
  $(info "Using PORT 1 HighSpeed")
else
  CFLAGS += -DBOARD_TUD_MAX_SPEED=OPT_MODE_FULL_SPEED
  $(info "Using PORT 0 FullSpeed")
endif

LDFLAGS_GCC += -specs=nosys.specs -specs=nano.specs

CFLAGS += -Os -flto
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += --warn-no-undef
CFLAGS += -Wno-unused-parameter

LDFLAGS += -Wl,--gc-sections -flto

SRC_C += \
	ports/renesas/boards.c \
	ports/renesas/board_flash.c \
	$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Source/startup.c \
	$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Source/system.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_clocks.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_common.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_delay.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_group_irq.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_guard.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_io.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_irq.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_register_protection.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_rom_registers.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_sbrk.c \
	$(FSP_RA)/src/bsp/mcu/all/bsp_security.c \
	$(FSP_RA)/src/r_ioport/r_ioport.c \
	$(FSP_RA)/src/r_sci_uart/r_sci_uart.c \
	$(FSP_RA)/src/r_flash_lp/r_flash_lp.c 

ifndef BUILD_NO_TINYUSB
SRC_C += lib/tinyusb/src/portable/renesas/rusb2/dcd_rusb2.c \
		 lib/tinyusb/src/portable/renesas/rusb2/hcd_rusb2.c \
		 lib/tinyusb/src/portable/renesas/rusb2/rusb2_common.c 
endif

INC += \
	$(TOP)/$(CMSIS_5)/CMSIS/Core/Include \
	$(TOP)/$(FSP_RA)/src/bsp/cmsis/Device/RENESAS/Include \
	$(TOP)/$(FSP_RA)/inc \
	$(TOP)/$(FSP_RA)/inc/api \
	$(TOP)/$(FSP_RA)/inc/instances \
  	$(TOP)/$(FSP_RA)/src/bsp/mcu/all \
	$(TOP)/$(FSP_RA)/src/bsp/mcu/$(MCU_VARIANT) \


LD_FILES = $(PORT_DIR)/linker/gcc/$(MCU_VARIANT).ld 

LDFLAGS += -L$(TOP)/$(PORT_DIR)/linker/gcc


$(info ${LD_FILES})


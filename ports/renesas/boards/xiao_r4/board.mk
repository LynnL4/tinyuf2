CPU_CORE = cortex-m4
MCU_VARIANT = ra4m1

CFLAGS += \
	-mthumb \
    -mcpu=cortex-m4 \
    -mfloat-abi=hard \
    -mfpu=fpv4-sp-d16 

INC += 	\
	$(TOP)/$(BOARD_DIR) \
	$(TOP)/$(BOARD_DIR)/fsp_cfg \
	$(TOP)/$(BOARD_DIR)/fsp_cfg/bsp \

# For flash-jlink target
JLINK_DEVICE = R7FA4M1AB

flash: flash-jlink

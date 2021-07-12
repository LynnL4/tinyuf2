CFLAGS += \
  -DSEEEDUINO_STAI \
  -DSTM32H725xx  \
  -DHSE_VALUE=25000000U \
  -DBOARD_DEVICE_RHPORT_NUM=1 

CFLAGS += -DBOARD_DEVICE_RHPORT_SPEED=OPT_MODE_FULL_SPEED

LD_FILES ?= ports/$(PORT)/boards/$(BOARD)/STM32H725AEIX_FLASH.ld

SRC_S += \
	$(ST_CMSIS)/Source/Templates/gcc/startup_stm32h725xx.s \
  

# For flash-jlink target
JLINK_DEVICE = stm32h725aeh

flash: flash-jlink
erase: erase-jlink

#flash: flash-stlink
#erase: erase-stlink

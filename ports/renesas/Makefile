# List of git submodules that is included as part of the UF2 version
GIT_SUBMODULES = tinyusb

# For flash-pyocd-bin target
FLASH_BIN_ADDR = $(UF2_$(MCU)_WRITE_ADDR)

include ../make.mk
include port.mk

# Port source
PORT_SRC_C += \
	$(addprefix $(CURRENT_PATH)/, $(wildcard *.c))

include ../rules.mk

#-------------- Self-update  --------------
self-update:
	@echo "not implemented yet"

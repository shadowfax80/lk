# Software-simulated I2C bus + EEPROM device.
# Implements LK's I2C API without hardware (for targets like qemu-virt-riscv).

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_SRCS += \
	$(LOCAL_DIR)/soft_i2c.c

include make/module.mk

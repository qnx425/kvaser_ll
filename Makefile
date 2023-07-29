# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
ifeq ($(EXTERNAL_CLOCK), 1)
	TARGET = kll_$(BOARD)_$(CRYSTAL_FREQ_MHZ)Mhz
else
	TARGET = kll_$(BOARD)
endif

#######################################
# paths
#######################################
# Build path
BUILD_DIR = build-$(BOARD)

######################################
# source
######################################
# C sources
C_SOURCES =                                                 \
Core/Src/main.c                                             \
Core/Src/system_stm32f1xx.c                                 \
Core/Src/stm32f1xx_hal_msp.c                                \
Core/Src/stm32f1xx_it.c                                     \
Core/Src/syscalls.c                                         \
Core/Src/sysmem.c                                           \
Src/dummy.c                                                 \
Src/qmain.c                                                 \
Src/usbd_conf.c                                             \
Src/usbd_desc.c                                             \
Src/xcan_can.c                                              \
Src/xcan_led.c                                              \
Src/xcan_protocol.c                                         \
Src/xcan_timestamp.c                                        \
Src/xcan_usb.c                                              \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c            \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c     \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c       \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c      \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c   \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c       \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c    \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c     \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c     \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c        \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c     \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c         \
STM32_USB_Device_Library/Core/Src/usbd_core.c               \
STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c             \
STM32_USB_Device_Library/Core/Src/usbd_ioreq.c              \
MCP251XFD/MCP251XFD.c                                       \
MCP251XFD/CRC/CRC16_CMS.c                                   \
mcp2518fd/mcp251xfd_can_config.c                            \
mcp2518fd/mcp251xfd_device_init.c                           \
mcp2518fd/mcp251xfd_driver_config.c                         \
mcp2518fd/mcp251xfd_driver_interface.c                      \
mcp2518fd/mcp251xfd_ff_config.c                             \
    
# ASM sources
ASM_SOURCES =  \
Core/Startup/startup_stm32f103c8tx.s


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m3

# fpu
# NONE for Cortex-M0/M0+/M3

# float-abi


# mcu
MCU = $(CPU) -mthumb $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F103xB \
-DNDEBUG \
-DEXTERNAL_CLOCK=$(EXTERNAL_CLOCK) \
-DBOARD_ID=$(BOARD_ID)

ifeq ($(EXTERNAL_CLOCK), 1)
	C_DEFS += -DHSE_VALUE=$(CRYSTAL_FREQ_MHZ)000000
endif

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES =  \
-ISrc \
-ICore/Inc \
-IDrivers/CMSIS/Include \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-ISTM32_USB_Device_Library/Core/Inc \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IMCP251XFD \
-IMCP251XFD/CRC \
-Imcp2518fd


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fno-common -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -std=gnu11 -Wall -Wextra -fno-common -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32F103C8TX_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

entree:
	$(MAKE) BOARD=entree BOARD_ID=1 EXTERNAL_CLOCK=0 DEBUG=0 OPT=-O2 elf hex bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))


ELF_TARGET = $(BUILD_DIR)/$(TARGET).elf
BIN_TARGET = $(BUILD_DIR)/$(TARGET).bin
HEX_TARGET = $(BUILD_DIR)/$(TARGET).hex

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

bin: $(BIN_TARGET)

elf: $(ELF_TARGET)

hex: $(HEX_TARGET)

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)*
  
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***

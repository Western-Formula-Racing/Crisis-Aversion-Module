##########################################################################################################################
# File automatically-generated by STM32forVSCode
##########################################################################################################################

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
TARGET = Crisis-Aversion-Module


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Og


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
Drivers/BSP/STM32U5xx_Nucleo/stm32u5xx_nucleo.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_adc_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_cortex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_dma_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_exti.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_fdcan.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_flash_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gpio.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_gtzc.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_i2c_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_icache.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_pwr_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_rcc_ex.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim.c \
Drivers/STM32U5xx_HAL_Driver/Src/stm32u5xx_hal_tim_ex.c \
Src/linked_list.c \
Src/main.c \
Src/stm32u5xx_hal_msp.c \
Src/stm32u5xx_it.c \
Src/system_stm32u5xx.c


CPP_SOURCES = \


# ASM sources
ASM_SOURCES =  \
STM32CubeIDE/Application/Startup/startup_stm32u575zitxq.s



#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
POSTFIX = "
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
GCC_PATH="c:/Users/Ya/AppData/Roaming/Code/User/globalStorage/bmd.stm32-for-vscode/@xpack-dev-tools/arm-none-eabi-gcc/12.2.1-1.2.1/.content/bin
ifdef GCC_PATH
CXX = $(GCC_PATH)/$(PREFIX)g++$(POSTFIX)
CC = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX)
AS = $(GCC_PATH)/$(PREFIX)gcc$(POSTFIX) -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy$(POSTFIX)
SZ = $(GCC_PATH)/$(PREFIX)size$(POSTFIX)
else
CXX = $(PREFIX)g++
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
CPU = -mcpu=cortex-m33

# fpu
FPU = -mfpu=fpv5-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DDEBUG \
-DSTM32U575xx \
-DUSE_HAL_DRIVER


# CXX defines
CXX_DEFS =  \
-DDEBUG \
-DSTM32U575xx \
-DUSE_HAL_DRIVER


# AS includes
AS_INCLUDES = \

# C includes
C_INCLUDES =  \
-IDrivers/BSP/STM32U5xx_Nucleo \
-IDrivers/CMSIS/Device/ST/STM32U5xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/STM32U5xx_HAL_Driver/Inc \
-IDrivers/STM32U5xx_HAL_Driver/Inc/Legacy \
-IInc



# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CXXFLAGS = $(MCU) $(CXX_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections -feliminate-unused-debug-types

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf -ggdb
CXXFLAGS += -g -gdwarf -ggdb
endif

# Add additional flags
CFLAGS += 
ASFLAGS += 
CXXFLAGS += -feliminate-unused-debug-types 

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = STM32CubeIDE/STM32U575ZITXQ_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = \


# Additional LD Flags from config file
ADDITIONALLDFLAGS = -specs=nosys.specs 

LDFLAGS = $(MCU) $(ADDITIONALLDFLAGS) -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin


#######################################
# build the application
#######################################
# list of cpp program objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))

# list of C objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# list of ASM program objects
# list of ASM program objects
UPPER_CASE_ASM_SOURCES = $(filter %.S,$(ASM_SOURCES))
LOWER_CASE_ASM_SOURCES = $(filter %.s,$(ASM_SOURCES))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(UPPER_CASE_ASM_SOURCES:.S=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(LOWER_CASE_ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.cpp STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cxx STM32Make.make | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cxx=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c STM32Make.make | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S STM32Make.make | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) STM32Make.make
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir $@

#######################################
# flash
#######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	"C:/USERS/YA/APPDATA/ROAMING/CODE/USER/GLOBALSTORAGE/BMD.STM32-FOR-VSCODE/@XPACK-DEV-TOOLS/OPENOCD/0.12.0-1.1/.CONTENT/BIN/OPENOCD.EXE" -f ./openocd.cfg -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

#######################################
# erase
#######################################
erase: $(BUILD_DIR)/$(TARGET).elf
	"C:/USERS/YA/APPDATA/ROAMING/CODE/USER/GLOBALSTORAGE/BMD.STM32-FOR-VSCODE/@XPACK-DEV-TOOLS/OPENOCD/0.12.0-1.1/.CONTENT/BIN/OPENOCD.EXE" -f ./openocd.cfg -c "init; reset halt; stm32u5x mass_erase 0; exit"

#######################################
# clean up
#######################################
clean:
	cmd /c rd /s /q $(BUILD_DIR)

#######################################
# custom makefile rules
#######################################


	
#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
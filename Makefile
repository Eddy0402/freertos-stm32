PROJECT = traffic

OUTDIR = build

EXECUTABLE = $(OUTDIR)/$(PROJECT).elf
BIN_IMAGE = $(OUTDIR)/$(PROJECT).bin
HEX_IMAGE = $(OUTDIR)/$(PROJECT).hex
PROJECT_LST = $(OUTDIR)/$(PROJECT).lst

TARGET = $(PROJECT)

# set the path to STM32F429I-Discovery firmware package
STDP ?= ../STM32F429I-Discovery_FW_V1.0.1

# Toolchain configurations
CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
OBJDUMP = $(CROSS_COMPILE)objdump
SIZE = $(CROSS_COMPILE)size

# Cortex-M4 implements the ARMv7E-M architecture
CPU = cortex-m4
CFLAGS = -mcpu=$(CPU) -march=armv7e-m -mtune=cortex-m4
CFLAGS += -mlittle-endian -mthumb
# Need study
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -O0

define get_library_path
    $(shell dirname $(shell $(CC) $(CFLAGS) -print-file-name=$(1)))
endef
LDFLAGS += -L $(call get_library_path,libc.a)
LDFLAGS += -L $(call get_library_path,libgcc.a)

# Basic configurations
CFLAGS += -g -std=c99
CFLAGS += -Wall

# Optimizations
CFLAGS += -g -std=c99 -O3 -ffast-math
CFLAGS += -ffunction-sections -fdata-sections
CFLAGS += -Wl,--gc-sections
CFLAGS += -fno-common
CFLAGS += --param max-inline-insns-single=1000

# specify STM32F429
CFLAGS += -DSTM32F429_439xx

# to run from FLASH
CFLAGS += -DVECT_TAB_FLASH
LDFLAGS += -T $(PWD)/CORTEX_M4F_STM32F4/stm32f429zi_flash.ld

# STM32F4xx_StdPeriph_Driver
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += -D"assert_param(expr)=((void)0)"

# STARTUP FILE
SRC += CORTEX_M4F_STM32F4/startup/startup_stm32f429_439xx.s \
       CORTEX_M4F_STM32F4/startup/system_stm32f4xx.s \

#My restart

SRCDIR = src \
         freertos/src \

INCDIR = include \
         freertos/include \
         CORTEX_M4F_STM32F4 \
         freertos/portable/GCC/ARM_CM4F \
         CORTEX_M4F_STM32F4/board \
         Utilities/STM32F429I-Discovery \
         Utilities/Libraries/CMSIS/Device/ST/STM32F4xx/Include \
         Utilities/Libraries/CMSIS/Include \
         Utilities/Libraries/STM32F4xx_StdPeriph_Driver/inc \

INCLUDES = $(addprefix -I,$(INCDIR))
SRC += $(wildcard $(addsuffix /*.c,$(SRCDIR))) \
       $(wildcard $(addsuffix /*.s,$(SRCDIR))) \
       freertos/portable/GCC/ARM_CM4F/port.c \
       freertos/portable/MemMang/heap_1.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma2d.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fmc.c \
       Utilities/Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c \
       Utilities/STM32F429I-Discovery/stm32f429i_discovery.c \
       Utilities/STM32F429I-Discovery/stm32f429i_discovery_sdram.c \
       Utilities/STM32F429I-Discovery/stm32f429i_discovery_lcd.c \
       Utilities/STM32F429I-Discovery/stm32f429i_discovery_ioe.c


OBJS := $(addprefix $(OUTDIR)/,$(patsubst %.s,%.o,$(SRC:.c=.o)))

# Traffic
CFLAGS += -DUSE_STDPERIPH_DRIVER
CFLAGS += $(INCLUDES)

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@
	$(OBJCOPY) -O ihex $^ $(HEX_IMAGE)
	$(OBJDUMP) -h -S -D $(EXECUTABLE) > $(PROJECT_LST)
	$(SIZE) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJS)
	$(LD) -o $@ $(OBJS) \
		--start-group $(LIBS) --end-group \
		$(LDFLAGS)

$(OUTDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "    CC      "$@
	@$(CC) $(CFLAGS) -c $< -o $@

$(OUTDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "    CC      "$@
	@$(CC) $(CFLAGS) -c $< -o $@

flash:
	st-flash write $(BIN_IMAGE) 0x8000000

openocd_flash:
	openocd \
	-f board/stm32f429discovery.cfg \
	-c "init" \
	-c "reset init" \
	-c "flash probe 0" \
	-c "flash info 0" \
	-c "flash write_image erase $(BIN_IMAGE)  0x08000000" \
	-c "reset run" -c shutdown

.PHONY: clean
clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)
	rm -rf $(HEX_IMAGE)
	rm -f $(OBJS)
	rm -f $(PROJECT_LST)

MAKDIR = mk
MAK = $(wildcard $(MAKDIR)/*.mk)

include $(MAK)

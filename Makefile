
# cross compilation tools
XTOOLS_DIR = /opt/gcc-arm-none-eabi-4_8-2014q2
X_LIBC_DIR = $(XTOOLS_DIR)/arm-none-eabi/lib/armv7e-m/fpu
X_LIBGCC_DIR = $(XTOOLS_DIR)/lib/gcc/arm-none-eabi/4.8.3/armv7e-m/fpu
X_CC = $(XTOOLS_DIR)/bin/arm-none-eabi-gcc
X_OBJCOPY = $(XTOOLS_DIR)/bin/arm-none-eabi-objcopy
X_AR = $(XTOOLS_DIR)/bin/arm-none-eabi-ar
X_LD = $(XTOOLS_DIR)/bin/arm-none-eabi-ld
X_GDB = $(XTOOLS_DIR)/bin/arm-none-eabi-gdb

OUTPUT = tinyg_stm32f4

# tinyg sources
TINYG_DIR = ./tinyg/TinyG-master/firmware/tinyg
SRC = $(TINYG_DIR)/canonical_machine.c \
      $(TINYG_DIR)/config.c \
      $(TINYG_DIR)/controller.c \
      $(TINYG_DIR)/cycle_homing.c \
      $(TINYG_DIR)/gcode_parser.c \
      $(TINYG_DIR)/help.c \
      $(TINYG_DIR)/json_parser.c \
      $(TINYG_DIR)/kinematics.c \
      $(TINYG_DIR)/main.c \
      $(TINYG_DIR)/plan_arc.c \
      $(TINYG_DIR)/plan_line.c \
      $(TINYG_DIR)/planner.c \
      $(TINYG_DIR)/report.c \
      $(TINYG_DIR)/spindle.c \
      $(TINYG_DIR)/network.c \
      $(TINYG_DIR)/util.c \
      $(TINYG_DIR)/test.c \
      $(TINYG_DIR)/gpio.c \
      $(TINYG_DIR)/pwm.c \
      $(TINYG_DIR)/stepper.c \
      $(TINYG_DIR)/system.c \

# hal sources
HAL_DIR = ./hal/src
SRC += $(HAL_DIR)/stm32f4xx_hal.c \
       $(HAL_DIR)/stm32f4xx_hal_rcc.c \
       $(HAL_DIR)/stm32f4xx_hal_cortex.c \
       $(HAL_DIR)/stm32f4xx_hal_gpio.c \
       $(HAL_DIR)/stm32f4xx_hal_pcd.c \
       $(HAL_DIR)/stm32f4xx_hal_dma.c \
       $(HAL_DIR)/stm32f4xx_ll_usb.c \
       $(HAL_DIR)/stm32f4xx_hal_tim.c \
       $(HAL_DIR)/stm32f4xx_hal_tim_ex.c \

# usb sources
#USB_DIR = ./usb
#SRC += $(USB_DIR)/core/usbd_core.c \
#       $(USB_DIR)/core/usbd_ctlreq.c \
#       $(USB_DIR)/core/usbd_ioreq.c \
#       $(USB_DIR)/cdc/usbd_cdc.c \

STM32F4_DIR = ./stm32f4
SRC += $(STM32F4_DIR)/system_stm32f4xx.c \
       $(STM32F4_DIR)/syscalls.c \
       $(STM32F4_DIR)/stm32f4xx_it.c \
       $(STM32F4_DIR)/stm32f4_regs.c \
       $(STM32F4_DIR)/delay.c \
       $(STM32F4_DIR)/xio.c \
       $(STM32F4_DIR)/eeprom.c \
       $(STM32F4_DIR)/rtc.c \
       $(STM32F4_DIR)/platform.c \

# board sources
#BOARD_DIR = ./board
#SRC += $(BOARD_DIR)/main.c \
#       $(BOARD_DIR)/usbd_conf.c \
#       $(BOARD_DIR)/usbd_desc.c \
#       $(BOARD_DIR)/usbd_cdc_interface.c \
#       $(BOARD_DIR)/gpio.c \
#       $(BOARD_DIR)/debounce.c \
#       $(BOARD_DIR)/timers.c \

OBJ = $(patsubst %.c, %.o, $(SRC))
OBJ += $(STM32F4_DIR)/start.o

# include files
INC += ./cmsis
INC += ./hal/inc
INC += $(STM32F4_DIR)
INC += $(TINYG_DIR)

#INC += $(USB_DIR)/core
#INC += $(USB_DIR)/cdc

INCLUDE = $(addprefix -I,$(INC))

# compiler flags
CFLAGS = -Wall
CFLAGS += -O
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -std=gnu99

# linker flags
LDSCRIPT = stm32f407vg_flash.ld
LDFLAGS = -T$(LDSCRIPT) -Wl,-Map,$(OUTPUT).map -Wl,--gc-sections

DEFINES = -DSTM32F407xx

.S.o:
	$(X_CC) $(INCLUDE) $(DEFINES) $(CFLAGS) -c $< -o $@
.c.o:
	$(X_CC) $(INCLUDE) $(DEFINES) $(CFLAGS) -c $< -o $@

all: tinyg_src $(OBJ)
	$(X_CC) $(CFLAGS) $(LDFLAGS) $(OBJ) -lm -o $(OUTPUT)
	$(X_OBJCOPY) -O binary $(OUTPUT) $(OUTPUT).bin

.PHONY: program
program: 
	st-flash write $(OUTPUT).bin 0x08000000

.PHONY: tinyg_src
tinyg_src:
	make -C tinyg all

clean:
	-rm $(OBJ)	
	-rm $(OUTPUT)
	-rm $(OUTPUT).map	
	-rm $(OUTPUT).bin	
	make -C tinyg clean


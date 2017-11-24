BOARD_TYPE          := 0x04
BOARD_REVISION      := 0x02
BOOTLOADER_VERSION  := 0x04
HW_TYPE             := 0x01

MCU                 := cortex-m3
CHIP				:= STM32F103RCT
BOARD				:= STM32103RC_CC_Rev1
MODEL				:= HD
MODEL_SUFFIX		:= _CC

OPENOCD_JTAG_CONFIG := foss-jtag.revb.cfg
OPENOCD_CONFIG      := stm32f1x.cfg

# Note: These must match the values in link_$(BOARD)_memory.ld
BL_BANK_BASE        := 0x08000000  # Start of bootloader flash
BL_BANK_SIZE        := 0x00003000  # Should include BD_INFO region
FW_BANK_BASE        := 0x08003000  # Start of firmware flash
FW_BANK_SIZE        := 0x0003D000  # Should include FW_DESC_SIZE (244kb)

FW_DESC_SIZE        := 0x00000064

OSCILLATOR_FREQ     :=   8000000
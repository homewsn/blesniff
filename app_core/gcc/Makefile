#--------------------------------------------------------------
#
# Copyright (c) 2025 Vladimir Alemasov
# All rights reserved
#
# This program and the accompanying materials are distributed under 
# the terms of GNU General Public License version 2 
# as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#--------------------------------------------------------------
# nRF5340 app_core_uart app_core_usbdev
#--------------------------------------------------------------

#--------------------------------------------------------------
# Target definitions
TARGETS = app_core_uart app_core_usbdev
DEF = -DNRF5340_XXAA -DNRF5340_XXAA_APPLICATION
DEF1 += $(DEF)
DEF2 += $(DEF)
DEF2 += -DCFG_TUSB_MCU=OPT_MCU_NRF5X

#--------------------------------------------------------------
# Paths
MAINDIR = ../src
CONFIGDIR = ../src/config
HALDIR = ../src/hal
MESSAGESDIR = ../src/messages
SHAREDBLEDIR = ../../shared/ble
TINYUSBDIR = ../src/usb/tinyusb
TINYUSBCDCDIR = ../src/usb/tinyusb/class/cdc
TINYUSBCOMMONDIR = ../src/usb/tinyusb/common
SHAREDCONFIGDIR = ../../shared/config
SHAREDHALDIR = ../../shared/hal
SHAREDMESSAGESDIR = ../../shared/messages
SHAREDPLATFORMDIR = ../../shared/platform
CMSISCOREDIR = ../../cmsis/core/include
CMSISDEVICEDIR = ../../cmsis/device

LINKERSCRIPTDIR = .

#--------------------------------------------------------------
# Include files directories
INCLDIRS += -I$(MAINDIR)
INCLDIRS += -I$(CONFIGDIR)
INCLDIRS += -I$(HALDIR)
INCLDIRS += -I$(MESSAGESDIR)
INCLDIRS += -I$(SHAREDBLEDIR)
INCLDIRS += -I$(TINYUSBDIR)
INCLDIRS += -I$(TINYUSBCOMMONDIR)
INCLDIRS += -I$(SHAREDCONFIGDIR)
INCLDIRS += -I$(SHAREDHALDIR)
INCLDIRS += -I$(SHAREDMESSAGESDIR)
INCLDIRS += -I$(SHAREDPLATFORMDIR)
INCLDIRS += -I$(CMSISCOREDIR)
INCLDIRS += -I$(CMSISDEVICEDIR)

#--------------------------------------------------------------
# Each source file must be added to the SOURCEFILES list
MAINSOURCEFILE1 = $(MAINDIR)/main_uart.c
SOURCEFILES1 += $(MAINSOURCEFILE1)
SOURCEFILES1 += $(MAINDIR)/serial.c
SOURCEFILES1 += $(MAINDIR)/uart.c
SOURCEFILES1 += $(HALDIR)/hal_perm.c
SOURCEFILES1 += $(HALDIR)/hal_reset.c
SOURCEFILES1 += $(HALDIR)/hal_uarte.c
SOURCEFILES1 += $(MESSAGESDIR)/msg_queue_ipc2serial.c
SOURCEFILES1 += $(MESSAGESDIR)/msg_queue_serial2ipc.c
SOURCEFILES1 += $(SHAREDHALDIR)/hal_ipc.c
SOURCEFILES1 += $(SHAREDHALDIR)/hal_mutex.c
SOURCEFILES1 += $(SHAREDMESSAGESDIR)/msg_queue.c
SOURCEFILES1 += $(SHAREDMESSAGESDIR)/msg_queue_app2net.c
SOURCEFILES1 += $(SHAREDMESSAGESDIR)/msg_queue_net2app.c
SOURCEFILES1 += $(SHAREDPLATFORMDIR)/platform.c
SOURCEFILES1 += $(CMSISDEVICEDIR)/system_nrf5340_application.c

MAINSOURCEFILE2 = $(MAINDIR)/main_usbdev.c
SOURCEFILES2 += $(MAINSOURCEFILE2)
SOURCEFILES2 += $(MAINDIR)/serial.c
SOURCEFILES2 += $(MAINDIR)/usb_descriptors.c
SOURCEFILES2 += $(MAINDIR)/usbdev.c
SOURCEFILES2 += $(HALDIR)/hal_perm.c
SOURCEFILES2 += $(HALDIR)/hal_reset.c
SOURCEFILES2 += $(HALDIR)/hal_usbd.c
SOURCEFILES2 += $(MESSAGESDIR)/msg_queue_ipc2serial.c
SOURCEFILES2 += $(MESSAGESDIR)/msg_queue_serial2ipc.c
SOURCEFILES2 += $(SHAREDHALDIR)/hal_ipc.c
SOURCEFILES2 += $(SHAREDHALDIR)/hal_mutex.c
SOURCEFILES2 += $(SHAREDMESSAGESDIR)/msg_queue.c
SOURCEFILES2 += $(SHAREDMESSAGESDIR)/msg_queue_app2net.c
SOURCEFILES2 += $(SHAREDMESSAGESDIR)/msg_queue_net2app.c
SOURCEFILES2 += $(SHAREDPLATFORMDIR)/platform.c
SOURCEFILES2 += $(CMSISDEVICEDIR)/system_nrf5340_application.c
SOURCEFILES2 += $(TINYUSBCDCDIR)/cdc_device.c
SOURCEFILES2 += $(TINYUSBDIR)/dcd_nrf5x.c
SOURCEFILES2 += $(TINYUSBDIR)/tusb.c
SOURCEFILES2 += $(TINYUSBCOMMONDIR)/tusb_fifo.c
SOURCEFILES2 += $(TINYUSBDIR)/usbc.c
SOURCEFILES2 += $(TINYUSBDIR)/usbd.c
SOURCEFILES2 += $(TINYUSBDIR)/usbd_control.c

SOURCEASMFILES += $(CMSISDEVICEDIR)/gcc_startup_nrf5340_application.S
SOURCEASMFILES1 += $(SOURCEASMFILES)
SOURCEASMFILES2 += $(SOURCEASMFILES)

LINKERSCRIPT = $(LINKERSCRIPTDIR)/nrf5340_xxaa_application.ld

#--------------------------------------------------------------
CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
#--------------------------------------------------------------
CFLAGS += -mcpu=cortex-m33+nodsp
CFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
CFLAGS += -mthumb -mlittle-endian
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fshort-enums -fomit-frame-pointer -fno-builtin
CFLAGS += -std=c11
CFLAGS += -Wall -Wdouble-promotion
CFLAGS += -Wno-main
# debug option
#CFLAGS += -O0 -g
#--------------------------------------------------------------
ASFLAGS += $(CFLAGS)
#--------------------------------------------------------------
LDFLAGS = -mcpu=cortex-m33+nodsp
LDFLAGS += -mfpu=fpv5-sp-d16 -mfloat-abi=hard
LDFLAGS += -mthumb -mabi=aapcs -mlittle-endian
LDFLAGS += --specs=nosys.specs
LDFLAGS += -T$(LINKERSCRIPT)
# disallow undefined symbols in object files
LDFLAGS += -Wl,-zdefs
LDFLAGS += -Wl,--warn-common
# disallow ".elf has a LOAD segment with RWX permissions" warning
LDFLAGS += -Wl,--no-warn-rwx-segments
# remove unused functions
LDFLAGS += -Wl,--gc-sections,--sort-section=alignment

#--------------------------------------------------------------
# Libraries
LIBS = -lgcc -lc
LIBDIRS =

#--------------------------------------------------------------
# Shell for Windows: cmd.exe
ifeq ($(OS),Windows_NT)
SHELL := cmd.exe
endif

#--------------------------------------------------------------
# The function creates the directory name for object files from the target name
# parameters:
# $(1) - target name
target2objdir = $(addsuffix _obj,$(1))
#--------------------------------------------------------------
#--------------------------------------------------------------
# The function creates the object filename from the source filename
# parameters:
# $(1) - directory name for object files
# $(2) - the c source filename(s) with (or without) path
c2obj = $(addprefix $(1)/,$(notdir $(patsubst %.c,%.o,$(2))))
#--------------------------------------------------------------
# The function creates the object filename from the source filename
# parameters:
# $(1) - directory name for object files
# $(2) - the asm source filename(s) with (or without) path
s2obj = $(addprefix $(1)/,$(notdir $(patsubst %.S,%.o,$(2))))
#--------------------------------------------------------------
#--------------------------------------------------------------
# The function creates an explicit rule based on a template common to all object files
# parameters:
# $(1) - object filename with path
# $(2) - c source filename with path
# $(3) - directory name for object files
# $(4) - c preprocessor definitions
define makecrule
$(1): $(2) | $(3)
	@echo $$<
	@$(CC) $(CFLAGS) $(4) $$< -o $$@ $(INCLDIRS) -c -MMD
endef
#--------------------------------------------------------------
# The function creates an explicit rule based on a template common to all object files
# parameters:
# $(1) - object filename with path
# $(2) - asm source filename with path
# $(3) - directory name for object files
define makesrule
$(1): $(2) | $(3)
	@echo $$<
	@$(AS) $(ASFLAGS) $$< -o $$@ -c
endef
#--------------------------------------------------------------
#--------------------------------------------------------------
# The function creates an explicit rule based on a template common to all targets
# parameters:
# $(1) - target name
# $(2) - directory name for object files
# $(3) - all object file names with path
define makerule_target
.PHONY: $(1)
$(1): $(1).hex $(1).bin
# Create directory for object files
$(2):
	@mkdir $$@
# Link firmware
$(1).elf: $(3)
	@echo ===========================
	@echo Creating elf file: $$@
	@$(LD) $(LDFLAGS) -Xlinker -Map=$(1).map $$^ -o $$@ $(LIBDIRS) $(LIBS)
# Post-process the hex file for programmers which dislike gcc output elf format
$(1).hex: $(1).elf
	@echo Creating hex file: $$@
	@$(OBJCOPY) -O ihex $$< $$@
# Post-process the bin file for programmers which dislike gcc output elf format
$(1).bin: $(1).elf
	@echo Creating bin file: $$@
	@$(OBJCOPY) -O binary $$< $$@
	@echo ===========================
endef
#--------------------------------------------------------------
#--------------------------------------------------------------
# The function creates an explicit rule to clean target
# parameters:
# $(1) - directory names for object files
define makerule_clean
.PHONY: clean
clean:
ifeq ($(OS),Windows_NT)
# The shell must be cmd.exe
#	@echo $(SHELL)
	@for %%d in ($(1)) do @if exist %%d rmdir /s /q %%d
else
	@rm -rf $(1)
endif
endef
#--------------------------------------------------------------
#--------------------------------------------------------------
# Additional functions
get_target_name = $(word $(1),$(TARGETS))
get_object_dir_name = $(call target2objdir,$(call get_target_name,$(1)))
get_object_file_names = $(call c2obj,$(call target2objdir,$(word $(1),$(TARGETS))),$(SOURCEFILES$(1)))
get_asm_object_file_names = $(call s2obj,$(call target2objdir,$(word $(1),$(TARGETS))),$(SOURCEASMFILES$(1)))
get_all_object_file_names = $(call get_object_file_names,$(1)) $(call get_asm_object_file_names,$(1))
#--------------------------------------------------------------


.PHONY: all
all: $(TARGETS)

ifeq ($(OS),Windows_NT)
CNTLIST = $(shell powershell -Command "1..$(words $(TARGETS))")
else
CNTLIST = $(shell seq 1 $(words $(TARGETS)))
endif

define makerules
$(foreach src,$(SOURCEFILES$(1)),$(eval $(call makecrule,$(call c2obj,$(call get_object_dir_name,$(1)),$(src)),$(src),$(call get_object_dir_name,$(1)),$(DEF$(1)))))
$(foreach src,$(SOURCEASMFILES$(1)),$(eval $(call makesrule,$(call s2obj,$(call get_object_dir_name,$(1)),$(src)),$(src),$(call get_object_dir_name,$(1)))))
$(eval $(call makerule_target,$(call get_target_name,$(1)),$(call get_object_dir_name,$(1)),$(call get_all_object_file_names,$(1))))
# Include additional explicit dependencies without recipes from the compiler (*.d files in the object directories)
-include $(call get_object_dir_name,$(1))/*.d
endef

$(foreach cnt,$(CNTLIST),$(eval $(call makerules,$(cnt))))

get_object_dir_names = $(foreach cnt,$(CNTLIST),$(call get_object_dir_name,$(cnt)))
$(eval $(call makerule_clean,$(call get_object_dir_names)))

.PHONY: distclean
distclean: clean
ifeq ($(OS),Windows_NT)
# The shell must be cmd.exe
#	@echo $(SHELL)
	@for %%f in (*.hex *.elf *.bin *.map) do @if exist %%f del /q %%f
else
	@rm -f *.hex *.elf *.bin *.map
endif

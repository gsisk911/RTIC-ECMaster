TARGET := thumbv7em-none-eabihf
PACKAGE := teensy-rust-modbus-base
MCU := imxrt1062
SOFT_REBOOT_ARGS ?=

ECAT_BUS_XML ?= ethercat-conf.bohign.xml
ECAT_ESI_XML ?= Bohign_MS_ECAT_V2.5.xml

RELEASE_DIR := target/$(TARGET)/release
ELF := $(RELEASE_DIR)/$(PACKAGE)
HEX := $(ELF).hex

.PHONY: all clean half-clean build hex reboot flash soft-bootloader config

all: hex

clean:
	cargo clean

half-clean:
	rm -f "$(HEX)"

# Regenerate src/ethercat/config/generated.rs from the bus XML + vendor ESI.
# Run this after editing the XML, then commit the regenerated .rs (never edit
# the generated file by hand).
config:
	@python3 scripts/generate_ethercat_config.py --bus "$(ECAT_BUS_XML)" --esi "$(ECAT_ESI_XML)"

build:
	cargo build --release

hex: build
	rust-objcopy -O ihex "$(ELF)" "$(HEX)"
	@echo "HEX ready: $(HEX)"

reboot:
	@python3 tools/soft_reboot_teensy.py $(SOFT_REBOOT_ARGS)

bootloader:
	@python3 tools/soft_reboot_teensy.py --bootloader $(SOFT_REBOOT_ARGS) || true

flash: hex bootloader
	teensy_loader_cli -mmcu=$(MCU) -w "$(HEX)"

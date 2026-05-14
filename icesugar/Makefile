# iCE40 SPWM Motor Controller Build System
# Requires: yosys, nextpnr-ice40, icepack, icetime, iceprog, iverilog, vvp

DEVICE   = up5k
PACKAGE  = sg48
FREQ     = 85          # constraint target (PLL is 82.5 MHz)
PCF      = constraints/pinout.pcf
TOP      = top

SRC      = $(wildcard src/*.v)
SINE_HEX = src/sine_init.hex

# The brams variant infers a 4096-entry ROM and needs this init file
# at $readmemh time. Generated next to sine_init.hex.
COUNTER_HEX = src/counter_table.hex

# Default variant for `make all`. Override on the command line, e.g.:
#   make all VARIANT=TWIN
# or use the per-variant convenience targets (make top_pipe, etc.).
# PIPE was the head-to-head Fmax winner in the integrated 85 MHz run.
VARIANT ?= PIPE

# Synthesis & PnR
.PHONY: all clean timing prog sine counter_table \
        top_pipe top_brams \
        top_compare

all: build/$(TOP).bin

$(SINE_HEX): scripts/gen_sine_table.py
	python3 scripts/gen_sine_table.py > $(SINE_HEX)

sine: $(SINE_HEX)

# Counter ROM for the brams variant. Other variants don't depend on
# this, but it's cheap to leave the rule unconditional.
$(COUNTER_HEX): scripts/gen_counter_rom.py
	python3 scripts/gen_counter_rom.py src

counter_table: $(COUNTER_HEX)

# Generic build (single variant chosen by VARIANT=).
build/$(TOP).json: $(SRC) $(SINE_HEX) $(COUNTER_HEX)
	@mkdir -p build
	yosys -p "read_verilog -DVARIANT_$(VARIANT) $(SRC); synth_ice40 -dsp -top $(TOP) -json $@"

build/$(TOP).asc: build/$(TOP).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --json $< --pcf $(PCF) --asc $@

build/$(TOP).bin: build/$(TOP).asc
	icepack $< $@

timing: build/$(TOP).asc
	icetime -d $(DEVICE) $<

prog: build/$(TOP).bin
	iceprog $<

# --- Per-variant convenience targets ---
# Each builds the full top design with one specific pwm_phase_correct
# variant selected via -DVARIANT_*. nextpnr log is teed to
# build/top_<variant>.nextpnr.log so they can be compared side by side.

define VARIANT_BUILD_RULE
build/top_$(1).json: $(SRC) $(SINE_HEX) $(COUNTER_HEX)
	@mkdir -p build
	yosys -p "read_verilog -DVARIANT_$(2) $(SRC); synth_ice40 -dsp -top $(TOP) -json $$@" \
		2>&1 | tee build/top_$(1).yosys.log

build/top_$(1).asc: build/top_$(1).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $$< --pcf $(PCF) --asc $$@ \
		2>&1 | tee build/top_$(1).nextpnr.log

build/top_$(1).bin: build/top_$(1).asc
	icepack $$< $$@

top_$(1): build/top_$(1).bin
endef

$(eval $(call VARIANT_BUILD_RULE,pipe,PIPE))
$(eval $(call VARIANT_BUILD_RULE,brams,BRAMS))

top_compare: top_pipe top_brams
	@for v in pipe brams; do \
		echo "--- top_$$v ---"; \
		grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/top_$$v.nextpnr.log | head -10; \
	done

# Simulation targets
SIM_DIR = build/sim

sim_tdm: $(SINE_HEX)
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_spwm_tdm -I src \
		tb/tb_spwm_tdm.v src/spwm_tdm.v
	cd src && vvp ../$(SIM_DIR)/tb_spwm_tdm

sim_uart:
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_uart_rx -I src \
		tb/tb_uart_rx.v src/uart_rx.v
	vvp $(SIM_DIR)/tb_uart_rx

sim_top: $(SINE_HEX) $(COUNTER_HEX)
	@mkdir -p $(SIM_DIR)
	iverilog -DVARIANT_$(VARIANT) -o $(SIM_DIR)/tb_top -I src \
		tb/tb_top.v $(SRC)
	cd src && vvp ../$(SIM_DIR)/tb_top

clean:
	rm -rf build
	rm -f src/sine_init.hex src/counter_table.hex

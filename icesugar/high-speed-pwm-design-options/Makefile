# iCE40 SPWM Motor Controller Build System
# Requires: yosys, nextpnr-ice40, icepack, icetime, iceprog, iverilog, vvp

DEVICE   = up5k
PACKAGE  = sg48
FREQ     = 85
PCF      = constraints/pinout.pcf
# Default top for the generic build/all targets. The variant_compare
# pipeline builds both pipelined and twin independently regardless of
# this setting.
TOP      = pwm_phase_correct_pipelined

SRC      = $(wildcard src/*.v)
SINE_HEX = src/sine_init.hex

# Synthesis & PnR
.PHONY: all clean timing prog sine

all: build/$(TOP).bin

$(SINE_HEX): scripts/gen_sine_table.py
	python3 scripts/gen_sine_table.py > $(SINE_HEX)

sine: $(SINE_HEX)

build/$(TOP).json: $(SRC) $(SINE_HEX)
	@mkdir -p build
	yosys -p "read_verilog $(SRC); synth_ice40 -dsp -top $(TOP) -json $@"

build/$(TOP).asc: build/$(TOP).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) --json $< --pcf $(PCF) --asc $@

build/$(TOP).bin: build/$(TOP).asc
	icepack $< $@

timing: build/$(TOP).asc
	icetime -d $(DEVICE) $<

prog: build/$(TOP).bin
	iceprog $<

# Simulation targets
SIM_DIR = build/sim

sim_tdm: $(SINE_HEX)
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_spwm_tdm -I src \
		tb/tb_spwm_tdm.v src/spwm_tdm.v src/sine_lut.v
	cd src && vvp ../$(SIM_DIR)/tb_spwm_tdm

sim_pwm:
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_pwm_deadtime -I src \
		tb/tb_pwm_deadtime.v src/pwm_phase_correct.v src/deadtime.v
	vvp $(SIM_DIR)/tb_pwm_deadtime

sim_uart:
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_uart_rx -I src \
		tb/tb_uart_rx.v src/uart_rx.v
	vvp $(SIM_DIR)/tb_uart_rx

sim_top: $(SINE_HEX)
	@mkdir -p $(SIM_DIR)
	iverilog -o $(SIM_DIR)/tb_top -I src \
		tb/tb_top.v $(SRC)
	cd src && vvp ../$(SIM_DIR)/tb_top

# --- Counter-source variant comparison ---
# Three counter-source families × two dead-time placements:
#   asymmetric (dead-time placed beside duty):
#     pipe       : src/pwm_phase_correct_pipelined.v
#     twin       : src/pwm_phase_correct_twin.v
#     brams      : src/pwm_phase_correct_brams.v
#   symmetric (dead-time centered on duty, no `duty_X` register needed):
#     pipe_sym   : src/pwm_phase_correct_pipelined_sym.v
#     twin_sym   : src/pwm_phase_correct_twin_sym.v
#     brams_sym  : src/pwm_phase_correct_brams_sym.v
.PHONY: pipe twin brams pipe_sym twin_sym brams_sym variant_compare

build/pipe.json: src/pwm_phase_correct_pipelined.v
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_pipelined -json $@" \
		2>&1 | tee build/pipe.yosys.log

build/pipe.asc: build/pipe.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/pipe.nextpnr.log

pipe: build/pipe.asc

build/twin.json: src/pwm_phase_correct_twin.v
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_twin -json $@" \
		2>&1 | tee build/twin.yosys.log

build/twin.asc: build/twin.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/twin.nextpnr.log

twin: build/twin.asc

# brams variant needs the 4096×13 lookup table generated first.
build/counter_table.hex: scripts/gen_counter_rom.py
	@mkdir -p build
	python3 scripts/gen_counter_rom.py build

build/brams.json: src/pwm_phase_correct_brams.v build/counter_table.hex
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_brams -json $@" \
		2>&1 | tee build/brams.yosys.log

build/brams.asc: build/brams.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/brams.nextpnr.log

brams: build/brams.asc

# --- Symmetric-dead-time variants (duty±dt/2 only, no duty_X register) ---

build/pipe_sym.json: src/pwm_phase_correct_pipelined_sym.v
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_pipelined_sym -json $@" \
		2>&1 | tee build/pipe_sym.yosys.log

build/pipe_sym.asc: build/pipe_sym.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/pipe_sym.nextpnr.log

pipe_sym: build/pipe_sym.asc

build/twin_sym.json: src/pwm_phase_correct_twin_sym.v
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_twin_sym -json $@" \
		2>&1 | tee build/twin_sym.yosys.log

build/twin_sym.asc: build/twin_sym.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/twin_sym.nextpnr.log

twin_sym: build/twin_sym.asc

build/brams_sym.json: src/pwm_phase_correct_brams_sym.v build/counter_table.hex
	@mkdir -p build
	yosys -p "read_verilog $<; synth_ice40 -dsp -top pwm_phase_correct_brams_sym -json $@" \
		2>&1 | tee build/brams_sym.yosys.log

build/brams_sym.asc: build/brams_sym.json
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --freq $(FREQ) \
		--json $< --asc $@ \
		2>&1 | tee build/brams_sym.nextpnr.log

brams_sym: build/brams_sym.asc

variant_compare: pipe twin brams pipe_sym twin_sym brams_sym
	@echo "--- pipe (asymmetric, registered-passthrough) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/pipe.nextpnr.log       | head -10
	@echo "--- pipe_sym (symmetric, registered-passthrough) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/pipe_sym.nextpnr.log   | head -10
	@echo "--- twin (asymmetric, independent counters) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/twin.nextpnr.log       | head -10
	@echo "--- twin_sym (symmetric, independent counters) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/twin_sym.nextpnr.log   | head -10
	@echo "--- brams (asymmetric, BRAM lookup) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/brams.nextpnr.log      | head -10
	@echo "--- brams_sym (symmetric, BRAM lookup, no dir mux) ---"
	@grep -E 'Max frequency|ICESTORM_LC|ICESTORM_RAM|SB_GB|SB_IO ' build/brams_sym.nextpnr.log  | head -10

clean:
	rm -rf build
	rm -f src/sine_init.hex

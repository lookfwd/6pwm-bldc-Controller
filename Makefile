# iCE40 SPWM Motor Controller Build System
# Requires: yosys, nextpnr-ice40, icepack, icetime, iceprog, iverilog, vvp

DEVICE   = hx1k
PACKAGE  = tq144
PCF      = constraints/pinout.pcf
TOP      = top

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
	yosys -p "read_verilog $(SRC); synth_ice40 -top $(TOP) -json $@"

build/$(TOP).asc: build/$(TOP).json $(PCF)
	nextpnr-ice40 --$(DEVICE) --package $(PACKAGE) --json $< --pcf $(PCF) --asc $@

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

clean:
	rm -rf build
	rm -f src/sine_init.hex

# iCE40 Three-Phase SPWM Motor Controller

FPGA-based sinusoidal PWM (SPWM) controller for three-phase brushless motors, targeting the Lattice iCE40HX1K. Designed as the high-speed complement to an STM32G4 "brain" in a split-task architecture.

## System Architecture

```
  STM32G4 (Brain/AI)              iCE40 FPGA (High-Speed PWM)
 ┌─────────────────┐             ┌──────────────────────────────┐
 │  FOC / Control   │── UART ───▶│  UART RX                     │
 │  Algorithms      │  115200    │    ↓                         │
 │                  │            │  Command Parser              │
 │  ADC Sampling    │◀── Sync ──│    ↓ (shadow registers)      │
 │                  │   Pulse   │  NCO → Sine LUT → TDM Calc   │
 └─────────────────┘            │    ↓                         │
                                │  Phase-Correct PWM + Dead-Time│
                                │    ↓                         │
                                │  6 Gate Outputs (UH/UL/VH/VL/WH/WL)
                                └──────────────────────────────┘
```

**STM32G4** handles closed-loop control (FOC), current sensing, and high-level state decisions.
**iCE40 FPGA** handles deterministic, jitter-free PWM generation with dead-time insertion.

## Clocking

- **Input**: 12 MHz external crystal
- **PLL Output**: 82.5 MHz (via `SB_PLL40_CORE`, DIVR=0, DIVF=54, DIVQ=3)
- **PWM Frequency**: 82.5 MHz / (2 × 2048) ≈ 20.14 kHz (phase-correct up-down mode)
- **PWM Resolution**: 11-bit (2048 steps)

The ideal target of 81.920 MHz (for exact 20.000 kHz) is not achievable from a 12 MHz reference on iCE40. The 82.5 MHz selection preserves an exact 11-bit power-of-two counter (0–2047) at the cost of 0.7% frequency deviation — well within motor control tolerances.

### PLL Parameters

| Parameter      | Value       | Meaning                         |
|---------------|-------------|----------------------------------|
| DIVR          | 0           | Reference divider (÷1)          |
| DIVF          | 54          | Feedback divider (÷55)          |
| DIVQ          | 3           | Output divider (÷8)             |
| FILTER_RANGE  | 1           | PFD frequency range             |
| F_VCO         | 660 MHz     | 12 × 55 = 660 MHz               |
| F_OUT         | 82.5 MHz    | 660 / 8 = 82.5 MHz              |

## Module Descriptions

### `pll.v` — Clock Generator
Wraps `SB_PLL40_CORE` to multiply the 12 MHz input to 82.5 MHz.

### `pwm_phase_correct.v` — Phase-Correct PWM Counter
11-bit up-down counter (0 → 2047 → 0). Generates:
- `sync` pulse at counter == 0 (triggers TDM recalculation and shadow register swap)
- `direction` signal (0 = counting up, 1 = counting down)

### `sine_lut.v` — Sine Lookup Table (BRAM)
2048-entry × 16-bit synchronous ROM initialized from `sine_init.hex`.

**Encoding: Unsigned Offset Binary**
- `0x0000` = negative peak (−1.0)
- `0x8000` = zero crossing (0.0)
- `0xFFFF` = positive peak (+1.0)

This encoding eliminates signed arithmetic in the downstream multiply. The BRAM has **1-cycle read latency** — the TDM state machine accounts for this.

### `spwm_tdm.v` — TDM State Machine with Integrated NCO (Core)
A single shared multiplier calculates duty cycles for all three phases sequentially. This saves ~400 LCs compared to three parallel multipliers.

The 32-bit NCO phase accumulator is integrated directly into the TDM module rather than being a separate module. This is architecturally correct because the TDM sequentially accesses a single-port sine LUT — having a separate NCO output all three phases concurrently would defeat the TDM strategy. The accumulator advances in the IDLE state, then phase offsets for U (+0), V (+683), W (+1365) are computed sequentially as each phase accesses the LUT.

| Phase | Offset | Degrees |
|-------|--------|---------|
| U     | 0      | 0°      |
| V     | +683   | 119.94° |
| W     | +1365  | 239.91° |

The `phase_inc` register (set via UART) controls motor electrical frequency:
```
f_electrical = phase_inc × f_pwm / 2^32
```

**Computation**: `duty = (sine_value × amplitude + 0x1000) >> 13`
- 16-bit sine × 8-bit amplitude = 24-bit product
- Add `0x1000` (= 2^12) for rounding before truncation
- Right-shift by 13 to produce 11-bit duty cycle

#### State Machine (16 states, 16 clock cycles)

Each phase follows: **SET_ADDR → WAIT_BRAM → LOAD_MULT → WAIT_MULT → STORE**

```
State           Action                                          Bug Fix
─────────────────────────────────────────────────────────────────────────
IDLE            Wait for pwm_sync pulse
SET_ADDR_U      lut_addr ← phase_u
WAIT_BRAM_U     (1-cycle BRAM read latency)                     Fix #1
LOAD_MULT_U     mult_a ← lut_data; mult_b ← amplitude          Fix #2
WAIT_MULT_U     (1-cycle registered multiply pipeline)          Fix #3
STORE_U         duty_u ← (product + 0x1000) >> 13              Fix #4
SET_ADDR_V      lut_addr ← phase_v
WAIT_BRAM_V     (BRAM latency)                                  Fix #1
LOAD_MULT_V     mult_a ← lut_data; mult_b ← amplitude          Fix #2
WAIT_MULT_V     (multiply pipeline)                             Fix #3
STORE_V         duty_v ← (product + 0x1000) >> 13              Fix #4
SET_ADDR_W      lut_addr ← phase_w
WAIT_BRAM_W     (BRAM latency)                                  Fix #1
LOAD_MULT_W     mult_a ← lut_data; mult_b ← amplitude          Fix #2
WAIT_MULT_W     (multiply pipeline)                             Fix #3
STORE_W         duty_w ← (product + 0x1000) >> 13              Fix #4
```

**Timing**: 16 clocks × 12.1 ns = 194 ns per PWM cycle. The PWM period is ~49.6 µs, so TDM computation occupies only 0.39% of available time.

#### Bugs Fixed from Original Design

| # | Bug | Root Cause | Fix |
|---|-----|-----------|-----|
| 1 | BRAM pipeline hazard | `lut_data` read in same cycle as `lut_addr` set; synchronous BRAM has 1-cycle latency | Added WAIT_BRAM states |
| 2 | Missing multiplier inputs for V/W | Only phase U loaded `mult_a`/`mult_b`; V and W multiplied stale values | Explicit LOAD_MULT for every phase |
| 3 | Ambiguous `mult_ready` timing | `mult_ready` checked in same state as input load; no pipeline delay | Separate WAIT_MULT states with registered multiplier |
| 4 | Truncation without rounding | `result[23:13]` truncation introduces DC offset | Add `0x1000` (half-LSB) before `>> 13` |

### `deadtime.v` — Dead-Time Insertion with State Control (per half-bridge)
Handles OPEN/RUNNING/BRAKE gate logic natively — there is no output mux in `top.v`. This eliminates a shoot-through hazard: if an external mux switched from RUNNING to BRAKE while a high-side gate was active, forcing the low-side ON immediately would cause shoot-through. By integrating state control into the dead-time module, every gate transition (including state changes) passes through the dead-time guard.

- **OPEN**: both gates driven LOW
- **RUNNING**: SPWM with dead-time insertion on every transition
- **BRAKE**: high-side OFF, low-side ON — but the 0→1 transition on the low-side still waits `dead_time` clocks
- 8-bit dead-time counter → max 255 cycles × 12.1 ns ≈ 3.1 µs
- Default dead-time of 50 counts = 606 ns (typical for mid-range MOSFETs)
- Instantiated 3× (one per phase: U, V, W)

### `uart_rx.v` — UART Receiver
Standard 8N1 UART receiver at 115200 baud.
- Clock divider: round(82.5e6 / 115200) = 716 → actual baud 115,223 (0.02% error)
- Double-flop synchronizer on RX input for metastability protection
- Outputs: 8-bit `data` + 1-cycle `valid` pulse

### `cmd_parser.v` — Command Decoder & Shadow Registers
Decodes 5-byte UART packets and manages shadow registers for glitch-free parameter updates.

**Packet format**: `[CMD] [D3] [D2] [D1] [D0]`

| CMD  | Function        | Data                        |
|------|-----------------|-----------------------------|
| 0x01 | Set State       | D0: 0=OPEN, 1=RUNNING, 2=BRAKE |
| 0x02 | Set Speed       | D3..D0: 32-bit NCO phase increment |
| 0x03 | Set Amplitude   | D0: 8-bit amplitude (0–255)  |

**Shadow Register Architecture**: Incoming UART data writes to `next_*` registers. Values transfer to `active_*` outputs only when the PWM counter reaches zero (`pwm_sync`), ensuring all parameters update atomically at a safe point in the PWM cycle.

## Control States

| State   | High Gates | Low Gates | Motor Behavior |
|---------|-----------|-----------|----------------|
| OPEN    | All LOW   | All LOW   | Free-wheeling, zero torque |
| RUNNING | SPWM      | SPWM      | Controlled sinusoidal current |
| BRAKE   | All LOW   | All HIGH  | Dynamic braking (windings shorted) |

State transitions from the command parser are gated on `pwm_sync`. The `deadtime` modules enforce dead-time on every gate change — including state transitions — so even if `ctrl_state` changes mid-cycle, no shoot-through can occur.

## Resource Budget

| Resource | Estimate | iCE40HX1K Limit |
|----------|----------|------------------|
| Logic Cells | ~700–800 | 1280 |
| EBR (BRAM) | 1 block (sine LUT) | 16 blocks |
| PLL | 1 | 1 |
| I/O Pins | ~9 (6 gates + UART + clk + LED) | 96 |

The iCE40HX has no hard multiplier. The 16×8-bit multiply is synthesized in fabric (~150–200 LCs).

## Build & Synthesis

### Prerequisites
- [Yosys](https://github.com/YosysHQ/yosys) — synthesis
- [nextpnr-ice40](https://github.com/YosysHQ/nextpnr) — place and route
- [IceStorm](https://github.com/YosysHQ/icestorm) — bitstream tools (`icepack`, `icetime`, `iceprog`)
- [Icarus Verilog](https://github.com/steveicarus/iverilog) — simulation
- Python 3 — sine table generation

### Generate Sine Table
```bash
python3 scripts/gen_sine_table.py > src/sine_init.hex
```

### Simulate
```bash
make sim_tdm        # TDM state machine
make sim_pwm        # PWM + dead-time
make sim_uart       # UART receiver
make sim_top        # Full system
```

### Synthesize & Program
```bash
make all            # synth → pnr → bitstream
make timing         # icetime timing analysis (verify Fmax ≥ 82.5 MHz)
make prog           # upload to FPGA via iceprog
```

## Verification Checklist

- [ ] PWM counter produces triangular waveform (0 → 2047 → 0)
- [ ] `sync` pulse fires exactly at counter == 0
- [ ] TDM completes all 16 states within one PWM cycle
- [ ] Duty values match `(sin(phase) × amplitude + 0x1000) >> 13`
- [ ] Dead-time: both gates LOW for exactly `dead_time` clocks at every transition
- [ ] Dead-time: high and low gates NEVER simultaneously HIGH (including state transitions)
- [ ] RUNNING→BRAKE transition: high-side turns off, dead-time elapses, then low-side turns on
- [ ] UART correctly receives bytes at 115200 baud
- [ ] Shadow registers hold until `pwm_sync`, then swap atomically
- [ ] OPEN state: all 6 gates LOW
- [ ] BRAKE state: high gates LOW, low gates HIGH
- [ ] State transitions wait for `pwm_sync`
- [ ] `icetime` reports Fmax ≥ 82.5 MHz
- [ ] Total LC usage < 1000

## File Structure

```
fpga-controller/
├── Makefile
├── README.md
├── src/
│   ├── top.v
│   ├── pll.v
│   ├── uart_rx.v
│   ├── cmd_parser.v
│   ├── sine_lut.v
│   ├── spwm_tdm.v
│   ├── pwm_phase_correct.v
│   ├── deadtime.v
│   └── sine_init.hex
├── tb/
│   ├── tb_top.v
│   ├── tb_spwm_tdm.v
│   ├── tb_pwm_deadtime.v
│   └── tb_uart_rx.v
├── constraints/
│   └── pinout.pcf
└── scripts/
    └── gen_sine_table.py
```

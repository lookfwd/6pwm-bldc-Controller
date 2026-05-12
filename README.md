# iCE40 Three-Phase SPWM Motor Controller

FPGA-based sinusoidal PWM (SPWM) controller for three-phase brushless motors, targeting the Lattice iCE40UP5K-SG48. It's a high-speed 11-bit / 82.5 MHz design.

## System Architecture

```
  Processor (Brain/AI)             iCE40 FPGA (High-Speed PWM)
 ┌──────────────────┐            ┌──────────────────────────────┐
 │  FOC / Control   │── UART ───▶│  UART RX                     │
 │  Algorithms      │  115200    │    ↓                         │
 │                  │            │  Command Parser              │
 │                  │            │    ↓ (shadow registers)      │
 │                  │            │  NCO → Sine LUT → TDM Calc   │
 └──────────────────┘            │    ↓                         │
                                 │ Phase-Correct PWM + Dead-Time│
                                 │    ↓                         │
                                 │  6 Outputs (UH/L,VH/L,WH/L)  │
                                 └──────────────────────────────┘
```

**Processor** handles closed-loop control (FOC), current sensing, and high-level state decisions.
**iCE40 FPGA** handles deterministic, jitter-free PWM generation with dead-time insertion.

## Application and Waves

Using [Hilitand Motor Speed Controller, DC 5V-36V 15A 3-Phase Brushless Motor Speed Control CW CCW Reversible Switch](https://www.amazon.com/dp/B07K7LLYR7?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_9) for power bridge, which in turn uses the [JY01 brushless DC motor controller IC](https://www.insightcentral.net/attachments/jy01_v3-5_2018-english-pdf.83073/). I removed the JY01 and drove the gates from PMOD2 of [iCESugar v1.5](https://github.com/wuxx/icesugar/blob/master/README_en.md) ([schematic](https://github.com/wuxx/icesugar/blob/master/schematic/iCESugar-v1.5.pdf)).


[![](https://img.youtube.com/vi/0ncPehXqvqU/maxresdefault.jpg)](https://www.youtube.com/watch?v=0ncPehXqvqU)

## Clocking

- **Input**: 12 MHz external crystal
- **PLL Output**: 82.5 MHz (via `SB_PLL40_PAD`, DIVR=0, DIVF=54, DIVQ=3)
- **PWM Frequency**: 82.5 MHz / (2 × 2048) ≈ 20.14 kHz (phase-correct up-down mode). The ideal target of 81.920 MHz (for exact 20.000 kHz) is not achievable from a 12 MHz reference on iCE40.
- **PWM Resolution**: 11-bit (2048 steps)

## Module Description

Clock domains. There are two clock domains, the fast and a synchronous slow with 1:4 gearing.

### `pll.v` - Clock Generator
Wraps `SB_PLL40_PAD` to multiply the 12 MHz input to 82.5 MHz.

### `pwm_phase_correct_{pipelined,twin,brams}.v` - Phase-Correct PWM + Gate Generator
Each variant is a self-contained fast-domain block that walks a 11-bit triangle counter (0 → 2047 → 0) and emits the six gate signals with symmetric dead-time. The 3 variations are:
- **pipelined** — single up/down counter with registered passthrough copies of `addr` feeding each phase's compare lanes.
- **twin** — one incrementing and one decrementing counter, eliminating the shared up/down adder.
- **brams** — counter materialized as a 4096 × 12-bit BRAM lookup table.

All three present the same external interface (6 pre-saturated `duty_*_minus_dt_half` / `_plus_dt_half` thresholds plus `sync` and 6 gate outputs), so they're swappable via a single `-DVARIANT_*` synthesis define. See [Variant Selection & Synthesis Comparison](#variant-selection--synthesis-comparison) below for measured Fmax / utilization head-to-head.

We keep all three - especially the BRAM and the alternative ones, because timing closure and slack relies on things that aren't that well under our control i.e. the placement of units and the routing delay between them. It's worth examining which of the 3 gives best timing by running `make top_compare`.

### `pwm_gate_unit.v` — Variant-Selecting Wrapper
Sits between the slow-domain command logic and the fast-domain PWM variant. Owns:
1. Saturating `duty ± dt/2` arithmetic in 12-bit intermediate (slow-domain combinational).
2. `ctrl_state` → threshold encoding (OPEN, RUNNING, BRAKE).
3. `ifdef`-selected instantiation of `pwm_phase_correct_pipelined` / `_twin` / `_brams`.

State encoding into the threshold pair (`duty_*_minus_dt_half`, `duty_*_plus_dt_half`):

| State   | `_minus_dt_half` | `_plus_dt_half` | Effect on gates |
|---------|------------------|-----------------|-----------------|
| OPEN    | 0                | 2047            | high always 0; low fires only at the 2-cycle triangle peak ≈ both off |
| BRAKE   | 0                | 0               | high always 0; low always 1 |
| RUNNING | sat(duty − dt/2) | sat(duty + dt/2)| symmetric SPWM with `dt` dead-time |

The slow-domain `cmd_parser` holds `ctrl_state == OPEN` for one full PWM period after every state change as a shoot-through guard. Default `DEAD_TIME = 50` fast cycles ≈ 606 ns.

### `sine_lut.v` — Sine Lookup Table (BRAM)
2048-entry × 16-bit synchronous ROM initialized from `sine_init.hex`.

**Encoding: Unsigned Offset Binary**
- `0x0000` = negative peak (−1.0)
- `0x8000` = zero crossing (0.0)
- `0xFFFF` = positive peak (+1.0)

This encoding eliminates signed arithmetic in the downstream multiply. The BRAM has **1-cycle read latency** — the TDM state machine accounts for this.

### `spwm_tdm.v` — Time-Division Multiplexing. (TDM) State Machine with Integrated NCO (Core)
A single shared multiplier calculates duty cycles for all three phases sequentially.

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

State transitions from the command parser are gated on `pwm_sync` and additionally sandwiched by one full PWM period of OPEN before the new state takes effect. Symmetric dead-time is enforced on every gate edge by the threshold encoding inside `pwm_gate_unit.v`, so even mid-cycle `ctrl_state` changes can't produce shoot-through.

## Resource Budget

Integrated post-route numbers on iCE40UP5K-SG48 (default `VARIANT = PIPE`; see [Variant Selection & Synthesis Comparison](#variant-selection--synthesis-comparison) for the other two):

| Resource     | Used | UP5K Limit | Notes |
|--------------|-----:|-----------:|-------|
| ICESTORM_LC  |  794 | 5280 (15%) | rises to 800–820 for `twin`, drops to 698 for `brams` |
| ICESTORM_RAM |    8 |   30 (26%) | rises to 20 for `brams` |
| ICESTORM_DSP |    1 |    8 (12%) | the 16×8 multiply infers a hard MAC on UP5K |
| ICESTORM_PLL |    1 |    1       | 12 MHz → 82.5 MHz |
| SB_IO        |    9 |   39 (23%) | 6 gates + UART RX + 12 MHz clock + heartbeat LED |
| SB_GB        |    8 |    8 (100%)| global buffers fully used (clocks + CE/reset promotions) |

## Variant Selection & Synthesis Comparison

The fast-domain PWM block has three interchangeable implementations. All three present an identical external interface and are selected at synthesis time with `-DVARIANT_PIPE`, `-DVARIANT_TWIN`, or `-DVARIANT_BRAMS`. The Makefile exposes them as `make top_pipe`, `make top_twin`, and `make top_brams`; `make top_compare` builds all three for side-by-side timing.

The numbers below are from a `nextpnr-ice40 --up5k --package sg48 --freq 85` run with the full integrated top (PLL + UART + cmd_parser + sine_lut + spwm_tdm + pwm_gate_unit + variant). All three pass the 85 MHz constraint (PLL target 82.5 MHz).

| Variant     | Fmax (clk_fast)  | clk_slow  | LCs  | BRAMs | Fast-domain critical path |
|-------------|-----------------:|----------:|-----:|------:|--------------------------:|
| `top_pipe`  | **103.57 MHz** ★ | 61.44 MHz | 794  |  8    | 4.37 ns logic + 5.28 ns routing |
| `top_twin`  |  86.61 MHz       | 63.02 MHz | 794  |  8    | 4.32 ns logic + 7.23 ns routing |
| `top_brams` |  87.00 MHz       | 54.36 MHz | 698  | 20    | 4.37 ns logic + 7.12 ns routing |

Logic depth is essentially identical across all three (~4.3 ns — the compare-merge into the gate FF). The differences are pure routing: the `pipelined` placement happens to find a tighter merge layout, which is why it's ~16 MHz faster despite having the same logic structure as `twin`. The `brams` variant saves 96 LCs by moving the counter into a ROM table but spends 12 extra BRAMs to do so.

**Default: `VARIANT = PIPE`.** It has the largest Fmax margin over the 82.5 MHz PLL, no BRAM cost beyond the sine LUT + PWM-internal RAMs, and the same LC count as `twin`. `twin` and `brams` are kept buildable as backups in case `pipe` regresses after future changes — placement is non-deterministic, so re-runs vary by a few MHz.

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
make sim_uart       # UART receiver
make sim_top        # Full system
```

### Synthesize & Program
```bash
make all                       # synth → pnr → bitstream using default VARIANT (PIPE)
make all VARIANT=TWIN          # build with a different variant
make top_pipe                  # per-variant build (also: top_twin, top_brams)
make top_compare               # build all three, print Fmax/util side by side
make timing                    # icetime timing analysis
make prog                      # upload to FPGA via iceprog
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
│   ├── pwm_gate_unit.v                  # variant-selecting wrapper
│   ├── pwm_phase_correct_pipelined.v    # variant: pipelined counter
│   ├── pwm_phase_correct_twin.v         # variant: dual up/down counters
│   ├── pwm_phase_correct_brams.v        # variant: BRAM lookup table
│   ├── sine_init.hex                    # generated by scripts/gen_sine_table.py
│   └── counter_table.hex                # generated by scripts/gen_counter_rom.py (used by brams variant)
│   ├── tb_top.v
│   ├── tb_spwm_tdm.v
│   └── tb_uart_rx.v
├── constraints/
│   └── pinout.pcf
└── scripts/
    ├── gen_sine_table.py
    └── gen_counter_rom.py               # BRAM counter table for brams variant
```


## Working with remote


```
while true
do
rsync -avz -e "ssh -i <>.pem" . ubuntu@<remote-ip>:~/icestick-6pwm-bldc-Controller/
sleep 5
done
```

```
make clean && rm log.txt && make top_compare 2>&1 | tee log.txt && zip build/logs.zip build/*nextpnr.log build/*.bin
```

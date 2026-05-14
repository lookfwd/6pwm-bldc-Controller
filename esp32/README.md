# ESP32-S3 SPWM Motor Controller

ESP-IDF port (under PlatformIO) of the iCE40 FPGA SPWM motor controller
(parent repo). Wire-compatible with the existing host tools —
[scripts/bldc_ctl.py](../scripts/bldc_ctl.py) and
[scripts/bldc_ctl.html](../scripts/bldc_ctl.html) drive this firmware unchanged.

## Architecture

- MCPWM peripheral in up-down (phase-correct) mode at 20141 Hz (PLL_F160M / 2·3972), ~12-bit duty resolution.
- Hardware dead-time module inserts ~625 ns between H/L transitions.
- TEZ (trough) event fires an `IRAM_ATTR` ISR that runs the 32-bit NCO phase
  accumulator, three sine-LUT lookups (2048 × signed int16), and three
  comparator writes. MCPWM shadow registers commit all three duties together
  on the next TEZ, reproducing the FPGA's atomic-update semantics.
- UART command parser runs in a FreeRTOS task on core 1 (the ISR sits on core
  0). MIDI-style framing with XOR checksum, same byte layout as the FPGA.

Files of note:

| File | Role |
|---|---|
| `src/spwm.[ch]` | MCPWM setup + TEZ ISR (NCO + LUT + writes) |
| `src/cmd_parser.[ch]` | UART RX task with the 7-state FSM |
| `src/sine_lut.h` | Generated 2048-entry signed sine table |
| `src/pinout.h` | All board GPIO assignments |
| `src/main.c` | Boot, heartbeat blink |

## Build (PlatformIO)

PlatformIO drives ESP-IDF under the hood — no separate `idf.py` invocation
needed.

```
# Regenerate the LUT header (only when scripts/gen_sine_table.py changes):
python3 ../scripts/gen_sine_table.py --format c > src/sine_lut.h

# Build / flash / monitor:
pio run                       # build
pio run -t upload             # flash
pio device monitor            # serial monitor at 115200 (set in platformio.ini)
```

VS Code with the PlatformIO IDE extension picks up `platformio.ini`
automatically and provides the same build/upload/monitor actions through the
status bar.

## Pinout (ESP32-S3-DevKitC-1-N8R8)

All in [src/pinout.h](src/pinout.h). Defaults are placeholders — edit before
wiring to a real gate driver.

| Signal | GPIO |
|---|---|
| UH | 4 |
| UL | 5 |
| VH | 6 |
| VL | 7 |
| WH | 15 |
| WL | 16 |
| UART RX (host → ESP32) | 18 |
| UART TX (ESP32 → host) | 17 |
| Heartbeat LED | 48 |

## Host-side compatibility

The PWM frequency on the ESP32 (20141 Hz) matches the FPGA's `F_PWM_HZ`
within 0.003%, so [scripts/bldc_ctl.py](../scripts/bldc_ctl.py)'s
`phase_inc = round(freq_hz × 2³² / F_PWM_HZ)` formula maps cleanly. No host
changes required.

## Verification

1. Heartbeat LED blinks at ~2 Hz after `pio run -t upload`.
2. Scope all six gate pins at boot — boot defaults are amplitude=255 and
   5 Hz fundamental (see `SPWM_BOOT_*` in `src/spwm.c`), so you should see
   full-scale SPWM at ~20 kHz carrier with a 5 Hz envelope and 120° offset
   between U/V/W. Dead-time on every transition ~625 ns.
3. From the host: `python ../scripts/bldc_ctl.py --speed 6 --amp 128` should
   produce the same SPWM envelope a logic analyzer would see from the FPGA.
4. Verify 120° phase offset between U/V/W by overlaying captures.

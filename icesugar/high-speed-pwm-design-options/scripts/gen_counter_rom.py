#!/usr/bin/env python3
"""
Emit the init file for the BRAM-table variant in
src/pwm_phase_correct_brams.v.

Layout: 4096 entries × 13 bits, written to build/counter_table.hex.
  Address: a free-running 12-bit counter (0 → 4095 → wraps to 0).
  Data[10:0]: displayed counter value (0 → 2047 → 2047 → 0 triangle
              with 1-cycle dwell at peak and trough).
  Data[11]  : sync, hardcoded high at addr == 0.
  Data[12]  : direction (0 = up for addr 0..2047, 1 = down for 2048..4095).

The other two variants (pwm_phase_correct_pipelined.v and
pwm_phase_correct_twin.v) do not need this file — they derive the same
signals combinationally from the free-running address counter.

Usage:
    python3 scripts/gen_counter_rom.py [out_dir]
    out_dir defaults to ./build
"""
import os
import sys


def main() -> None:
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "build"
    os.makedirs(out_dir, exist_ok=True)

    table_path = os.path.join(out_dir, "counter_table.hex")

    with open(table_path, "w") as f:
        for addr in range(4096):
            counter   = addr if addr <= 2047 else (4095 - addr)
            sync      = 1 if addr == 0 else 0
            direction = 0 if addr <= 2047 else 1
            word      = (direction << 12) | (sync << 11) | counter
            f.write(f"{word:04x}\n")

    print(f"Wrote {table_path}")


if __name__ == "__main__":
    main()

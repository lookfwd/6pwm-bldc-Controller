#!/usr/bin/env python3
"""Generate a 2048-entry, 16-bit unsigned offset-binary sine table.

Output format: one hex value per line, suitable for Verilog $readmemh.

Encoding (Unsigned Offset Binary):
    0x0000 = negative peak (-1.0)
    0x8000 = zero crossing  (0.0)
    0xFFFF = positive peak  (+1.0)
"""

import math

ENTRIES = 2048
BITS = 16
MAX_VAL = (1 << BITS) - 1  # 65535

for i in range(ENTRIES):
    angle = 2.0 * math.pi * i / ENTRIES
    value = int(round(32767.5 + 32767.5 * math.sin(angle)))
    value = max(0, min(MAX_VAL, value))
    print(f"{value:04X}")

#!/usr/bin/env python3
"""
Control the iCE40 SPWM motor controller over UART.

Protocol (matches src/cmd_parser.v):
    7-byte packet, MIDI-style framing with XOR checksum:

      [1xxx_xxxx]                  sync/status (MSB=1, low 7 bits = cmd)
      [0xxx_xxxx] × 5              5 data bytes (7-bit each, big-endian)
      [0xxx_xxxx]                  XOR checksum (7-bit)

    Any byte with MSB=1 resyncs the receiver to a new packet immediately,
    so a single bad/dropped byte loses at most one packet.

    Status bytes:
      0x80 — set_state  (reserved, currently ignored by FPGA)
      0x81 — set_speed  payload[31:0] = 32-bit NCO phase increment
      0x82 — set_amp    payload[7:0]  = 8-bit amplitude

    Payload reconstruction (35 bits, big-endian 7-bit chunks):
      payload = (D4 << 28) | (D3 << 21) | (D2 << 14) | (D1 << 7) | D0

    Checksum = (cmd ^ D4 ^ D3 ^ D2 ^ D1 ^ D0) & 0x7F.

UART line settings: 8N1, 115200 baud, no flow control.

Examples:
    # Set 50 Hz output, half amplitude:
    python bldc_ctl.py /dev/ttyUSB1 amp 128
    python bldc_ctl.py /dev/ttyUSB1 speed 50

    # Quick 5-second demo run:
    python bldc_ctl.py /dev/ttyUSB1 demo

    # Or use as a library:
    from bldc_ctl import Controller
    with Controller('/dev/ttyUSB1') as c:
        c.set_amplitude(200)
        c.set_speed_hz(60)
"""

import argparse
import sys
import time

import serial


# PWM frequency = clk_fast / (2 * counter_period)
#               = 82.5 MHz / (2 * 2048)
F_PWM_HZ = 82_500_000 / (2 * 2048)   # ≈ 20141.60 Hz


def _pack(cmd, payload):
    """Build a 7-byte MIDI-framed packet: sync + 5 data + XOR checksum."""
    cmd     = cmd & 0x7F
    payload = payload & 0x7FFFFFFFF  # 35-bit clamp
    sync = 0x80 | cmd
    d4 = (payload >> 28) & 0x7F
    d3 = (payload >> 21) & 0x7F
    d2 = (payload >> 14) & 0x7F
    d1 = (payload >>  7) & 0x7F
    d0 = (payload >>  0) & 0x7F
    cs = (cmd ^ d4 ^ d3 ^ d2 ^ d1 ^ d0) & 0x7F
    return bytes([sync, d4, d3, d2, d1, d0, cs])


class Controller:
    """Send 7-byte MIDI-framed command packets to the BLDC controller over UART."""

    CMD_STATE = 0x00   # reserved, currently ignored by FPGA
    CMD_SPEED = 0x01
    CMD_AMP   = 0x02

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(
            port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
        )

    def close(self):
        self.ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _send(self, cmd, payload):
        self.ser.write(_pack(cmd, payload))
        self.ser.flush()

    # --- Commands ---------------------------------------------------------

    def set_amplitude(self, amplitude):
        """8-bit unsigned amplitude scale, 0..255."""
        if not 0 <= amplitude <= 255:
            raise ValueError(f"amplitude out of range: {amplitude}")
        self._send(self.CMD_AMP, amplitude)

    def set_speed_inc(self, phase_inc):
        """Set the raw 32-bit NCO phase increment."""
        if not 0 <= phase_inc <= 0xFFFFFFFF:
            raise ValueError(f"phase_inc out of 32-bit range: {phase_inc}")
        self._send(self.CMD_SPEED, phase_inc)

    def set_speed_hz(self, freq_hz):
        """
        Set output frequency in Hz. Returns the phase_inc actually programmed.
        Negative frequencies are accepted and produce reverse rotation
        (handled naturally by NCO wrap-around).
        """
        phase_inc = round(freq_hz * (1 << 32) / F_PWM_HZ) & 0xFFFFFFFF
        self.set_speed_inc(phase_inc)
        return phase_inc


# --- CLI ------------------------------------------------------------------

def _int_auto(s):
    """argparse type accepting decimal or 0x-prefixed hex."""
    return int(s, 0)


def main(argv=None):
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("port", help="Serial device (e.g. /dev/ttyUSB1, COM3)")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("amp", help="Set amplitude (0..255)")
    sp.add_argument("value", type=int)

    sp = sub.add_parser("speed", help="Set output frequency in Hz")
    sp.add_argument("hz", type=float)

    sp = sub.add_parser("inc", help="Set raw 32-bit phase increment")
    sp.add_argument("value", type=_int_auto, help="decimal or 0x-prefixed hex")

    sp = sub.add_parser("demo", help="Quick 5-second spin sequence")
    sp.add_argument("--hz", type=float, default=50.0)
    sp.add_argument("--amp", type=int, default=128)
    sp.add_argument("--seconds", type=float, default=5.0)

    args = p.parse_args(argv)

    try:
        with Controller(args.port, args.baud) as c:
            if args.cmd == "amp":
                c.set_amplitude(args.value)
                print(f"amplitude = {args.value}")

            elif args.cmd == "speed":
                inc = c.set_speed_hz(args.hz)
                print(f"speed = {args.hz} Hz  (phase_inc = {inc} / 0x{inc:08X})")

            elif args.cmd == "inc":
                c.set_speed_inc(args.value)
                hz = args.value * F_PWM_HZ / (1 << 32)
                print(f"phase_inc = {args.value} (0x{args.value:08X}) ≈ {hz:.3f} Hz")

            elif args.cmd == "demo":
                print(f"amp={args.amp}, speed={args.hz} Hz")
                c.set_amplitude(args.amp)
                c.set_speed_hz(args.hz)
                print(f"running for {args.seconds} s …")
                time.sleep(args.seconds)
                c.set_amplitude(0)
                print("amp=0")

    except serial.SerialException as e:
        print(f"serial error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

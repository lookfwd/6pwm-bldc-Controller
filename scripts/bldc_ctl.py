#!/usr/bin/env python3
"""
Control the iCE40 SPWM motor controller over UART.

Protocol (matches src/cmd_parser.v):
    Each command is a 5-byte packet: [CMD] [D3] [D2] [D1] [D0]
    Data bytes are big-endian — D3 is sent first, D0 last.

    CMD 0x01 — set state.    D0 = 0 (OPEN), 1 (RUNNING), 2 (BRAKE)
    CMD 0x02 — set speed.    D3..D0 = 32-bit NCO phase increment
    CMD 0x03 — set amplitude. D0 = 0..255

UART line settings: 8N1, 115200 baud, no flow control.

Note: cmd_parser inserts an OPEN sandwich on every state change, so transitions
RUNNING <-> BRAKE go through a guaranteed dead-time period automatically.

Examples:
    # Set 50 Hz output, half amplitude, run the motor:
    python bldc_ctl.py /dev/ttyUSB1 amp 128
    python bldc_ctl.py /dev/ttyUSB1 speed 50
    python bldc_ctl.py /dev/ttyUSB1 state running

    # Quick 5-second demo run:
    python bldc_ctl.py /dev/ttyUSB1 demo

    # Or use as a library:
    from bldc_ctl import Controller
    with Controller('/dev/ttyUSB1') as c:
        c.set_amplitude(200)
        c.set_speed_hz(60)
        c.set_state(Controller.STATE_RUNNING)
"""

import argparse
import struct
import sys
import time

import serial


# PWM frequency = clk_fast / (2 * counter_period)
#               = 50.25 MHz / (2 * 1024)
F_PWM_HZ = 50_250_000 / (2 * 1024)   # ≈ 24536.13 Hz


class Controller:
    """Send 5-byte command packets to the BLDC controller over UART."""

    STATE_OPEN    = 0
    STATE_RUNNING = 1
    STATE_BRAKE   = 2

    CMD_STATE = 0x01
    CMD_SPEED = 0x02
    CMD_AMP   = 0x03

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
        if len(payload) != 4:
            raise ValueError("payload must be exactly 4 bytes")
        pkt = bytes([cmd]) + payload
        self.ser.write(pkt)
        self.ser.flush()

    # --- Commands ---------------------------------------------------------

    def set_state(self, state):
        """0 = OPEN, 1 = RUNNING, 2 = BRAKE."""
        if state not in (0, 1, 2):
            raise ValueError(f"state must be 0, 1, or 2; got {state!r}")
        self._send(self.CMD_STATE, bytes([0, 0, 0, state]))

    def set_amplitude(self, amplitude):
        """8-bit unsigned amplitude scale, 0..255."""
        if not 0 <= amplitude <= 255:
            raise ValueError(f"amplitude out of range: {amplitude}")
        self._send(self.CMD_AMP, bytes([0, 0, 0, amplitude]))

    def set_speed_inc(self, phase_inc):
        """Set the raw 32-bit NCO phase increment."""
        if not 0 <= phase_inc <= 0xFFFFFFFF:
            raise ValueError(f"phase_inc out of 32-bit range: {phase_inc}")
        self._send(self.CMD_SPEED, struct.pack(">I", phase_inc))

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

_STATE_NAMES = {"open": 0, "running": 1, "brake": 2}


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

    sp = sub.add_parser("state", help="Set controller state")
    sp.add_argument("which", choices=list(_STATE_NAMES))

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
            if args.cmd == "state":
                c.set_state(_STATE_NAMES[args.which])
                print(f"state = {args.which.upper()}")

            elif args.cmd == "amp":
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
                print(f"amp={args.amp}, speed={args.hz} Hz, state=RUNNING")
                c.set_amplitude(args.amp)
                c.set_speed_hz(args.hz)
                c.set_state(Controller.STATE_RUNNING)
                print(f"running for {args.seconds} s …")
                time.sleep(args.seconds)
                print("state=OPEN")
                c.set_state(Controller.STATE_OPEN)

    except serial.SerialException as e:
        print(f"serial error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())

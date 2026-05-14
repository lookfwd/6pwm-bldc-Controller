// UART command parser — direct port of the FPGA's MIDI-style state machine.
//
// Wire-compatible with src/cmd_parser.v: any host that talks to the FPGA
// (e.g. scripts/bldc_ctl.py) drives this firmware unchanged.

#pragma once

// Install the UART driver, then spawn a FreeRTOS task that pumps bytes
// through the parser state machine and calls spwm_set_*() on each valid
// frame. Call once at boot, after spwm_init().
void cmd_parser_start(void);

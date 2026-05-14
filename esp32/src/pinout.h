// Pin assignments for the ESP32-S3-DevKitC-1-N8R8 motor controller harness.
// All board-specific GPIO numbers are isolated here so retargeting to another
// dev board is a one-file change.

#pragma once

// Gate driver outputs — six MCPWM generators, one per IGBT/MOSFET gate.
// Order matches the FPGA design's pinout: {UH, UL, VH, VL, WH, WL}.
#define GATE_UH_GPIO    4
#define GATE_UL_GPIO    5
#define GATE_VH_GPIO    6
#define GATE_VL_GPIO    7
#define GATE_WH_GPIO   15
#define GATE_WL_GPIO   16

// UART0 — shared with the boot/log console and the on-board USB-UART bridge.
// One USB-C cable carries both the IDF log output (ESP32 → host) and the
// command-protocol RX (host → ESP32). The two directions are independent
// on the wire, so they don't collide; you'll just see your outgoing command
// bytes echoed as garbage in `pio device monitor` when the host writes.
//
// Switch to a dedicated UART1 / GPIO17+18 later if you wire a second
// USB-TTL adapter for clean separation.
#define HOST_UART_NUM      0
#define HOST_UART_RX_GPIO  44   // bridge RX (= ESP32 RX on UART0)
#define HOST_UART_TX_GPIO  43   // bridge TX (= ESP32 TX on UART0)

// Heartbeat indicator.  GPIO38 drives the DevKitC-1's on-board WS2812B
// addressable RGB LED; main.c uses the led_strip component to send a soft
// green breathing pulse.  Re-point this and the led_strip config in
// main.c if you wire a different LED.
#define LED_HEARTBEAT_GPIO 38

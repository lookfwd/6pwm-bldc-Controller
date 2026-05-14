// MCPWM-based 3-phase symmetric SPWM generator.
//
// One MCPWM timer in up-down (phase-correct) mode drives three operators,
// one per phase. Hardware dead-time splits each phase's single comparator
// value into the H/L gate pair so the high-level pipeline writes only three
// duty values per PWM period.
//
// The TEZ (timer-equals-zero) event fires an IRAM-resident ISR that runs the
// NCO phase accumulator, the sine LUT lookups, and the comparator writes.
//
// Wire-protocol compatibility with the FPGA design is maintained by matching
// the PWM frequency: 80 MHz / (2 * 1986) = 20140.99 Hz (0.003% below the
// FPGA's 20141.6 Hz — within F_PWM_HZ precision).

#pragma once

#include <stdint.h>

// PWM timer configuration.
//   F_PWM ≈ 20141 Hz, full period = 2 * 3972 = 7944 ticks at 160 MHz (49.65 µs).
//   Resolution: 3972 distinct duty steps (≈11.96-bit, effectively 12-bit).
//
// ESP32-S3's MCPWM_TIMER_CLK_SRC_DEFAULT is PLL_F160M, so we ask the driver
// for a 160 MHz tick rate (prescaler = 1) to double the resolution vs the
// 80 MHz / 11-bit FPGA parity setup. F_PWM stays unchanged so host scripts
// don't need to update F_PWM_HZ. ISR budget stays at 49.6 µs.
//
// NOTE: IDF's mcpwm_timer_config_t.period_ticks is halved internally for
// symmetric / up-down mode (esp_driver_mcpwm/src/mcpwm_timer.c:101). The HW
// counter peak ends up at config.period_ticks / 2.  SPWM_PEAK_TICKS below is
// the HW peak — pass 2 * SPWM_PEAK_TICKS to the IDF config to land there.
// Valid comparator values are then [0, SPWM_PEAK_TICKS].
#define SPWM_RESOLUTION_HZ   160000000u   // MCPWM tick rate (PLL_F160M)
#define SPWM_PEAK_TICKS      3972u        // HW counter peak in up-down mode

// Dead-time inserted by MCPWM's hardware dead-time module between each
// gate-pair transition. Matches the FPGA's 2*HALF_DEAD_TIME=50 ticks at
// 82.5 MHz (~606 ns); 100 ticks at 160 MHz = 625 ns, same as before.
#define SPWM_DEAD_TIME_TICKS 100u

// Phase offsets in 2048-entry sine LUT (mirrors src/spwm_tdm.v).
//   V = +120° = +683 entries, W = +240° = +1365 entries.
#define SPWM_OFFSET_V        683u
#define SPWM_OFFSET_W        1365u

// Initialise the MCPWM peripheral, configure the three operators with their
// generators and hardware dead-time, register the TEZ ISR, and start the
// timer.  Must be called once at boot.
void spwm_init(void);

// Atomic setters called by the UART command parser. The ISR reads these on
// every TEZ; 32-bit aligned writes on Xtensa are atomic from the application
// perspective.
void spwm_set_phase_inc(uint32_t phase_inc);
void spwm_set_amplitude(uint8_t amplitude);

// Read accessor: top bit of the NCO accumulator. Returns false (0) while
// sin(phase U) >= 0, true (1) while sin(phase U) < 0. Toggles at each sine
// zero crossing → fundamental-frequency heartbeat indicator.
#include <stdbool.h>
bool spwm_sine_sign(void);

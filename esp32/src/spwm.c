// MCPWM-based 3-phase symmetric SPWM generator — implementation.
//
// The MCPWM peripheral handles up-down counting and hardware dead-time. The
// TEZ event (timer hits zero at the trough) wakes an IRAM-resident ISR that
// advances the NCO accumulator, looks up three sine samples, and writes
// three comparator values. MCPWM shadow registers latch all three
// comparators atomically on the next TEZ, reproducing the FPGA's
// "duty-at-sync" coherent update.

#include "spwm.h"

#include <stdint.h>

#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "pinout.h"
#include "sine_lut.h"

static const char *TAG = "spwm";

// ---- Shared state ---------------------------------------------------------
// Written by the cmd_parser task, read by the ISR. Both are word-aligned
// scalars; aligned writes on Xtensa are atomic from the application's view.
//
// Boot defaults: 5 Hz fundamental, full amplitude. The host can override
// either at any time via the UART command parser.
//   phase_inc = round(5 Hz * 2^32 / F_PWM)
//             = round(5 * 4294967296 * 2 * SPWM_PEAK_TICKS / SPWM_RESOLUTION_HZ)
//             = round(5 * 4294967296 * 7944 / 160000000)
//             = 1066226
#define SPWM_BOOT_PHASE_INC  1066226u   // ~5 Hz
#define SPWM_BOOT_AMPLITUDE  255u       // full scale

static volatile uint32_t s_phase_inc = SPWM_BOOT_PHASE_INC;
static volatile uint8_t  s_amplitude = SPWM_BOOT_AMPLITUDE;

// ISR-state. Lives in IRAM so the cache miss penalty doesn't hit the PWM
// period budget. `volatile` because the heartbeat loop reads the top bit
// across thread boundaries (32-bit aligned reads are atomic on Xtensa).
static DRAM_ATTR volatile uint32_t s_phase_acc;

// MCPWM handles retained for the ISR.
static mcpwm_cmpr_handle_t s_cmp_u;
static mcpwm_cmpr_handle_t s_cmp_v;
static mcpwm_cmpr_handle_t s_cmp_w;

// Pre-computed duty-math constants. The Verilog formulation centres duty at
// the midpoint of the counter and scales the sine swing by amplitude. We
// keep a dead-time pad on both sides so the comparator never lands inside
// the dead-time window (which would create asymmetric glitches when the
// hardware dead-time module clamps the gate-on time).
//
//   midpoint   = SPWM_PEAK_TICKS / 2
//   swing_max  = midpoint - SPWM_DEAD_TIME_TICKS
//   duty(s,a)  = midpoint + (s * a * swing_max) / (32767 * 255)
//
// At SPWM_PEAK_TICKS = 3972, SPWM_DEAD_TIME_TICKS = 100:
//   midpoint  = 1986
//   swing_max = 1886
#define SPWM_MIDPOINT   (SPWM_PEAK_TICKS / 2u)
#define SPWM_SWING_MAX  (SPWM_MIDPOINT - SPWM_DEAD_TIME_TICKS)
#define SPWM_NORM_DEN   (32767 * 255)

// ---- Duty math (inlined into the ISR) -------------------------------------
static inline IRAM_ATTR uint32_t duty_from_sine(int32_t sine_s16, uint32_t amp)
{
    // sine_s16 in [-32767, +32767], amp in [0, 255].
    // |sine * amp| ≤ 8,355,585 → fits in int32_t. The next step needs 64-bit:
    // |scaled * swing_max| can reach 8,355,585 * 1886 ≈ 1.58e10 which overflows
    // int32_t. After dividing by SPWM_NORM_DEN the final |swing| ≤ swing_max,
    // so the result fits comfortably back into int32_t.
    int32_t scaled = sine_s16 * (int32_t)amp;
    int64_t product = (int64_t)scaled * (int32_t)SPWM_SWING_MAX;
    int32_t swing = (int32_t)(product / SPWM_NORM_DEN);
    int32_t duty  = (int32_t)SPWM_MIDPOINT + swing;
    // Bounds check is defensive — by construction duty ∈
    // [SPWM_DEAD_TIME_TICKS, SPWM_PEAK_TICKS - SPWM_DEAD_TIME_TICKS].
    // MCPWM requires cmp_ticks <= peak_ticks; saturate one below for safety.
    if (duty < 0) duty = 0;
    if (duty >= (int32_t)SPWM_PEAK_TICKS) duty = SPWM_PEAK_TICKS - 1;
    return (uint32_t)duty;
}

// ---- ISR ------------------------------------------------------------------
// Fires on every TEZ (= start of the up half = end of the down half = the
// trough of the triangle). Pure leaf function; no FreeRTOS calls.
static bool IRAM_ATTR mcpwm_tez_isr(mcpwm_timer_handle_t timer,
                                    const mcpwm_timer_event_data_t *edata,
                                    void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    uint32_t phase_inc = s_phase_inc;
    uint32_t amp       = s_amplitude;

    s_phase_acc += phase_inc;

    // Top 11 bits of the 32-bit NCO index a 2048-entry table.
    uint32_t idx_u = (s_phase_acc >> 21) & 0x7FFu;
    uint32_t idx_v = (idx_u + SPWM_OFFSET_V) & 0x7FFu;
    uint32_t idx_w = (idx_u + SPWM_OFFSET_W) & 0x7FFu;

    int32_t s_u = sine_lut[idx_u];
    int32_t s_v = sine_lut[idx_v];
    int32_t s_w = sine_lut[idx_w];

    uint32_t duty_u = duty_from_sine(s_u, amp);
    uint32_t duty_v = duty_from_sine(s_v, amp);
    uint32_t duty_w = duty_from_sine(s_w, amp);

    // Shadow registers update on the *next* TEZ, so all three duties land
    // together for the upcoming PWM period. This is the parity of
    // pwm_phase_correct's duty-at-sync latch.
    mcpwm_comparator_set_compare_value(s_cmp_u, duty_u);
    mcpwm_comparator_set_compare_value(s_cmp_v, duty_v);
    mcpwm_comparator_set_compare_value(s_cmp_w, duty_w);

    return false;  // no task wakeup needed
}

// ---- Setup helpers --------------------------------------------------------
// Build one phase's operator + comparator + two complementary generators +
// hardware dead-time module, all routed onto the supplied GPIO pair.
static void configure_phase(mcpwm_timer_handle_t timer,
                            int gpio_high, int gpio_low,
                            mcpwm_cmpr_handle_t *out_cmp)
{
    mcpwm_oper_handle_t oper;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, out_cmp));
    // Start at the trough's mid-point so the bridge is balanced before the
    // first ISR-driven update.
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(*out_cmp, SPWM_MIDPOINT));

    // High-side generator: classic phase-correct PWM — high in the centred
    // window around TEZ, low around TEP. Force HIGH at TEZ so the very first
    // cycle has the correct polarity; compare-event actions handle the rest.
    mcpwm_gen_handle_t gen_high;
    mcpwm_generator_config_t gen_high_cfg = { .gen_gpio_num = gpio_high };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_high_cfg, &gen_high));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
        gen_high,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_TIMER_EVENT_ACTION_END()));
    ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
        gen_high,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,   *out_cmp, MCPWM_GEN_ACTION_LOW),
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, *out_cmp, MCPWM_GEN_ACTION_HIGH),
        MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

    // Low-side: hardware dead-time module derives it from the high-side
    // waveform — inverted, with rising edge delayed by SPWM_DEAD_TIME_TICKS
    // so the H and L gates never overlap.
    //
    // IDF's dead-time module has two parallel hardware paths keyed by edge:
    //   RED path: posedge_delay_ticks → output naturally goes to gen_id 0
    //   FED path: negedge_delay_ticks → output naturally goes to gen_id 1
    // Each path has its own delay register, so we MUST use one edge on each
    // call (high = RED via posedge, low = FED via negedge + invert) — using
    // posedge on both clobbers RED's delay register and re-routes the path,
    // leaving the high-side with no dead-time.
    mcpwm_gen_handle_t gen_low;
    mcpwm_generator_config_t gen_low_cfg = { .gen_gpio_num = gpio_low };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_low_cfg, &gen_low));

    // RED path: delay the high-side's rising edge.
    mcpwm_dead_time_config_t dt_high = {
        .posedge_delay_ticks = SPWM_DEAD_TIME_TICKS,
        .negedge_delay_ticks = 0,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_high, gen_high, &dt_high));

    // FED path: take the high-side signal, delay its falling edge, then
    // invert before driving gen_low's pin. Net effect: gen_low rises
    // SPWM_DEAD_TIME_TICKS after gen_high falls.
    mcpwm_dead_time_config_t dt_low = {
        .posedge_delay_ticks = 0,
        .negedge_delay_ticks = SPWM_DEAD_TIME_TICKS,
        .flags.invert_output = true,
    };
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_high, gen_low, &dt_low));
}

void spwm_init(void)
{
    ESP_LOGI(TAG, "MCPWM init: F_PWM≈%u Hz, peak=%u ticks, dt=%u ticks",
             SPWM_RESOLUTION_HZ / (2u * SPWM_PEAK_TICKS),
             SPWM_PEAK_TICKS,
             SPWM_DEAD_TIME_TICKS);

    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,   // PLL_F160M
        .resolution_hz = SPWM_RESOLUTION_HZ,
        // IDF halves this internally in symmetric mode → HW peak = SPWM_PEAK_TICKS.
        .period_ticks  = 2u * SPWM_PEAK_TICKS,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    configure_phase(timer, GATE_UH_GPIO, GATE_UL_GPIO, &s_cmp_u);
    configure_phase(timer, GATE_VH_GPIO, GATE_VL_GPIO, &s_cmp_v);
    configure_phase(timer, GATE_WH_GPIO, GATE_WL_GPIO, &s_cmp_w);

    mcpwm_timer_event_callbacks_t cbs = {
        .on_empty = mcpwm_tez_isr,   // TEZ in up-down mode = on_empty
    };
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &cbs, NULL));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void spwm_set_phase_inc(uint32_t phase_inc)
{
    s_phase_inc = phase_inc;
}

bool spwm_sine_sign(void)
{
    // Top bit of the NCO accumulator. == 0 while sin(phase) >= 0 (phase
    // 0°..180°), == 1 in the negative half. Toggles at every zero crossing.
    return (s_phase_acc & 0x80000000u) != 0;
}

void spwm_set_amplitude(uint8_t amplitude)
{
    s_amplitude = amplitude;
}

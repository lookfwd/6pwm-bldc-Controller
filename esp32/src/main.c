// Entry point for the ESP32-S3 SPWM motor controller firmware.
//
// Brings up the MCPWM peripheral (which immediately starts producing
// 50%-duty switching at ~20 kHz on all six gate pins, with the configured
// dead-time), then installs the UART command parser, then runs a soft
// green breathing pulse on the on-board WS2812B as a "firmware is alive"
// indicator.
//
// All the real work happens elsewhere: spwm_init() owns the PWM hardware
// and the TEZ ISR, cmd_parser_start() owns the UART task. main_app just
// owns the heartbeat LED.

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "cmd_parser.h"
#include "pinout.h"
#include "spwm.h"

static const char *TAG = "main";

// Heartbeat: green during the positive half of phase-U sine, off during
// the negative half. Toggle rate == fundamental frequency, so the LED's
// blink period is a direct visual readout of the commanded motor speed.
// Sample at 20 Hz — fast enough to track up to ~10 Hz fundamental without
// aliasing visibly; above that the LED looks steady-on anyway (PoV).
#define HEARTBEAT_TICK_MS    50u
#define HEARTBEAT_PEAK       32u   // dim green (0..255)

static led_strip_handle_t s_led_strip;

static void heartbeat_init(void)
{
    led_strip_config_t strip_cfg = {
        .strip_gpio_num         = LED_HEARTBEAT_GPIO,
        .max_leds               = 1,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,   // 10 MHz; ample timing margin
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_led_strip));
    ESP_ERROR_CHECK(led_strip_clear(s_led_strip));
}

static void heartbeat_loop(void)
{
    for (;;) {
        // spwm_sine_sign(): false during sin >= 0, true during sin < 0.
        uint8_t green = spwm_sine_sign() ? 0u : HEARTBEAT_PEAK;
        led_strip_set_pixel(s_led_strip, 0, 0, green, 0);
        led_strip_refresh(s_led_strip);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_TICK_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "iCE40 SPWM port — ESP32-S3 boot");

    heartbeat_init();
    spwm_init();
    cmd_parser_start();

    heartbeat_loop();   // never returns
}

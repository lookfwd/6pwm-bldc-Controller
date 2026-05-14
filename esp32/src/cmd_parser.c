// UART command parser — implementation.
//
// State machine mirrors src/cmd_parser.v byte-for-byte. Any received byte
// with MSB=1 is a sync/status byte and instantly restarts the frame, so a
// truncated packet does not poison the next one. Data bytes are 7-bit; the
// trailing checksum is the XOR of the cmd and all five data bytes (low 7
// bits). A bad checksum silently drops the frame.

#include "cmd_parser.h"

#include <stdint.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pinout.h"
#include "spwm.h"

static const char *TAG = "cmd_parser";

// Command codes — match src/cmd_parser.v.
#define CMD_STATE  0x00u   // reserved (no-op)
#define CMD_SPEED  0x01u
#define CMD_AMP    0x02u

// 7-state FSM, same encoding as the Verilog.
typedef enum {
    S_IDLE = 0,
    S_D4,
    S_D3,
    S_D2,
    S_D1,
    S_D0,
    S_CHK,
} parse_state_t;

typedef struct {
    parse_state_t state;
    uint8_t cmd;
    uint8_t d4, d3, d2, d1, d0;
    uint8_t running_xor;
} parser_t;

// Apply a completed, checksum-verified frame.
static void parser_apply(const parser_t *p)
{
    switch (p->cmd) {
        case CMD_SPEED: {
            // payload[31:0] = {d4[3:0], d3, d2, d1, d0}
            uint32_t phase_inc =
                ((uint32_t)(p->d4 & 0x0F) << 28) |
                ((uint32_t)p->d3 << 21) |
                ((uint32_t)p->d2 << 14) |
                ((uint32_t)p->d1 << 7)  |
                ((uint32_t)p->d0);
            spwm_set_phase_inc(phase_inc);
            break;
        }
        case CMD_AMP: {
            // payload[7:0] = {d1[0], d0}
            uint8_t amplitude =
                (uint8_t)(((p->d1 & 0x01) << 7) | p->d0);
            spwm_set_amplitude(amplitude);
            break;
        }
        case CMD_STATE:
        default:
            // Reserved / unknown — drop.
            break;
    }
}

static void parser_feed(parser_t *p, uint8_t byte)
{
    if (byte & 0x80u) {
        // Sync byte — always restarts the frame.
        p->cmd = byte & 0x7Fu;
        p->running_xor = p->cmd;
        p->state = S_D4;
        return;
    }

    uint8_t d7 = byte & 0x7Fu;

    switch (p->state) {
        case S_D4:
            p->d4 = d7;
            p->running_xor ^= d7;
            p->state = S_D3;
            break;
        case S_D3:
            p->d3 = d7;
            p->running_xor ^= d7;
            p->state = S_D2;
            break;
        case S_D2:
            p->d2 = d7;
            p->running_xor ^= d7;
            p->state = S_D1;
            break;
        case S_D1:
            p->d1 = d7;
            p->running_xor ^= d7;
            p->state = S_D0;
            break;
        case S_D0:
            p->d0 = d7;
            p->running_xor ^= d7;
            p->state = S_CHK;
            break;
        case S_CHK:
            if (d7 == p->running_xor) {
                parser_apply(p);
            }
            // Whether the checksum matched or not, fall back to IDLE and wait
            // for the next sync byte.
            p->state = S_IDLE;
            break;
        case S_IDLE:
        default:
            // Stray data byte with no preceding sync — ignore.
            break;
    }
}

// FreeRTOS task: blocks on the UART RX queue and feeds bytes through the
// parser. Static buffer is the RX chunk; one byte at a time is the cleanest
// way to map onto the byte-driven FSM.
static void cmd_parser_task(void *arg)
{
    (void)arg;
    uint8_t buf[64];
    for (;;) {
        int n = uart_read_bytes(HOST_UART_NUM, buf, sizeof(buf), portMAX_DELAY);
        if (n <= 0) continue;

        static parser_t parser;   // zero-initialised: state = S_IDLE
        for (int i = 0; i < n; ++i) {
            parser_feed(&parser, buf[i]);
        }
    }
}

void cmd_parser_start(void)
{
    uart_config_t cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(HOST_UART_NUM,
                                        /* rx_buffer_size */ 1024,
                                        /* tx_buffer_size */ 0,
                                        /* queue_size     */ 0,
                                        /* uart_queue     */ NULL,
                                        /* intr_alloc_flags */ 0));
    ESP_ERROR_CHECK(uart_param_config(HOST_UART_NUM, &cfg));
#if HOST_UART_NUM == 0
    // UART0 is the boot console — pins are already routed by the IDF startup
    // path. Calling uart_set_pin here would re-touch GPIO drive strength /
    // pull-ups on a wire that's actively carrying log output, so we leave it
    // alone. RX still arrives at the driver's FIFO via the existing matrix.
#else
    ESP_ERROR_CHECK(uart_set_pin(HOST_UART_NUM,
                                 HOST_UART_TX_GPIO,
                                 HOST_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
#endif

    ESP_LOGI(TAG, "UART%d 115200 8N1, rx=%d tx=%d",
             HOST_UART_NUM, HOST_UART_RX_GPIO, HOST_UART_TX_GPIO);

    BaseType_t ok = xTaskCreatePinnedToCore(cmd_parser_task,
                                            "cmd_parser",
                                            /* stack    */ 3072,
                                            /* arg      */ NULL,
                                            /* priority */ 10,
                                            NULL,
                                            /* core 1 — keep MCPWM ISR on core 0 */ 1);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "cmd_parser task failed to start");
    }
}

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "hal.h"
#include "config.h"
#include "system_event.h"
#include "safety_monitor.h"
#include "event_dispatcher.h"

static const char *TAG = "HAL";

/** Handle to the central Dispatcher queue (set during hal_init). */
static QueueHandle_t s_dispatcher_queue = NULL;

/** TX queue handle — backpressure-safe outbound message buffer. */
static QueueHandle_t s_tx_queue = NULL;

typedef struct {
    uint8_t  raw_state;                      
    uint8_t  stable_state;    
    uint32_t last_change_ms;  
    uint8_t  pending;         
} debounce_ctx_t;

static debounce_ctx_t s_dbc_fully_open   = {0, 0, 0, 0};
static debounce_ctx_t s_dbc_fully_closed = {0, 0, 0, 0};
static debounce_ctx_t s_dbc_obstruction  = {0, 0, 0, 0};

static uint8_t crc8_compute(const uint8_t *data, size_t len)
{
    uint8_t crc = CRC8_INITIAL;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8u; bit++) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1u) ^ CRC8_POLYNOMIAL);
            } else {
                crc <<= 1u;
            }
        }
    }
    return crc;
}

static uint8_t debounce_update(debounce_ctx_t *ctx, uint8_t new_sample)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);

    if (new_sample != ctx->raw_state) {
        ctx->raw_state      = new_sample;
        ctx->last_change_ms = now_ms;
        ctx->pending        = 1u;
        return 0u;
    }

    if (ctx->pending) {
        uint32_t elapsed = now_ms - ctx->last_change_ms;
        if (elapsed >= SENSOR_DEBOUNCE_MS) {
            ctx->stable_state = new_sample;
            ctx->pending      = 0u;
            return 1u; 
        }
    }
    return 0u;
}

static uint8_t spof_check(uint8_t fully_open_stable, uint8_t fully_closed_stable)
{
    return (fully_open_stable && fully_closed_stable) ? 1u : 0u;
}

static uint8_t parse_frame(const char *frame, system_event_t *out_evt)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000LL);
    out_evt->source       = SRC_UART_SUPERVISOR;
    out_evt->timestamp_ms = now_ms;
    out_evt->payload      = 0u;

    /* CMD frames */
    if (strncmp(frame, "$CMD,TYPE=OPEN", 14) == 0) {
        out_evt->type = EVT_CMD_OPEN;
        return 1u;
    }
    if (strncmp(frame, "$CMD,TYPE=CLOSE", 15) == 0) {
        out_evt->type = EVT_CMD_CLOSE;
        return 1u;
    }
    if (strncmp(frame, "$CMD,TYPE=EMERGENCY_OPEN", 24) == 0) {
        out_evt->type = EVT_CMD_EMERGENCY_OPEN;
        return 1u;
    }
    if (strncmp(frame, "$CMD,TYPE=RESET", 15) == 0) {
        out_evt->type = EVT_CMD_RESET;
        return 1u;
    }

    /* SENSOR frames — debounced and SPOF-checked before dispatching */
    if (strncmp(frame, "$SENSOR,OBSTRUCTION=", 20) == 0) {
        uint8_t val = (uint8_t)atoi(frame + 20);
        if (debounce_update(&s_dbc_obstruction, val)) {
            out_evt->type    = val ? EVT_OBSTRUCTION_DETECTED : EVT_OBSTRUCTION_CLEAR;
            out_evt->payload = val;
            return 1u;
        }
        out_evt->type = EVT_UNKNOWN;
        return 0u;
    }
    if (strncmp(frame, "$SENSOR,FULLY_OPEN=", 19) == 0) {
        uint8_t val = (uint8_t)atoi(frame + 19);
        if (debounce_update(&s_dbc_fully_open, val)) {
            /* Notify safety monitor for SPOF cross-check. */
            safety_monitor_update_sensors(s_dbc_fully_open.stable_state,
                                          s_dbc_fully_closed.stable_state);
            if (spof_check(s_dbc_fully_open.stable_state,
                           s_dbc_fully_closed.stable_state)) {
                ESP_LOGE(TAG, "SPOF: fully_open && fully_closed simultaneously!");
                out_evt->type   = EVT_SPOF_DETECTED;
                out_evt->source = SRC_HAL_SENSOR;
                return 1u;
            }
            if (val) {
                out_evt->type = EVT_SENSOR_FULLY_OPEN;
                return 1u;
            }
        }
        out_evt->type = EVT_UNKNOWN;
        return 0u;
    }
    if (strncmp(frame, "$SENSOR,FULLY_CLOSED=", 21) == 0) {
        uint8_t val = (uint8_t)atoi(frame + 21);
        if (debounce_update(&s_dbc_fully_closed, val)) {
            safety_monitor_update_sensors(s_dbc_fully_open.stable_state,
                                          s_dbc_fully_closed.stable_state);
            if (spof_check(s_dbc_fully_open.stable_state,
                           s_dbc_fully_closed.stable_state)) {
                ESP_LOGE(TAG, "SPOF: fully_open && fully_closed simultaneously!");
                out_evt->type   = EVT_SPOF_DETECTED;
                out_evt->source = SRC_HAL_SENSOR;
                return 1u;
            }
            if (val) {
                out_evt->type = EVT_SENSOR_FULLY_CLOSED;
                return 1u;
            }
        }
        out_evt->type = EVT_UNKNOWN;
        return 0u;
    }

    out_evt->type = EVT_UNKNOWN;
    return 0u;
}

esp_err_t hal_init(QueueHandle_t dispatcher_queue)
{
    if (dispatcher_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    s_dispatcher_queue = dispatcher_queue;

    /* Create TX queue */
    s_tx_queue = xQueueCreate(UART_TX_QUEUE_DEPTH, UART_FRAME_MAX_LEN);
    if (s_tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        return ESP_ERR_NO_MEM;
    }

    /* Configure UART driver. */
    const uart_config_t uart_cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(UART_PORT_NUM,
                                        UART_RX_BUF_SIZE,
                                        UART_TX_BUF_SIZE,
                                        0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(UART_PORT_NUM, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "HAL initialised (UART%d @ %d baud)", UART_PORT_NUM, UART_BAUD_RATE);
    return ESP_OK;
}

/* ---------------------------------------------------------------------------
 * UART RX Task
 * --------------------------------------------------------------------------- */
void hal_uart_rx_task(void *pvParameters)
{
    (void)pvParameters;

    static uint8_t rx_buf[UART_FRAME_MAX_LEN]; // latest byte read
    static char    frame_buf[UART_FRAME_MAX_LEN]; // accumulating current frame
    size_t frame_len = 0u;

    ESP_LOGI(TAG, "HAL RX task started (Core %d)", xPortGetCoreID());

    for (;;) {
        /* Read one byte at a time; accumulate until newline. */
        int bytes = uart_read_bytes(UART_PORT_NUM, rx_buf, 1u,
                                    pdMS_TO_TICKS(COMM_TIMEOUT_MS));
        if (bytes <= 0) {
            system_event_t timeout_evt = {
                .type         = EVT_COMM_TIMEOUT,
                .source       = SRC_HAL_SENSOR,
                .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
                .payload      = 0u,
            };
            if (dispatcher_post_event(s_dispatcher_queue, &timeout_evt) != pdTRUE) {
                ESP_LOGW(TAG, "Dispatcher queue OVERFLOW on comm-timeout event");
            }
            frame_len = 0u; 
            continue;
        }

        uint8_t byte = rx_buf[0];

        /* Newline terminates a frame. */
        if (byte == '\n' || byte == '\r') {
            if (frame_len == 0u) {
                continue; 
            }
            frame_buf[frame_len] = '\0';

            uint8_t crc_valid = 0u;
            if (frame_len > 3u && frame_buf[frame_len - 3u] == ',') {
                /* Extract declared CRC byte from the trailing ",XX". */
                char crc_hex[3] = { frame_buf[frame_len - 2u],
                                    frame_buf[frame_len - 1u], '\0' };
                uint8_t declared_crc = (uint8_t)strtol(crc_hex, NULL, 16);
                /* Compute CRC over the frame body (excluding the ",XX" suffix). */
                uint8_t computed_crc = crc8_compute((const uint8_t *)frame_buf,
                                                    frame_len - 3u);
                if (declared_crc == computed_crc) {
                    crc_valid = 1u;
                    /* Strip CRC suffix before parsing. */
                    frame_buf[frame_len - 3u] = '\0';
                    frame_len -= 3u;
                }
            } else {
                ESP_LOGW(TAG, "Frame has no CRC suffix — accepted in sim mode: %s",
                         frame_buf);
                crc_valid = 1u;
            }

            if (!crc_valid) {
                ESP_LOGW(TAG, "CRC mismatch — sending NACK for: %s", frame_buf);
                hal_send_nack();
                frame_len = 0u;
                continue;
            }

            /* Parse the frame -> typed event. */
            system_event_t evt = {0};
            uint8_t parsed = parse_frame(frame_buf, &evt);

            if (!parsed || evt.type == EVT_UNKNOWN) {
                ESP_LOGW(TAG, "Unknown frame: %s — NACK", frame_buf);
                hal_send_nack();
                frame_len = 0u;
                continue;
            }

            /* Valid frame — ACK and reset comm-timeout watchdog. */
            hal_send_ack();
            safety_monitor_reset_comm_watchdog();

            /* Push to Dispatcher. Log overflow. */
            if (dispatcher_post_event(s_dispatcher_queue, &evt) != pdTRUE) {
                ESP_LOGW(TAG, "Dispatcher queue OVERFLOW — event type 0x%02X dropped",
                         (unsigned)evt.type);
            }

            frame_len = 0u;
        } else {
            if (frame_len < (UART_FRAME_MAX_LEN - 1u)) {
                frame_buf[frame_len++] = (char)byte;
            } else {
                /* Frame too long — discard */
                ESP_LOGW(TAG, "Frame too long — discarded");
                frame_len = 0u;
            }
        }
    }
}

/* ---------------------------------------------------------------------------
 * UART TX Task 
 * --------------------------------------------------------------------------- */
void hal_uart_tx_task(void *pvParameters)
{
    (void)pvParameters;
    static char tx_msg[UART_FRAME_MAX_LEN];

    ESP_LOGI(TAG, "HAL TX task started (Core %d)", xPortGetCoreID());

    for (;;) {
        /* Block until a TX message is available. */
        if (xQueueReceive(s_tx_queue, tx_msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(UART_PORT_NUM, tx_msg, strlen(tx_msg));
        }
    }
}

/* ---------------------------------------------------------------------------
 * Actuator Commands
 * --------------------------------------------------------------------------- */
void hal_motor_open(void)
{
    /* (hardware): code */
    ESP_LOGI(TAG, "[ACTUATOR] Motor → OPEN");
    hal_tx_enqueue("$ACTUATOR,CMD=OPEN\n");
}

void hal_motor_close(void)
{
    /* (hardware): code */
    ESP_LOGI(TAG, "[ACTUATOR] Motor → CLOSE");
    hal_tx_enqueue("$ACTUATOR,CMD=CLOSE\n");
}

void hal_motor_stop(void)
{
    /* (hardware): code */
    ESP_LOGI(TAG, "[ACTUATOR] Motor → STOP");
    hal_tx_enqueue("$ACTUATOR,CMD=STOP\n");
}

/* ---------------------------------------------------------------------------
 * TX Enqueue 
 * --------------------------------------------------------------------------- */
BaseType_t hal_tx_enqueue(const char *state_str)
{
    if (s_tx_queue == NULL) {
        return pdFALSE;
    }

    static char msg_buf[UART_FRAME_MAX_LEN];
    
    strncpy(msg_buf, state_str, UART_FRAME_MAX_LEN - 1u);
    msg_buf[UART_FRAME_MAX_LEN - 1u] = '\0';

    BaseType_t result = xQueueSendToBack(s_tx_queue, msg_buf, 0u);
    if (result != pdTRUE) {
        /* Queue full: drop oldest */
        static char dropped[UART_FRAME_MAX_LEN];
        xQueueReceive(s_tx_queue, dropped, 0u); /* Discard oldest. */
        ESP_LOGW(TAG, "TX queue full — dropped oldest: %s", dropped);
        result = xQueueSendToBack(s_tx_queue, msg_buf, 0u);
    }
    return result;
}

void hal_send_ack(void)
{
    hal_tx_enqueue("$ACK\n");
}

void hal_send_nack(void)
{
    hal_tx_enqueue("$NACK\n");
}
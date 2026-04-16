// =============================================================
// uart/uart.cpp — UART receive and transmit tasks
//
// RX task:
//   • Assembles newline-terminated frames into a fixed buffer
//   • Validates CRC-8 via protocol module
//   • Tracks monotonically-increasing sequence numbers;
//     accepts SEQ=1 as a "reset" from the supervisor
//   • Maps command strings to FsmEvents and posts to eventQueue
//   • Sends ACK / NACK via txQueue (non-blocking)
//   • Updates g_lastCmdTimeMs on every accepted command
//
// TX task:
//   • Drains txQueue and writes each message to Serial
// =============================================================
#include "uart.h"
#include "config.h"
#include "system.h"
#include "fsm/fsm.h"
#include "protocol/protocol.h"

#include <Arduino.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Spinlock for protecting g_obstructionActive and g_lastCmdTimeMs
// from both the UART RX task and the Safety task.
static portMUX_TYPE s_sharedMux = portMUX_INITIALIZER_UNLOCKED;

// -------------------------------------------------------
void uart_init(void)
{
    Serial.begin(UART_BAUD);
    // delay() is safe here; called from setup() before scheduler starts
    delay(100);
}

// -------------------------------------------------------
// uart_rx_task
// -------------------------------------------------------
void uart_rx_task(void *param)
{
    (void)param;

    static char rxBuf[UART_BUF_SIZE];
    static int  rxIdx = 0;
    static int32_t lastSeq = 0;   // Sequence tracking

    for (;;) {
        // Drain all available bytes this tick
        while (Serial.available() > 0) {
            char c = (char)Serial.read();

            if (c == '\r') continue;  // ignore CR

            if (c == '\n') {
                // Frame complete — process it
                rxBuf[rxIdx] = '\0';

                if (rxIdx > 0 && rxBuf[0] == PROTO_START_CHAR) {
                    ParsedMsg msg = proto_parse(rxBuf);

                    if (!msg.valid) {
                        // Send NACK — bad CRC or malformed
                        char nack[TX_MSG_MAX];
                        proto_format_nack(nack, sizeof(nack),
                                          "CRC_OR_FORMAT_ERROR", lastSeq);
                        xQueueSend(g_txQueue, nack, 0);
                        log_post("RX: invalid frame (CRC calc=%02X recv=%02X)",
                                 msg.crc_calc, msg.crc_recv);
                    } else {
                        // Sequence number check
                        // Accept: seq == lastSeq+1 (normal) OR seq == 1 (reset)
                        bool seqOk = (msg.seq == lastSeq + 1) ||
                                     (msg.seq == 1);
                        if (!seqOk) {
                            char nack[TX_MSG_MAX];
                            proto_format_nack(nack, sizeof(nack),
                                              "SEQ_ERROR", msg.seq);
                            xQueueSend(g_txQueue, nack, 0);
                            log_post("RX: seq error (expected %d got %d)",
                                     lastSeq + 1, (int)msg.seq);
                        } else {
                            // Accepted — update tracking
                            lastSeq = msg.seq;

                            // Update comm-timeout timer (thread-safe write)
                            portENTER_CRITICAL(&s_sharedMux);
                            g_lastCmdTimeMs = (uint32_t)millis();
                            portEXIT_CRITICAL(&s_sharedMux);

                            // Map command string → FsmEvent
                            FsmEvent ev = EVT_NONE;
                            if      (strcmp(msg.cmd, "OPEN")           == 0) ev = EVT_CMD_OPEN;
                            else if (strcmp(msg.cmd, "CLOSE")          == 0) ev = EVT_CMD_CLOSE;
                            else if (strcmp(msg.cmd, "STOP")           == 0) ev = EVT_CMD_STOP;
                            else if (strcmp(msg.cmd, "EMERGENCY_OPEN") == 0) ev = EVT_CMD_EMERGENCY_OPEN;
                            else if (strcmp(msg.cmd, "RESET")          == 0) ev = EVT_CMD_RESET;
                            // Sensor simulation commands
                            else if (strcmp(msg.cmd, "OBS_ON")         == 0) {
                                portENTER_CRITICAL(&s_sharedMux);
                                g_obstructionActive = true;
                                portEXIT_CRITICAL(&s_sharedMux);
                                log_post("SIM: obstruction ON");
                            }
                            else if (strcmp(msg.cmd, "OBS_OFF")        == 0) {
                                portENTER_CRITICAL(&s_sharedMux);
                                g_obstructionActive = false;
                                portEXIT_CRITICAL(&s_sharedMux);
                                log_post("SIM: obstruction OFF");
                                ev = EVT_OBSTRUCTION_CLEAR;
                            }

                            if (ev == EVT_NONE &&
                                strcmp(msg.cmd, "OBS_ON") != 0 &&
                                strcmp(msg.cmd, "OBS_OFF") != 0) {
                                // Unknown command
                                char nack[TX_MSG_MAX];
                                proto_format_nack(nack, sizeof(nack),
                                                  "UNKNOWN_CMD", msg.seq);
                                xQueueSend(g_txQueue, nack, 0);
                                log_post("RX: unknown cmd '%s'", msg.cmd);
                            } else {
                                // Send ACK
                                char ack[TX_MSG_MAX];
                                proto_format_ack(ack, sizeof(ack),
                                                 msg.cmd, msg.seq);
                                xQueueSend(g_txQueue, ack, 0);

                                if (ev != EVT_NONE) {
                                    fsm_post_event(ev);
                                    log_post("RX: accepted %s (seq=%d)",
                                             msg.cmd, (int)msg.seq);
                                }
                            }
                        }
                    }
                }

                // Reset buffer
                rxIdx = 0;
            } else {
                // Accumulate character
                if (rxIdx < (int)(UART_BUF_SIZE - 1)) {
                    rxBuf[rxIdx++] = c;
                } else {
                    // Buffer overflow — discard frame and reset
                    log_post("RX: buffer overflow, frame discarded");
                    rxIdx = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(UART_RX_PERIOD_MS));
    }
}

// -------------------------------------------------------
// uart_tx_task — drains g_txQueue to Serial
// -------------------------------------------------------
void uart_tx_task(void *param)
{
    (void)param;
    static char buf[TX_MSG_MAX];

    for (;;) {
        // Block until a message arrives (with 200 ms timeout)
        if (xQueueReceive(g_txQueue, buf, pdMS_TO_TICKS(200)) == pdTRUE) {
            // Write all queued messages without yielding between them
            Serial.print(buf);
            // Drain any additional messages already queued
            while (xQueueReceive(g_txQueue, buf, 0) == pdTRUE) {
                Serial.print(buf);
            }
        }
    }
}

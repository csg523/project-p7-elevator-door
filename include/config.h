// =============================================================
// config.h — System-wide constants
// =============================================================
#pragma once

// -------------------------------------------------------
// TIMING (all in milliseconds)
// -------------------------------------------------------
#define COMM_TIMEOUT_MS 2000u         // No valid cmd → FAULT
#define DOOR_MOTION_SIM_MS 600u       // Simulated door travel time (very fast for instant obstruction detection)
#define STATE_REPORT_INTERVAL_MS 200u // Periodic $STATE transmit rate
#define SAFETY_POLL_MS 5u             // Safety task polling interval
#define FSM_TASK_PERIOD_MS 10u        // FSM task loop period
#define UART_RX_PERIOD_MS 5u          // UART RX task period
#define WATCHDOG_PERIOD_MS 500u       // Software watchdog check period
#define WATCHDOG_TIMEOUT_MS 3000u     // Kick window for FSM task

// -------------------------------------------------------
// QUEUE CAPACITIES
// -------------------------------------------------------
#define EVT_QUEUE_SIZE 20
#define TX_QUEUE_SIZE 50
#define LOG_QUEUE_SIZE 30

// -------------------------------------------------------
// BUFFER SIZES (bytes)
// -------------------------------------------------------
#define UART_BUF_SIZE 128
#define LOG_MSG_MAX 96
#define TX_MSG_MAX 96

// -------------------------------------------------------
// TASK STACK SIZES (bytes — ESP32 Arduino uses bytes)
// -------------------------------------------------------
#define STACK_UART_RX 3072
#define STACK_FSM 4096
#define STACK_SAFETY 2048
#define STACK_UART_TX 2048
#define STACK_LOGGER 2048
#define STACK_WATCHDOG 2048
#define STACK_DISPLAY 4096

// -------------------------------------------------------
// TASK PRIORITIES  (1 = lowest, 5 = highest used here)
// -------------------------------------------------------
#define PRI_LOGGER 1
#define PRI_UART_TX 2
#define PRI_UART_RX 3
#define PRI_WATCHDOG 3
#define PRI_FSM 4
#define PRI_SAFETY 5

// -------------------------------------------------------
// PROTOCOL
// -------------------------------------------------------
#define PROTO_START_CHAR '$'
#define PROTO_END_CHAR '\n'
// CRC-8 polynomial (Dallas/Maxim 1-Wire)
#define CRC8_POLY 0x8Cu

// -------------------------------------------------------
// HARDWARE
// -------------------------------------------------------
#define UART_BAUD 115200

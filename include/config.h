#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define PRIORITY_SAFETY_TASK        5u
#define PRIORITY_DISPATCHER_TASK    4u
#define PRIORITY_CONTROL_TASK       4u
#define PRIORITY_HAL_INPUT_TASK     3u
#define PRIORITY_UART_TX_TASK       2u
#define PRIORITY_LOGGER_TASK        1u
#define PRIORITY_DISPLAY_TASK       1u

/* ---------------------------------------------------------------------------
 * FreeRTOS Core Assignments (Dual-Core ESP32)
 * Core 0: HAL / Comms / Logging 
 * Core 1: Safety / Dispatcher / FSM
 * --------------------------------------------------------------------------- */
#define CORE_COMMS      0
#define CORE_SAFETY     1


#define STACK_SAFETY_TASK       3072u
#define STACK_DISPATCHER_TASK   3072u
#define STACK_CONTROL_TASK      4096u  /* FSM + NVS calls need more stack */
#define STACK_HAL_INPUT_TASK    3072u
#define STACK_UART_TX_TASK      2048u
#define STACK_LOGGER_TASK       2048u
#define STACK_DISPLAY_TASK      3072u

#define DISPATCHER_QUEUE_DEPTH  32u
#define UART_TX_QUEUE_DEPTH     16u
#define LOGGER_QUEUE_DEPTH      16u

/* ---------------------------------------------------------------------------
 * Communication & Timing
 * --------------------------------------------------------------------------- */
#define COMM_TIMEOUT_MS         2000000u

#define STATE_REPORT_INTERVAL_MS 7000u

#define OBSTRUCTION_REACT_MS    100u

#define MOTOR_STALL_TIMEOUT_MS  100000u

#define CLOCK_DRIFT_MARGIN_PCT  20u

/* ---------------------------------------------------------------------------
 * Debounce
 * --------------------------------------------------------------------------- */
#define SENSOR_DEBOUNCE_MS      2000u

/* ---------------------------------------------------------------------------
 * UART Configuration
 * --------------------------------------------------------------------------- */
#define UART_PORT_NUM           UART_NUM_0
#define UART_BAUD_RATE          115200
#define UART_RX_BUF_SIZE        512u
#define UART_TX_BUF_SIZE        512u
#define UART_FRAME_MAX_LEN      64u

/* ---------------------------------------------------------------------------
 * NVS Namespace & Keys
 * --------------------------------------------------------------------------- */
#define NVS_NAMESPACE           "elev_door"
#define NVS_KEY_FAULT_STATE     "fault_state"
#define NVS_KEY_FAULT_CODE      "fault_code"

/* ---------------------------------------------------------------------------
 * TWDT (Task Watchdog Timer)
 * --------------------------------------------------------------------------- */
#define TWDT_FEED_INTERVAL_MS   1000u

/* ---------------------------------------------------------------------------
 * CRC-8 polynomial.
 * --------------------------------------------------------------------------- */
#define CRC8_POLYNOMIAL         0x31u
#define CRC8_INITIAL            0xFFu


#define LOG_MSG_MAX_LEN         128u
#define STACK_REPORT_INTERVAL_MS 5000u

#endif 
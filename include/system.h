// =============================================================
// system.h — Global FreeRTOS handle declarations
// Defined in main.cpp; all modules include this header.
// =============================================================
//preprocessor directive that ensures the header file is only included once.
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// -------------------------------------------------------
// Inter-task queues
// -------------------------------------------------------
extern QueueHandle_t g_eventQueue; // FsmEvent  → FSM task
extern QueueHandle_t g_txQueue;    // char[TX_MSG_MAX] → UART TX task
extern QueueHandle_t g_logQueue;   // LogEntry  → Logger task

// -------------------------------------------------------
// Shared synchronisation
// -------------------------------------------------------
extern SemaphoreHandle_t g_stateMutex; // Protects FSM state reads

// -------------------------------------------------------
// Cross-task flags (written atomically via portENTER_CRITICAL)
// -------------------------------------------------------
extern volatile bool g_obstructionActive; // Set by Safety task
extern volatile uint32_t g_lastCmdTimeMs; // Updated by UART RX task
extern volatile uint32_t g_fsmTickCount; // Updated by FSM task
// -------------------------------------------------------
// Task handles (for watchdog monitoring)
// -------------------------------------------------------
extern TaskHandle_t g_hFsmTask;
extern TaskHandle_t g_hSafetyTask;
extern TaskHandle_t g_hUartRxTask;
extern TaskHandle_t g_hUartTxTask;
extern TaskHandle_t g_hLoggerTask;

// -------------------------------------------------------
// Convenience: post a formatted log string from any task.
// Non-blocking — if the log queue is full the message is dropped.
// Defined in logger.cpp.
// -------------------------------------------------------
void log_post(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

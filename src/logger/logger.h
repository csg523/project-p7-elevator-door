// =============================================================
// logger/logger.h — Asynchronous, queue-based event logger
//
// Callers use log_post() (declared in system.h) which is
// non-blocking and safe from any task context.
// The logger_task drains the queue and prints to Serial.
// =============================================================
#pragma once

#include <stdint.h>

// Log entry stored in queue
typedef struct {
    uint32_t timestampMs;
    char     msg[96];
} LogEntry;

void logger_init(void);
void logger_task(void *param);

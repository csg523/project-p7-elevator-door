// =============================================================
// logger/logger.cpp — Non-blocking queue-based event logger
// =============================================================
#include "logger.h"
#include "system.h"
#include "config.h"

#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

void logger_init(void)
{
    // Queue is created in main.cpp before tasks start
}

// -------------------------------------------------------
// log_post — non-blocking, callable from any task
// Defined here (declared extern in system.h)
// -------------------------------------------------------
void log_post(const char *fmt, ...)
{
    LogEntry entry;
    entry.timestampMs = (uint32_t)millis();

    va_list args;
    va_start(args, fmt);
    vsnprintf(entry.msg, sizeof(entry.msg), fmt, args);
    va_end(args);

    // Non-blocking send: if queue is full the log is silently dropped
    // (logging must NEVER block control or safety tasks)
    if (g_logQueue != NULL) {
        xQueueSend(g_logQueue, &entry, 0);
    }
}

// -------------------------------------------------------
// logger_task — lowest-priority, drains queue to Serial
// -------------------------------------------------------
void logger_task(void *param)
{
    (void)param;
    LogEntry entry;

    for (;;) {
        if (xQueueReceive(g_logQueue, &entry, pdMS_TO_TICKS(500)) == pdTRUE) {
            // Format: [LOG  12345] message
            Serial.printf("[LOG %7lu] %s\n",
                          (unsigned long)entry.timestampMs,
                          entry.msg);
            // Drain burst without sleeping between entries
            while (xQueueReceive(g_logQueue, &entry, 0) == pdTRUE) {
                Serial.printf("[LOG %7lu] %s\n",
                              (unsigned long)entry.timestampMs,
                              entry.msg);
            }
        }
    }
}

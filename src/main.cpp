// =============================================================
// main.cpp — System entry point
//
// Responsibilities:
//   • Define all global RTOS handles
//   • Initialise modules in correct order
//   • Create FreeRTOS tasks with proper priorities and stacks
//   • Run a software watchdog task that resets the FSM if the
//     FSM task stops making progress
// =============================================================
#include <Arduino.h>
#include "config.h"
#include "system.h"

#include "fsm/fsm.h"
#include "uart/uart.h"
#include "safety/safety.h"
#include "logger/logger.h"
#include "motor/motor.h"
#include "display/display.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// =============================================================
// Global handle definitions (declared extern in system.h)
// =============================================================
QueueHandle_t g_eventQueue = NULL;
QueueHandle_t g_txQueue = NULL;
QueueHandle_t g_logQueue = NULL;
SemaphoreHandle_t g_stateMutex = NULL;

volatile bool g_obstructionActive = false;
volatile uint32_t g_lastCmdTimeMs = 0;
volatile uint32_t g_fsmTickCount = 0;

TaskHandle_t g_hFsmTask = NULL;
TaskHandle_t g_hSafetyTask = NULL;
TaskHandle_t g_hUartRxTask = NULL;
TaskHandle_t g_hUartTxTask = NULL;
TaskHandle_t g_hLoggerTask = NULL;

// =============================================================
// Software watchdog task
// Checks that the FSM task is alive by examining whether it
// has run at all (task state != suspended/deleted).
// If the FSM appears wedged, posts a SYSTEM_FAULT event.
// =============================================================
static void watchdog_task(void *param)
{
    (void)param;
    uint32_t lastFsmTick = 0;
    uint8_t stuckCount = 0;
    bool commFaultTriggered = false; // Prevent spamming the event queue

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(WATCHDOG_PERIOD_MS));
        uint32_t now = (uint32_t)millis();

        if (g_hFsmTask == NULL)
            continue;

        // 1. Check FSM Task Health (Our previous fix)
        if (g_fsmTickCount == lastFsmTick)
        {
            stuckCount++;
            if (stuckCount >= 6)
            {
                // ~3 s of no FSM activity trigger
                FsmEvent fe = EVT_SYSTEM_FAULT;
                // non blocking send to avoid watchdog task getting stuck
                xQueueSend(g_eventQueue, &fe, 0);
                log_post("WATCHDOG: FSM task appears stuck");
                stuckCount = 0;
            }
        }
        else
        {
            stuckCount = 0;
        }
        lastFsmTick = g_fsmTickCount;

        // 2. Check Communication Timeout (SR-5)
        // If the time since the last valid command exceeds the limit
        if ((now - g_lastCmdTimeMs) > COMM_TIMEOUT_MS)
        {
            if (!commFaultTriggered)
            {
                FsmEvent fe = EVT_COMM_TIMEOUT;
                // non blocking send to avoid watchdog task getting stuck
                xQueueSend(g_eventQueue, &fe, 0);
                log_post("WATCHDOG: Comm timeout detected");
                commFaultTriggered = true;
            }
        }
        else
        {
            // Valid command received recently, reset the latch
            commFaultTriggered = false;
        }
    }
}

// =============================================================
// setup — runs once on core 1 before FreeRTOS scheduler starts
// =============================================================
// arduino entry point
void setup(void)
{
    // 1. Serial / UART
    uart_init();

    // Small boot delay so the serial monitor can connect
    delay(1500);
    Serial.println("\n=== Elevator Door Safety Controller ===");
    Serial.println("FreeRTOS / ESP32 / UART supervisory control");
    Serial.println("========================================\n");

    // 2. Create RTOS primitives BEFORE any task that uses them
    g_logQueue = xQueueCreate(LOG_QUEUE_SIZE, sizeof(LogEntry));
    g_eventQueue = xQueueCreate(EVT_QUEUE_SIZE, sizeof(FsmEvent));
    g_txQueue = xQueueCreate(TX_QUEUE_SIZE, TX_MSG_MAX);
    g_stateMutex = xSemaphoreCreateMutex();

    // if queue or mutex creation failed, halt here (critical failure)
    configASSERT(g_logQueue);
    configASSERT(g_eventQueue);
    configASSERT(g_txQueue);
    configASSERT(g_stateMutex);

    // 3. Initialise modules
    logger_init();
    motor_init();
    safety_init();
    display_init();
    fsm_init();

    // Seed comm-timeout timer so we don't immediately fault
    g_lastCmdTimeMs = (uint32_t)millis();

    // 4. Create FreeRTOS tasks
    //    Priority order: Safety(5) > FSM(4) > UartRx(3) ≈ Watchdog(3)
    //                  > UartTx(2) > Logger(1)
    xTaskCreate(logger_task, "Logger", STACK_LOGGER, NULL, PRI_LOGGER, &g_hLoggerTask);
    xTaskCreate(uart_tx_task, "UART_TX", STACK_UART_TX, NULL, PRI_UART_TX, &g_hUartTxTask);
    xTaskCreate(uart_rx_task, "UART_RX", STACK_UART_RX, NULL, PRI_UART_RX, &g_hUartRxTask);
    xTaskCreate(watchdog_task, "Watchdog", STACK_WATCHDOG, NULL, PRI_WATCHDOG, NULL);
    xTaskCreate(fsm_task, "FSM", STACK_FSM, NULL, PRI_FSM, &g_hFsmTask);
    xTaskCreate(safety_task, "Safety", STACK_SAFETY, NULL, PRI_SAFETY, &g_hSafetyTask);

    // Display task (lower priority, fine to miss frames)
    xTaskCreate(display_task, "Display", STACK_DISPLAY, NULL, PRI_LOGGER, NULL);

    log_post("System: all tasks created, scheduler running");
}

// loop() is called by the Arduino FreeRTOS idle task on core 1.
// Nothing should block here.
void loop(void)
{
    // max value of a tick count is ~49 days
    vTaskDelay(portMAX_DELAY);
}

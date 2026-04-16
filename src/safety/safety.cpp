// =============================================================
// safety/safety.cpp — Real-time safety monitoring (highest priority)
//
// Safety decisions are made here independently of all other tasks.
// This task posts directly to g_eventQueue so the FSM reacts
// within one FSM task cycle (≤ FSM_TASK_PERIOD_MS = 10 ms).
//
// Monitored conditions:
//   1. Obstruction during CLOSING → EVT_OBSTRUCTION
//   2. Communication timeout      → EVT_COMM_TIMEOUT
// =============================================================
#include "safety.h"
#include "config.h"
#include "system.h"
#include "fsm/fsm.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void safety_init(void)
{
    // Nothing to initialise in simulation mode.
    // In hardware: configure sensor GPIO interrupts here.
    log_post("Safety: initialized");
}

void safety_task(void *param)
{
    (void)param;

    bool obsPrev = false;
    bool commFaultSent = false;
    uint32_t lastPostEventMs = 0; // Track when we last posted an EVENT to prevent rapid-fire

    for (;;)
    {
        // ---- Read shared flags (atomic snapshot) ----
        bool obsNow = g_obstructionActive;
        uint32_t lastCmd = g_lastCmdTimeMs;
        uint32_t now = (uint32_t)millis();

        DoorState state = fsm_get_state();

        // ---- SR-1 / SR-2: Obstruction detection ----
        if (obsNow && !obsPrev)
        {
            // Rising edge — obstruction just activated
            if (state == STATE_CLOSING)
            {
                log_post("SAFETY: obstruction detected during CLOSING");
                fsm_post_event(EVT_OBSTRUCTION);
                lastPostEventMs = now;
            }
        }
        // Rate-limit obstruction re-posting to every 10ms minimum
        if (obsNow && state == STATE_CLOSING && (now - lastPostEventMs) >= 10)
        {
            fsm_post_event(EVT_OBSTRUCTION);
            lastPostEventMs = now;
        }
        obsPrev = obsNow;

        // ---- SR-5: Communication timeout ----
        // Only monitor timeout while door is in MOTION (moving).
        // When stationary (IDLE, OPEN, CLOSED), timeout doesn't apply.
        bool inMotion = (state == STATE_OPENING || state == STATE_CLOSING);
        bool timedOut = inMotion && ((now - lastCmd) > COMM_TIMEOUT_MS);

        if (timedOut && !commFaultSent && state != STATE_FAULT &&
            state != STATE_EMERGENCY)
        {
            log_post("SAFETY: comm timeout (%lu ms) during motion", (unsigned long)(now - lastCmd));
            fsm_post_event(EVT_COMM_TIMEOUT);
            commFaultSent = true;
        }
        // Reset the latch when a command is received (lastCmd updated) OR when door stops moving
        if (!timedOut || !inMotion)
        {
            commFaultSent = false;
        }

        (void)inMotion; // reserved for future timing-specific logic

        vTaskDelay(pdMS_TO_TICKS(SAFETY_POLL_MS));
    }
}
// =============================================================
// fsm/fsm.cpp — Deterministic elevator door state machine
//
// Design decisions:
//   • State owned exclusively by fsm_task; other tasks READ via
//     fsm_get_state() (mutex-protected, non-blocking snapshot).
//   • Door motion is *simulated*: after DOOR_MOTION_SIM_MS ms
//     in OPENING/CLOSING the FSM self-generates the position event.
//   • EMERGENCY_OPEN is handled before the state dispatch so it
//     overrides any state without an explicit guard.
//   • Every state transition runs an entry-action (motor command,
//     log, TX report).
// =============================================================
#include "fsm.h"
#include "config.h"
#include "system.h"
#include "motor/motor.h"
#include "protocol/protocol.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// -------------------------------------------------------
// Private state
// -------------------------------------------------------
static volatile DoorState s_state = STATE_IDLE;
static uint32_t s_entryMs = 0;         // millis() on last entry
static bool s_motionEvtPosted = false; // prevent double-post of events

// -------------------------------------------------------
// Helpers
// -------------------------------------------------------
static void enter_state(DoorState newState)
{
    if (s_state == newState)
    {
        // Prevent duplicate state entry + TX spam
        return;
    }

    s_state = newState;
    s_entryMs = (uint32_t)millis();
    s_motionEvtPosted = false;

    // Entry actions
    switch (newState)
    {
    case STATE_IDLE:
        motor_stop();
        break;
    case STATE_OPENING:
        motor_open();
        break;
    case STATE_OPEN:
        motor_stop();
        break;
    case STATE_CLOSING:
        motor_close();
        break;
    case STATE_CLOSED:
        motor_stop();
        break;
    case STATE_STOPPED:
        motor_stop();
        break;
    case STATE_EMERGENCY:
        motor_open();
        break;
    case STATE_FAULT:
        motor_stop();
        break;
    default:
        motor_stop();
        break;
    }

    // Report new state to supervisor via TX queue
    char buf[TX_MSG_MAX];
    proto_format_state(buf, sizeof(buf), newState);
    // non blocking send; if queue is full; dont stall FSM
    if (xQueueSend(g_txQueue, buf, 0) != pdTRUE)
    {
        // TX queue full — not fatal, state will be re-sent on next periodic report
        log_post("WARN: TX queue full on state change");
    }

    log_post("STATE → %s", fsm_state_name(newState));
}

// -------------------------------------------------------
// fsm_init
// -------------------------------------------------------
bool fsm_init(void)
{
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    s_state = STATE_IDLE;
    s_entryMs = (uint32_t)millis();
    s_motionEvtPosted = false;
    xSemaphoreGive(g_stateMutex);

    motor_stop();
    log_post("FSM: initialized → IDLE");
    return true;
}

// -------------------------------------------------------
// fsm_get_state — thread-safe snapshot
// -------------------------------------------------------
DoorState fsm_get_state(void)
{
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    DoorState s = s_state;
    xSemaphoreGive(g_stateMutex);
    return s;
}

// -------------------------------------------------------
// fsm_post_event — callable from any task / ISR context
// -------------------------------------------------------
bool fsm_post_event(FsmEvent e)
{
    if (xQueueSend(g_eventQueue, &e, 0) == pdTRUE)
    {
        return true;
    }
    log_post("WARN: eventQueue full, dropped %s", fsm_event_name(e));
    return false;
}

// -------------------------------------------------------
// fsm_process — core transition logic (called from fsm_task)
// Must only be called from the FSM task context.
// -------------------------------------------------------
static void fsm_process(FsmEvent ev)
{
    DoorState cur = s_state;
    // Handled before state-specific logic since they override normal transitions
    // --------------------------------------------------
    // EMERGENCY_OPEN: highest priority, overrides any state
    // --------------------------------------------------
    if (ev == EVT_CMD_EMERGENCY_OPEN)
    {
        if (cur != STATE_EMERGENCY)
        {
            log_post("EMERGENCY_OPEN override from %s", fsm_state_name(cur));
            enter_state(STATE_EMERGENCY);
        }
        return;
    }

    // --------------------------------------------------
    // RESET: recovers from FAULT or EMERGENCY
    // --------------------------------------------------
    if (ev == EVT_CMD_RESET)
    {
        if (cur != STATE_IDLE)
        {
            log_post("RESET from %s", fsm_state_name(cur));
            enter_state(STATE_IDLE);
        }
        return;
    }

    // --------------------------------------------------
    // State-specific transitions
    // --------------------------------------------------
    switch (cur)
    {

    // ---- IDLE ----
    case STATE_IDLE:
        switch (ev)
        {
        case EVT_CMD_OPEN:
            enter_state(STATE_OPENING);
            break;
        case EVT_CMD_CLOSE:
            // Already closed / idle — acknowledge but no change
            log_post("FSM: CMD_CLOSE ignored in IDLE");
            break;
        case EVT_CMD_STOP:
            // Already stopped
            break;
        case EVT_COMM_TIMEOUT:
            log_post("FSM: COMM_TIMEOUT in IDLE → FAULT");
            enter_state(STATE_FAULT);
            break;
        case EVT_SYSTEM_FAULT:
        case EVT_SENSOR_FAULT:
            enter_state(STATE_FAULT);
            break;
        default:
            break;
        }
        break;

    // ---- OPENING ----
    case STATE_OPENING:
        switch (ev)
        {
        case EVT_CMD_OPEN:
            // Redundant — already opening, ignore
            break;
        case EVT_CMD_CLOSE:
            // Interrupt opening → closing
            log_post("FSM: CMD_CLOSE preempts OPENING");
            enter_state(STATE_CLOSING);
            break;
        case EVT_CMD_STOP:
            enter_state(STATE_STOPPED);
            break;
        case EVT_DOOR_FULLY_OPEN:
            enter_state(STATE_OPEN);
            break;
        case EVT_SENSOR_FAULT:
        case EVT_SYSTEM_FAULT:
            enter_state(STATE_FAULT);
            break;
        case EVT_COMM_TIMEOUT:
            // Keep opening — safer to finish than abort mid-motion
            log_post("FSM: COMM_TIMEOUT during OPENING — continuing");
            break;
        default:
            break;
        }
        break;

    // ---- OPEN ----
    case STATE_OPEN:
        switch (ev)
        {
        case EVT_CMD_OPEN:
            // Already open — ignore; log for diagnostics
            log_post("FSM: redundant OPEN in OPEN (staying OPEN)");
            break;
        case EVT_CMD_CLOSE:
            enter_state(STATE_CLOSING);
            break;
        case EVT_CMD_STOP:
            // No motion, but acknowledge
            break;
        case EVT_COMM_TIMEOUT:
            log_post("FSM: COMM_TIMEOUT in OPEN → FAULT");
            enter_state(STATE_FAULT);
            break;
        case EVT_SENSOR_FAULT:
        case EVT_SYSTEM_FAULT:
            enter_state(STATE_FAULT);
            break;
        default:
            break;
        }
        break;

    // ---- CLOSING ----
    case STATE_CLOSING:
        switch (ev)
        {
        case EVT_CMD_CLOSE:
            // Redundant — already closing
            break;
        case EVT_CMD_OPEN:
            // SR-3 preemption: CMD_OPEN > CMD_CLOSE
            log_post("FSM: CMD_OPEN preempts CLOSING");
            enter_state(STATE_OPENING);
            break;
        case EVT_CMD_STOP:
            enter_state(STATE_STOPPED);
            break;
        case EVT_OBSTRUCTION:
            // SR-1 / SR-2: immediate reversal
            log_post("FSM: OBSTRUCTION → reversing to OPENING");
            enter_state(STATE_OPENING);
            break;
        case EVT_DOOR_FULLY_CLOSED:
            enter_state(STATE_CLOSED);
            break;
        case EVT_SENSOR_FAULT:
        case EVT_SYSTEM_FAULT:
            enter_state(STATE_FAULT);
            break;
        case EVT_COMM_TIMEOUT:
            log_post("FSM: COMM_TIMEOUT during CLOSING → FAULT (stop)");
            enter_state(STATE_FAULT);
            break;
        default:
            break;
        }
        break;

    // ---- CLOSED ----
    case STATE_CLOSED:
        switch (ev)
        {
        case EVT_CMD_OPEN:
            enter_state(STATE_OPENING);
            break;
        case EVT_CMD_CLOSE:
            // Already closed — log for diagnostics
            log_post("FSM: redundant CLOSE in CLOSED (staying CLOSED)");
            break;
        case EVT_COMM_TIMEOUT:
            log_post("FSM: COMM_TIMEOUT in CLOSED → FAULT");
            enter_state(STATE_FAULT);
            break;
        case EVT_SENSOR_FAULT:
        case EVT_SYSTEM_FAULT:
            enter_state(STATE_FAULT);
            break;
        default:
            break;
        }
        break;

    // ---- STOPPED ----
    case STATE_STOPPED:
        switch (ev)
        {
        case EVT_CMD_OPEN:
            enter_state(STATE_OPENING);
            break;
        case EVT_CMD_CLOSE:
            enter_state(STATE_CLOSING);
            break;
        case EVT_COMM_TIMEOUT:
            enter_state(STATE_FAULT);
            break;
        case EVT_SENSOR_FAULT:
        case EVT_SYSTEM_FAULT:
            enter_state(STATE_FAULT);
            break;
        default:
            break;
        }
        break;

    // ---- EMERGENCY ----
    case STATE_EMERGENCY:
        // Stays until explicit RESET (handled above)
        // Do NOT allow CLOSE commands
        if (ev == EVT_CMD_CLOSE)
        {
            log_post("FSM: CMD_CLOSE rejected in EMERGENCY");
        }
        break;

    // ---- FAULT ----
    case STATE_FAULT:
        // No motion allowed; recovery via RESET (handled above)
        if (ev != EVT_CMD_RESET)
        {
            log_post("FSM: event %s ignored in FAULT (send RESET)",
                     fsm_event_name(ev));
        }
        break;

    default:
        break;
    }
}

// -------------------------------------------------------
// fsm_task — FreeRTOS task
// -------------------------------------------------------
void fsm_task(void *param)
{
    (void)param;
    FsmEvent ev;
    uint32_t lastReportMs = 0;
    //uint32_t lastWatchdogMs = (uint32_t)millis();

    for (;;)
    {
        uint32_t now = (uint32_t)millis();

        // ---- Kick software watchdog ----
        //lastWatchdogMs = now;
        // (void)lastWatchdogMs; // used by watchdog task via vTaskGetInfo
        g_fsmTickCount++;

        // ---- Process all pending events ----
        while (xQueueReceive(g_eventQueue, &ev, 0) == pdTRUE)
        {
            xSemaphoreTake(g_stateMutex, portMAX_DELAY);
            fsm_process(ev);
            xSemaphoreGive(g_stateMutex);
        }

        // ---- Simulate door motion completion ----
        {
            xSemaphoreTake(g_stateMutex, portMAX_DELAY);
            DoorState cur = s_state;
            uint32_t elapsed = now - s_entryMs;
            bool doPost = false;
            FsmEvent motionEv = EVT_NONE;

            if (!s_motionEvtPosted)
            {
                if (cur == STATE_OPENING && elapsed >= DOOR_MOTION_SIM_MS)
                {
                    doPost = true;
                    motionEv = EVT_DOOR_FULLY_OPEN;
                    s_motionEvtPosted = true;
                }
                else if (cur == STATE_CLOSING && elapsed >= DOOR_MOTION_SIM_MS)
                {
                    doPost = true;
                    motionEv = EVT_DOOR_FULLY_CLOSED;
                    s_motionEvtPosted = true;
                }
            }
            xSemaphoreGive(g_stateMutex);

            if (doPost)
            {
                // Re-enter to process immediately (avoids one-cycle delay)
                xSemaphoreTake(g_stateMutex, portMAX_DELAY);
                // fsm_process(motionEv);
                if ((s_state == STATE_OPENING && motionEv == EVT_DOOR_FULLY_OPEN) || (s_state == STATE_CLOSING && motionEv == EVT_DOOR_FULLY_CLOSED))
                {
                    fsm_process(motionEv);
                }
                else
                {
                    log_post("FSM: Dropped stale motion event %s in state %s", fsm_event_name(motionEv), fsm_state_name(s_state));
                }
                xSemaphoreGive(g_stateMutex);
            }
        }

        // ---- Periodic state report (FR-5) ----
        if (now - lastReportMs >= STATE_REPORT_INTERVAL_MS)
        {
            lastReportMs = now;
            char buf[TX_MSG_MAX];
            proto_format_state(buf, sizeof(buf), fsm_get_state());
            xQueueSend(g_txQueue, buf, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(FSM_TASK_PERIOD_MS));
    }
}

// -------------------------------------------------------
// String helpers
// -------------------------------------------------------
const char *fsm_state_name(DoorState s)
{
    switch (s)
    {
    case STATE_IDLE:
        return "IDLE";
    case STATE_OPENING:
        return "OPENING";
    case STATE_OPEN:
        return "OPEN";
    case STATE_CLOSING:
        return "CLOSING";
    case STATE_CLOSED:
        return "CLOSED";
    case STATE_STOPPED:
        return "STOPPED";
    case STATE_EMERGENCY:
        return "EMERGENCY";
    case STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

const char *fsm_event_name(FsmEvent e)
{
    switch (e)
    {
    case EVT_NONE:
        return "NONE";
    case EVT_CMD_OPEN:
        return "CMD_OPEN";
    case EVT_CMD_CLOSE:
        return "CMD_CLOSE";
    case EVT_CMD_STOP:
        return "CMD_STOP";
    case EVT_CMD_EMERGENCY_OPEN:
        return "CMD_EMERGENCY_OPEN";
    case EVT_CMD_RESET:
        return "CMD_RESET";
    case EVT_DOOR_FULLY_OPEN:
        return "DOOR_FULLY_OPEN";
    case EVT_DOOR_FULLY_CLOSED:
        return "DOOR_FULLY_CLOSED";
    case EVT_OBSTRUCTION:
        return "OBSTRUCTION";
    case EVT_OBSTRUCTION_CLEAR:
        return "OBSTRUCTION_CLEAR";
    case EVT_COMM_TIMEOUT:
        return "COMM_TIMEOUT";
    case EVT_SENSOR_FAULT:
        return "SENSOR_FAULT";
    case EVT_SYSTEM_FAULT:
        return "SYSTEM_FAULT";
    default:
        return "?";
    }
}

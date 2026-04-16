// =============================================================
// fsm/fsm.h — Finite State Machine for elevator door control
// =============================================================
#pragma once

#include <stdint.h>
#include <stdbool.h>

// -------------------------------------------------------
// Door states
// -------------------------------------------------------
typedef enum {
    STATE_IDLE      = 0,  // Powered on, no motion
    STATE_OPENING,        // Door actively moving to open
    STATE_OPEN,           // Door fully open
    STATE_CLOSING,        // Door actively moving to close
    STATE_CLOSED,         // Door fully closed
    STATE_STOPPED,        // Motion interrupted by CMD_STOP
    STATE_EMERGENCY,      // Emergency open — stays open until RESET
    STATE_FAULT,          // Fault detected — no motion until RESET
    STATE_COUNT
} DoorState;

// -------------------------------------------------------
// FSM events (commands + internal triggers)
// -------------------------------------------------------
typedef enum {
    EVT_NONE = 0,
    // External commands
    EVT_CMD_OPEN,
    EVT_CMD_CLOSE,
    EVT_CMD_STOP,
    EVT_CMD_EMERGENCY_OPEN,
    EVT_CMD_RESET,
    // Internal / sensor events
    EVT_DOOR_FULLY_OPEN,    // Simulated: door reached open position
    EVT_DOOR_FULLY_CLOSED,  // Simulated: door reached closed position
    EVT_OBSTRUCTION,        // Obstruction detected during closing
    EVT_OBSTRUCTION_CLEAR,  // Obstruction cleared
    EVT_COMM_TIMEOUT,       // No valid command within timeout window
    EVT_SENSOR_FAULT,       // Sensor inconsistency detected
    EVT_SYSTEM_FAULT,       // Generic system fault
    EVT_COUNT
} FsmEvent;

// -------------------------------------------------------
// Public API
// -------------------------------------------------------
bool        fsm_init(void);
DoorState   fsm_get_state(void);          // Thread-safe read
bool        fsm_post_event(FsmEvent e);   // Post from any task
const char *fsm_state_name(DoorState s);
const char *fsm_event_name(FsmEvent e);

// FreeRTOS task entry point (pass NULL as param)
void fsm_task(void *param);

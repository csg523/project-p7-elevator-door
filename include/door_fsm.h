#ifndef DOOR_FSM_H
#define DOOR_FSM_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"

/* ---------------------------------------------------------------------------
 * FSM States
 * --------------------------------------------------------------------------- */
typedef enum {
    FSM_STATE_INIT      = 0, 
    FSM_STATE_HOMING    = 1, 
    FSM_STATE_IDLE      = 2, 
    FSM_STATE_OPENING   = 3, 
    FSM_STATE_OPEN      = 4, 
    FSM_STATE_CLOSING   = 5, 
    FSM_STATE_CLOSED    = 6, 
    FSM_STATE_FAULT     = 7, 
} fsm_state_t;

typedef enum {
    EMERGENCY_LOCK_OFF = 0,
    EMERGENCY_LOCK_ON  = 1,
} emergency_lock_t;

/* ---------------------------------------------------------------------------
 * Fault Codes — stored in NVS 
 * --------------------------------------------------------------------------- */
typedef enum {
    FAULT_NONE              = 0x00,
    FAULT_MOTOR_STALL       = 0x01,
    FAULT_SPOF              = 0x02,
    FAULT_COMM_TIMEOUT      = 0x03,
    FAULT_OBSTRUCTION       = 0x04,  /* Stale obstruction after timeout.      */
    FAULT_NVS_BOOT          = 0x05,  /* Fault persisted from previous session.*/
} fault_code_t;

/* ---------------------------------------------------------------------------
 * Initialisation & Task Entry
 * --------------------------------------------------------------------------- */
esp_err_t fsm_init(void);


void fsm_control_task(void *pvParameters);

fsm_state_t fsm_get_state(void);
emergency_lock_t fsm_get_emergency_lock(void);
fault_code_t fsm_get_fault_code(void);

#endif 
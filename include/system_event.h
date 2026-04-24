#ifndef SYSTEM_EVENT_H
#define SYSTEM_EVENT_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Event Types
 * --------------------------------------------------------------------------- */
typedef enum {
    /* --- Command events (from Supervisor/user → HAL → Dispatcher → FSM) --- */
    EVT_CMD_OPEN            = 0x01,  
    EVT_CMD_CLOSE           = 0x02,  
    EVT_CMD_EMERGENCY_OPEN  = 0x03,  
    EVT_CMD_RESET           = 0x04,  

    /* --- Sensor events (HAL-debounced before dispatch) --- */
    EVT_SENSOR_FULLY_OPEN   = 0x10,  
    EVT_SENSOR_FULLY_CLOSED = 0x11,  
    EVT_OBSTRUCTION_DETECTED= 0x12,  
    EVT_OBSTRUCTION_CLEAR   = 0x13,  

    /* --- Internal / system events --- */
    EVT_COMM_TIMEOUT        = 0x20,  
    EVT_MOTOR_STALL         = 0x21,  
    EVT_SPOF_DETECTED       = 0x22,  
    EVT_HOMING_COMPLETE     = 0x23,  
    EVT_FAULT_PERSIST       = 0x24,  
    EVT_HEARTBEAT           = 0x25,  

    EVT_UNKNOWN             = 0xFF,  
} event_type_t;

/* ---------------------------------------------------------------------------
 * Event Source — for audit logging
 * --------------------------------------------------------------------------- */
typedef enum {
    SRC_UART_SUPERVISOR = 0, 
    SRC_INTERNAL_FSM    = 1,  
    SRC_INTERNAL_SAFETY = 2,  
    SRC_HAL_SENSOR      = 3,  
} event_source_t;

typedef struct {
    event_type_t   type;        
    event_source_t source;      
    uint32_t       timestamp_ms;
    uint32_t       payload;     
} system_event_t;

#endif 
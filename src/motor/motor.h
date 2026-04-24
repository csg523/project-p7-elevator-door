// =============================================================
// motor/motor.h — Simulated motor / actuator driver
//
// In hardware: replace these functions with GPIO or PWM calls.
// In simulation: they log the command and track state.
// Deliberately kept stateless (called by FSM; FSM owns state).
// =============================================================
#pragma once

typedef enum
{
    MOTOR_STOPPED = 0,
    MOTOR_OPENING,
    MOTOR_CLOSING
} MotorCommand;

void motor_init(void);
void motor_open(void);
void motor_close(void);
void motor_stop(void);
MotorCommand motor_current(void);

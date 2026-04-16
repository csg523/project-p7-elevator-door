// =============================================================
// motor/motor.cpp — Simulated motor driver
// Replace the Serial prints with real GPIO/PWM in hardware.
// =============================================================
#include "motor.h"
#include "system.h"

#include <Arduino.h>

static volatile MotorCommand s_current = MOTOR_STOPPED;

void motor_init(void)
{
    s_current = MOTOR_STOPPED;
    // Hardware: configure direction GPIO and PWM channel here
}

void motor_open(void)
{
    if (s_current != MOTOR_OPENING) {
        s_current = MOTOR_OPENING;
        log_post("MOTOR: → OPENING");
        // Hardware: set direction pin HIGH, enable PWM
    }
}

void motor_close(void)
{
    if (s_current != MOTOR_CLOSING) {
        s_current = MOTOR_CLOSING;
        log_post("MOTOR: → CLOSING");
        // Hardware: set direction pin LOW, enable PWM
    }
}

void motor_stop(void)
{
    if (s_current != MOTOR_STOPPED) {
        s_current = MOTOR_STOPPED;
        log_post("MOTOR: → STOP");
        // Hardware: disable PWM
    }
}

MotorCommand motor_current(void)
{
    return s_current;
}

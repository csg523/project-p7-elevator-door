// =============================================================
// motor/motor.cpp — Simulated motor driver
// Replace the Serial prints with real GPIO/PWM in hardware.
// =============================================================
#include "motor.h"

// Standard C library for printing
#include <stdio.h>

// Native ESP32 Hardware Libraries
#include "driver/gpio.h"
#include "driver/ledc.h"


// Hardware Pin Definitions
#define MOTOR_PIN_DIR1 GPIO_NUM_26 // Direction Pin 1
#define MOTOR_PIN_DIR2 GPIO_NUM_27 // Direction Pin 2
#define MOTOR_PIN_PWM GPIO_NUM_14  // PWM Speed Control Pin

// PWM Configuration Settings
#define MOTOR_PWM_FREQ_HZ 5000 // 5 kHz switching frequency
#define MOTOR_PWM_DUTY 4095    // ~50% speed (13-bit max is 8191)

static volatile MotorCommand s_current = MOTOR_STOPPED;

// Initialization
void motor_init(void)
{
    s_current = MOTOR_STOPPED;

    // 1. Configure Direction Pins
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << MOTOR_PIN_DIR1) | (1ULL << MOTOR_PIN_DIR2);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Ensure stopped state
    gpio_set_level(MOTOR_PIN_DIR1, 0);
    gpio_set_level(MOTOR_PIN_DIR2, 0);

    // 2. Configure the PWM Timer
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = MOTOR_PWM_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    // 3. Configure the PWM Channel
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.intr_type = LEDC_INTR_DISABLE;
    ledc_channel.gpio_num = MOTOR_PIN_PWM;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel_config(&ledc_channel);

    printf("MOTOR: Hardware Initialized (Pins 26, 27, 14)\n");
}

// Motor Commands
void motor_open(void)
{
    if (s_current != MOTOR_OPENING)
    {
        s_current = MOTOR_OPENING;
        printf("MOTOR: Actuator engaged → OPENING\n");

        // Hardware: DIR1=HIGH, DIR2=LOW
        gpio_set_level(MOTOR_PIN_DIR1, 1);
        gpio_set_level(MOTOR_PIN_DIR2, 0);

        // Output PWM wave
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOTOR_PWM_DUTY);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void motor_close(void)
{
    if (s_current != MOTOR_CLOSING)
    {
        s_current = MOTOR_CLOSING;
        printf("MOTOR: Actuator engaged → CLOSING\n");

        // Hardware: DIR1=LOW, DIR2=HIGH
        gpio_set_level(MOTOR_PIN_DIR1, 0);
        gpio_set_level(MOTOR_PIN_DIR2, 1);

        // Output PWM wave
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOTOR_PWM_DUTY);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void motor_stop(void)
{
    if (s_current != MOTOR_STOPPED)
    {
        s_current = MOTOR_STOPPED;
        printf("MOTOR: Actuator disengaged → STOP\n");

        // Hardware: Both DIR pins LOW, PWM to 0
        gpio_set_level(MOTOR_PIN_DIR1, 0);
        gpio_set_level(MOTOR_PIN_DIR2, 0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

MotorCommand motor_current(void)
{
    return s_current;
}

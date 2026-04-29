#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"

/* ---------------------------------------------------------------------------
 * HAL Initialisation
 * --------------------------------------------------------------------------- */

esp_err_t hal_init(QueueHandle_t dispatcher_queue);

/* ---------------------------------------------------------------------------
 * HAL Tasks (created by app_main)
 * --------------------------------------------------------------------------- */

void hal_uart_rx_task(void *pvParameters);
void hal_uart_tx_task(void *pvParameters);

/* ---------------------------------------------------------------------------
 * Actuator Commands (called by FSM — translated to GPIO/motor driver in HAL)
 * --------------------------------------------------------------------------- */

void hal_motor_open(void);
void hal_motor_close(void);
void hal_motor_stop(void);

/* ---------------------------------------------------------------------------
 * State Feedback TX
 * --------------------------------------------------------------------------- */

BaseType_t hal_tx_enqueue(const char *state_str);

/* ---------------------------------------------------------------------------
 * Acknowledgement helpers (FR-6)
 * --------------------------------------------------------------------------- */

void hal_send_ack(void);
void hal_send_nack(void);

#endif 
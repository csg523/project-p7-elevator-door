#ifndef EVENT_LOGGER_H
#define EVENT_LOGGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"
#include "door_fsm.h"

QueueHandle_t logger_init(void);

void logger_task(void *pvParameters);

void logger_log_event(QueueHandle_t logger_q, const system_event_t *evt);

void logger_log_transition(QueueHandle_t logger_q,
                           fsm_state_t old_state,
                           fsm_state_t new_state,
                           const system_event_t *trigger);

void logger_log_queue_overflow(QueueHandle_t logger_q, const char *queue_name);

#endif
#ifndef EVENT_DISPATCHER_H
#define EVENT_DISPATCHER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "system_event.h"
#include "esp_err.h"

QueueHandle_t dispatcher_create_queue(void);

void dispatcher_task(void *pvParameters);


BaseType_t dispatcher_post_event(QueueHandle_t q, const system_event_t *evt);

BaseType_t dispatcher_post_event_from_isr(QueueHandle_t q,
                                          const system_event_t *evt,
                                          BaseType_t *pxHigherPriorityTaskWoken);

#endif 
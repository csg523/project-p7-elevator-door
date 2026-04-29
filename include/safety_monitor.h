#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"

esp_err_t safety_monitor_init(void);

void safety_monitor_task(void *pvParameters);

void safety_monitor_reset_comm_watchdog(void);

void safety_monitor_update_sensors(uint8_t fully_open, uint8_t fully_closed);

#endif 
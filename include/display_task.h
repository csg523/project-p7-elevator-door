#ifndef DISPLAY_TASK_H
#define DISPLAY_TASK_H

#include "esp_err.h"

esp_err_t display_init(void);

void display_task(void *pvParameters);

#endif /* DISPLAY_TASK_H */
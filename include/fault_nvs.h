#ifndef FAULT_NVS_H
#define FAULT_NVS_H

#include <stdint.h>
#include "esp_err.h"
#include "door_fsm.h"  /* for fault_code_t */

esp_err_t fault_nvs_open(void);

esp_err_t fault_nvs_write(fault_code_t code);

esp_err_t fault_nvs_read(fault_code_t *code);

esp_err_t fault_nvs_clear(void);

void fault_nvs_close(void);

#endif /* FAULT_NVS_H */
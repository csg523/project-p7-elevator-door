#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
/*Watchdog Timer (TWDT) API for safety-critical task monitoring. */
#include "esp_task_wdt.h"

#include "config.h"
#include "system_event.h"
#include "hal.h"
#include "door_fsm.h"
#include "safety_monitor.h"
#include "event_dispatcher.h"
#include "event_logger.h"
#include "fault_nvs.h"
#include "display_task.h"

static const char *TAG = "MAIN";

/* ---------------------------------------------------------------------------
 * Task Handle Storage
 * --------------------------------------------------------------------------- */
TaskHandle_t g_task_safety     = NULL;
TaskHandle_t g_task_dispatcher = NULL;
TaskHandle_t g_task_control    = NULL;
TaskHandle_t g_task_hal_rx     = NULL;
TaskHandle_t g_task_hal_tx     = NULL;
TaskHandle_t g_task_logger     = NULL;

void vApplicationStackOverflowHook(TaskHandle_t Task, char *TaskName)
{
    ESP_LOGE(TAG, "!!! STACK OVERFLOW in task: %s !!!", TaskName);
    (void)Task;
    
    esp_restart();
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Elevator Door Safety Controller Booting ===");

    /* ── 1. NVS Flash Init ──────────────────────────────────────────────── */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition damaged — erasing");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS flash initialised");

    /* ── 2. Open NVS namespace ──────────────────────────────────────────── */
    ESP_ERROR_CHECK(fault_nvs_open());

    /* ── 3. Check for persisted fault (SR-4, NFR-3) ─────────────────────── */
    fault_code_t boot_fault = FAULT_NONE;
    ESP_ERROR_CHECK(fault_nvs_read(&boot_fault));
    if (boot_fault != FAULT_NONE) {
        ESP_LOGW(TAG, "Persisted fault detected on boot: code=%d", (int)boot_fault);
    }

    
    QueueHandle_t logger_queue = logger_init();
    if (logger_queue == NULL) {
        ESP_LOGE(TAG, "Logger init failed — cannot continue");
        return;
    }
    extern QueueHandle_t g_logger_queue;
    g_logger_queue = logger_queue;

    
    ESP_ERROR_CHECK(fsm_init());
    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(safety_monitor_init());

    
    QueueHandle_t dispatcher_queue = dispatcher_create_queue();
    if (dispatcher_queue == NULL) {
        ESP_LOGE(TAG, "Dispatcher queue creation failed — cannot continue");
        return;
    }

    ESP_ERROR_CHECK(hal_init(dispatcher_queue));

    const esp_task_wdt_config_t twdt_cfg = {
        .timeout_ms    = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000u,
        .idle_core_mask= 0u, 
        .trigger_panic = true,
    };
    ret = esp_task_wdt_init(&twdt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means TWDT was already initialised. */
        ESP_ERROR_CHECK(ret);
    }
    ESP_LOGI(TAG, "TWDT configured (%d s timeout, panic on expiry)",
             CONFIG_ESP_TASK_WDT_TIMEOUT_S);

    /*
     * Core 1 — Safety-Critical (Safety > Dispatcher = Control)
     */
    xTaskCreatePinnedToCore(
        safety_monitor_task,
        "SafetyTask",
        STACK_SAFETY_TASK,
        (void *)dispatcher_queue,   
        PRIORITY_SAFETY_TASK,
        &g_task_safety,
        CORE_SAFETY
    );

    xTaskCreatePinnedToCore(
        dispatcher_task,
        "DispatcherTask",
        STACK_DISPATCHER_TASK,
        (void *)dispatcher_queue,
        PRIORITY_DISPATCHER_TASK,
        &g_task_dispatcher,
        CORE_SAFETY
    );

    xTaskCreatePinnedToCore(
        fsm_control_task,
        "ControlTask",
        STACK_CONTROL_TASK,
        (void *)dispatcher_queue,  
        PRIORITY_CONTROL_TASK,
        &g_task_control,
        CORE_SAFETY
    );

    /*
     * Core 0 — Communications & Peripherals
     */
    xTaskCreatePinnedToCore(
        hal_uart_rx_task,
        "HAL_RX",
        STACK_HAL_INPUT_TASK,
        NULL,                      
        PRIORITY_HAL_INPUT_TASK,
        &g_task_hal_rx,
        CORE_COMMS
    );

    xTaskCreatePinnedToCore(
        hal_uart_tx_task,
        "HAL_TX",
        STACK_UART_TX_TASK,
        NULL,
        PRIORITY_UART_TX_TASK,
        &g_task_hal_tx,
        CORE_COMMS
    );

    xTaskCreatePinnedToCore(
        logger_task,
        "LoggerTask",
        STACK_LOGGER_TASK,
        (void *)logger_queue,
        PRIORITY_LOGGER_TASK,
        &g_task_logger,
        CORE_COMMS
    );

    xTaskCreatePinnedToCore(
        display_task, 
        "DisplayTask",
        STACK_DISPLAY_TASK,
        NULL,
        PRIORITY_DISPLAY_TASK,
        NULL,          // or &g_task_display if you want stack monitoring
        CORE_COMMS     // Core 0
);

    ESP_LOGI(TAG, "All tasks spawned");

    /* ── 11. Inject boot-fault event if NVS fault was found (SR-4) ──────── */
    if (boot_fault != FAULT_NONE) {
        
        vTaskDelay(pdMS_TO_TICKS(50u));

        system_event_t fault_evt = {
            .type         = EVT_FAULT_PERSIST,
            .source       = SRC_INTERNAL_SAFETY,
            .timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000LL),
            .payload      = (uint32_t)boot_fault,
        };

        extern QueueHandle_t g_fsm_queue;
        if (xQueueSendToBack(g_fsm_queue, &fault_evt, pdMS_TO_TICKS(100u)) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to inject EVT_FAULT_PERSIST — FSM queue full");
        } else {
            ESP_LOGW(TAG, "EVT_FAULT_PERSIST injected — FSM will boot into FAULT");
        }
    }

    ESP_LOGI(TAG, "app_main complete — scheduler running");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
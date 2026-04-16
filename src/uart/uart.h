// =============================================================
// uart/uart.h — UART receive task and initialisation
// =============================================================
#pragma once

// Initialise Serial (call from setup() before task creation)
void uart_init(void);

// FreeRTOS task entry point for UART receive
void uart_rx_task(void *param);

// FreeRTOS task entry point for UART transmit
void uart_tx_task(void *param);

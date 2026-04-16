// =============================================================
// safety/safety.h — Independent safety monitoring task
//
// Runs at the highest priority. Independently monitors:
//   • Obstruction flag (SR-1, SR-2)
//   • Communication timeout (SR-5)
// Posts safety events to g_eventQueue without going through
// the UART or any lower-priority task.
// =============================================================
#pragma once

void safety_init(void);
void safety_task(void *param);

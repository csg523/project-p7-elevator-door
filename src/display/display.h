// =============================================================
// display/display.h — TFT status display (TTGO T-Display)
// Guarded by ENABLE_DISPLAY so it compiles out on other boards.
// =============================================================
#pragma once

#include "fsm/fsm.h"

void display_init(void);
void display_task(void *param);

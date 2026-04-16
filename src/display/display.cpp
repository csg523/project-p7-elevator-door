// =============================================================
// display/display.cpp — TFT status display for TTGO T-Display
// Refreshed every 200 ms; shows state, uptime, last motor cmd.
// =============================================================
#include "display.h"
#include "config.h"
#include "system.h"
#include "fsm/fsm.h"
#include "motor/motor.h"

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef ENABLE_DISPLAY
#include <TFT_eSPI.h>

static TFT_eSPI tft = TFT_eSPI();

// Colour map per state
static uint16_t stateColour(DoorState s)
{
    switch (s)
    {
    case STATE_IDLE:
        return TFT_DARKGREY;
    case STATE_OPENING:
        return TFT_GREEN;
    case STATE_OPEN:
        return TFT_CYAN;
    case STATE_CLOSING:
        return TFT_YELLOW;
    case STATE_CLOSED:
        return TFT_BLUE;
    case STATE_STOPPED:
        return TFT_ORANGE;
    case STATE_EMERGENCY:
        return TFT_MAGENTA;
    case STATE_FAULT:
        return TFT_RED;
    default:
        return TFT_WHITE;
    }
}

void display_init(void)
{
    tft.init();
    tft.setRotation(1); // Landscape 240×135
    tft.fillScreen(TFT_BLACK);

    // Backlight on
    // pinMode(TFT_BL, OUTPUT);
    // digitalWrite(TFT_BL, HIGH);

    tft.setTextFont(4);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("Door Controller", 10, 10);
    tft.drawString("Initializing...", 10, 50);
}

void display_task(void *param)
{
    (void)param;
    DoorState prevState = (DoorState)0xFF;

    for (;;)
    {
        DoorState cur = fsm_get_state();
        uint32_t upSec = millis() / 1000;

        if (cur != prevState)
        {
            tft.fillScreen(TFT_BLACK);
            prevState = cur;
        }

        uint16_t col = stateColour(cur);

        // Title bar
        tft.fillRect(0, 0, 240, 30, col);
        tft.setTextColor(TFT_BLACK, col);
        tft.setTextFont(4);
        tft.drawString("DOOR CONTROLLER", 10, 5);

        // State
        tft.setTextColor(col, TFT_BLACK);
        tft.setTextFont(6);
        tft.drawString(fsm_state_name(cur), 10, 40);

        // Uptime
        tft.setTextFont(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        char uptimeBuf[32];
        snprintf(uptimeBuf, sizeof(uptimeBuf), "Up: %lus", (unsigned long)upSec);
        tft.drawString(uptimeBuf, 10, 110);

        // Motor status
        const char *mStr = "STOP";
        switch (motor_current())
        {
        case MOTOR_OPENING:
            mStr = "OPEN";
            break;
        case MOTOR_CLOSING:
            mStr = "CLOSE";
            break;
        default:
            break;
        }
        char motBuf[32];
        snprintf(motBuf, sizeof(motBuf), "Motor: %s", mStr);
        tft.drawString(motBuf, 120, 110);

        // Obstruction indicator
        if (g_obstructionActive)
        {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawString("!! OBS !!", 80, 90);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

#else // ENABLE_DISPLAY not defined

void display_init(void) {}

void display_task(void *param)
{
    (void)param;
    // No display — task just idles
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

#endif // ENABLE_DISPLAY

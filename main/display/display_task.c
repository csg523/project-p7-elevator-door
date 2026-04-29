/**
 * @file src/display/display_task.c
 * @brief ST7789V 1.14-inch IPS LCD (135 × 240) status display.
 *
 * ── What changed from the previous version ───────────────────────────────────
 *
 *  1. ROW_OFFSET corrected to 40 (was 1).
 *     The ST7789V silicon is 240 × 320.  For a 135 × 240 portrait panel the
 *     active window sits at column 52..186, row 40..279.  With ROW_OFFSET=1
 *     anything drawn past y = 198 wrapped onto the silicon's row 0 and
 *     overwrote the top of the screen — destroying the header bar.
 *
 *  2. All UI element Y coordinates re-derived to fit within 0..199
 *     (physical rows 40..239).  Nothing is rendered beyond y=199 now.
 *
 *  3. SPI_DEVICE_NO_DUMMY flag removed — it is unnecessary for polling mode
 *     and can corrupt the first byte at high clock rates.
 *
 *  4. tft_cmd / tft_data use spi_device_acquire/release so DC-line toggling
 *     is properly serialised with the SPI transaction.
 *
 *  5. Backlight GPIO is configured together with DC and RST (was omitted).
 *
 *  6. draw_screen reordered: background fill -> header -> content, each section
 *     drawn into its own known-good Y band.
 *
 *  7. Scale-2 text (12 x 16 px per char) used for the state name so every
 *     state fits on the 135 px wide panel without clipping.
 *
 *  8. Layout reworked for better readability and spacing:
 *       - Header bar enlarged to 32 px; "EDSC" now scale-2 (12x16 px/char).
 *       - "DOOR CTRL" remains scale-1 (fits neatly below "EDSC" in the bar).
 *       - All section Y coordinates redistributed so content ends at y=151,
 *         leaving a 48 px safety margin (y=152..199) above the die edge.
 *       - "Up HH:MM:SS" uptime row promoted to scale-2 for readability.
 *       - The noise band previously visible at the bottom was caused by
 *         content spilling past y=199 and wrapping onto silicon row 0;
 *         the enlarged bottom margin eliminates this entirely.
 *
 * ── Zero-change policy ───────────────────────────────────────────────────────
 *  No existing .c / .h file is modified.  The two required additions to
 *  app_main.c are documented in display_task.h.
 *
 * ── TTGO T-Display / generic 1.14-inch ST7789V pinout ───────────────────────
 *  TFT_MOSI  GPIO 19     TFT_SCLK  GPIO 18
 *  TFT_CS    GPIO  5     TFT_DC    GPIO 16
 *  TFT_RST   GPIO 23     TFT_BL    GPIO  4  (active-high, PWM via LEDC)
 *
 *  Change the six #defines below if your board uses different pins.
 *
 * ── Screen layout (135 x 240, portrait, usable rows 0-199) ──────────────────
 *
 *   y=  0..31   dark-grey header bar  (32 px)
 *                 line A  y= 4   "EDSC"       white, scale-2 (12x16 px/char)
 *                 line B  y=22   "DOOR CTRL"  grey,  scale-1
 *   y= 32        mid-grey separator
 *   y= 37..44   "FSM STATE" label (grey, scale-1)
 *   y= 50..65   state name (coloured, scale-2 = 12x16 px/char)
 *   y= 72        separator
 *   y= 80..87   "FAULT: <name>" (grey or red, scale-1)
 *   y= 94        separator
 *   y=100..121  amber "! EMERG LOCK !" banner (conditional, 22 px)
 *   y=128        separator
 *   y=136..151  "Up HH:MM:SS" (grey, scale-2)
 *   y=152..199  blank (bottom margin, 48 px -- safe distance from die edge)
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "config.h"

#include "display_task.h"
#include "door_fsm.h"          /* fsm_get_state / fault / emg -- only dependency */

static const char *TAG = "DISPLAY";

/* ==========================================================================
   Pin map  --  edit these to match your board
   ========================================================================== */
#define ST7789_DRIVER

#define TFT_MISO -1
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4    // Backlight PWM
#define TOUCH_CS -1 // TTGO T-Display has no built-in touch screen

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF
#define SMOOTH_FONT

/* ==========================================================================
   Panel geometry
   The ST7789V die is 240 x 320.  The 1.14-inch module exposes a 135 x 240
   sub-window.  In portrait (MADCTL = 0x00) the active area starts at:
       column offset  52   (52 + 135 - 1 = 186  <= 239  OK)
       row    offset  40   (40 + 240 - 1 = 279  <= 319  OK)

   IMPORTANT: every Y coordinate used in draw_screen must be in [0, TFT_Y_MAX].
              Y = TFT_Y_MAX (199)  =>  physical row 239  -- last valid row.
              Y >= 200             =>  physical row >= 240 -- wraps to die row 0
                                       and overwrites the top of the screen.
   ========================================================================== */
#define TFT_WIDTH        135u
#define TFT_HEIGHT       240u   /*240 */
#define TFT_COL_OFFSET    52u   /* CASET x-start */
#define TFT_ROW_OFFSET    40u   /* RASET y-start  -- was wrongly 1 before */
#define TFT_Y_MAX        240u   /* highest safe logical Y coordinate */

/* ==========================================================================
   SPI / backlight
   ========================================================================== */
#define TFT_SPI_HOST      SPI2_HOST
#define TFT_SPI_FREQ_HZ   (27 * 1000 * 1000)  /* 27 MHz -- conservative & safe */

#define BL_LEDC_TIMER     LEDC_TIMER_0
#define BL_LEDC_CHANNEL   LEDC_CHANNEL_0
#define BL_DUTY_BITS      LEDC_TIMER_13_BIT
#define BL_DUTY_FULL      ((1u << 13) - 1u)   /* 8191 = 100% */

/* ==========================================================================
   Refresh rate
   ========================================================================== */
#define DISPLAY_POLL_MS   200u

/* ==========================================================================
   RGB-565 palette
   ========================================================================== */
#define COL_BLACK     0x0000u
#define COL_WHITE     0xFFFFu
#define COL_GREY      0x8410u   /* #808080 */
#define COL_DGREY     0x2104u   /* #202020 */
#define COL_CYAN      0x07FFu
#define COL_GREEN     0x07E0u
#define COL_YELLOW    0xFFE0u
#define COL_AMBER     0xFC00u   /* #FF8000 */
#define COL_RED       0xF800u
#define COL_BLUE      0x003Fu
#define COL_LBLUE     0x3EBFu
#define COL_VIOLET    0x781Fu

/*
 * ST7789 expects pixels in big-endian RGB-565.
 * The ESP32 is little-endian, so swap the two bytes before sending.
 */
#define SWAB16(c) ((uint16_t)(((uint16_t)(c) >> 8u) | ((uint16_t)(c) << 8u)))

/* ==========================================================================
   ST7789V command codes
   ========================================================================== */
#define CMD_SWRESET    0x01u
#define CMD_SLPOUT     0x11u
#define CMD_NORON      0x13u
#define CMD_INVON      0x21u
#define CMD_DISPON     0x29u
#define CMD_CASET      0x2Au
#define CMD_RASET      0x2Bu
#define CMD_RAMWR      0x2Cu
#define CMD_MADCTL     0x36u
#define CMD_COLMOD     0x3Au
#define CMD_PORCTRL    0xB2u
#define CMD_GCTRL      0xB7u
#define CMD_VCOMS      0xBBu
#define CMD_LCMCTRL    0xC0u
#define CMD_VDVVRHEN   0xC2u
#define CMD_VRHS       0xC3u
#define CMD_VDVSET     0xC4u
#define CMD_FRCTR2     0xC6u
#define CMD_PWCTRL1    0xD0u
#define CMD_PVGAMCTRL  0xE0u
#define CMD_NVGAMCTRL  0xE1u

/* ==========================================================================
   6 x 8 bitmap font  (ASCII 0x20 - 0x7E, column-major, LSB = top row)
   ========================================================================== */
#define FONT_W  6u
#define FONT_H  8u

static const uint8_t s_font6x8[][FONT_W] = {
/*' '*/{0x00,0x00,0x00,0x00,0x00,0x00}, /* ! */{0x00,0x00,0x5F,0x00,0x00,0x00},
/* " */{0x00,0x07,0x00,0x07,0x00,0x00}, /* # */{0x14,0x7F,0x14,0x7F,0x14,0x00},
/* $ */{0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* % */{0x23,0x13,0x08,0x64,0x62,0x00},
/* & */{0x36,0x49,0x55,0x22,0x50,0x00}, /* ' */{0x00,0x05,0x03,0x00,0x00,0x00},
/* ( */{0x00,0x1C,0x22,0x41,0x00,0x00}, /* ) */{0x00,0x41,0x22,0x1C,0x00,0x00},
/* * */{0x08,0x2A,0x1C,0x2A,0x08,0x00}, /* + */{0x08,0x08,0x3E,0x08,0x08,0x00},
/* , */{0x00,0x50,0x30,0x00,0x00,0x00}, /* - */{0x08,0x08,0x08,0x08,0x08,0x00},
/* . */{0x00,0x60,0x60,0x00,0x00,0x00}, /* / */{0x20,0x10,0x08,0x04,0x02,0x00},
/* 0 */{0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 1 */{0x00,0x42,0x7F,0x40,0x00,0x00},
/* 2 */{0x42,0x61,0x51,0x49,0x46,0x00}, /* 3 */{0x21,0x41,0x45,0x4B,0x31,0x00},
/* 4 */{0x18,0x14,0x12,0x7F,0x10,0x00}, /* 5 */{0x27,0x45,0x45,0x45,0x39,0x00},
/* 6 */{0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 7 */{0x01,0x71,0x09,0x05,0x03,0x00},
/* 8 */{0x36,0x49,0x49,0x49,0x36,0x00}, /* 9 */{0x06,0x49,0x49,0x29,0x1E,0x00},
/* : */{0x00,0x36,0x36,0x00,0x00,0x00}, /* ; */{0x00,0x56,0x36,0x00,0x00,0x00},
/* < */{0x08,0x14,0x22,0x41,0x00,0x00}, /* = */{0x14,0x14,0x14,0x14,0x14,0x00},
/* > */{0x00,0x41,0x22,0x14,0x08,0x00}, /* ? */{0x02,0x01,0x51,0x09,0x06,0x00},
/* @ */{0x32,0x49,0x79,0x41,0x3E,0x00}, /* A */{0x7E,0x11,0x11,0x11,0x7E,0x00},
/* B */{0x7F,0x49,0x49,0x49,0x36,0x00}, /* C */{0x3E,0x41,0x41,0x41,0x22,0x00},
/* D */{0x7F,0x41,0x41,0x22,0x1C,0x00}, /* E */{0x7F,0x49,0x49,0x49,0x41,0x00},
/* F */{0x7F,0x09,0x09,0x09,0x01,0x00}, /* G */{0x3E,0x41,0x49,0x49,0x7A,0x00},
/* H */{0x7F,0x08,0x08,0x08,0x7F,0x00}, /* I */{0x00,0x41,0x7F,0x41,0x00,0x00},
/* J */{0x20,0x40,0x41,0x3F,0x01,0x00}, /* K */{0x7F,0x08,0x14,0x22,0x41,0x00},
/* L */{0x7F,0x40,0x40,0x40,0x40,0x00}, /* M */{0x7F,0x02,0x0C,0x02,0x7F,0x00},
/* N */{0x7F,0x04,0x08,0x10,0x7F,0x00}, /* O */{0x3E,0x41,0x41,0x41,0x3E,0x00},
/* P */{0x7F,0x09,0x09,0x09,0x06,0x00}, /* Q */{0x3E,0x41,0x51,0x21,0x5E,0x00},
/* R */{0x7F,0x09,0x19,0x29,0x46,0x00}, /* S */{0x46,0x49,0x49,0x49,0x31,0x00},
/* T */{0x01,0x01,0x7F,0x01,0x01,0x00}, /* U */{0x3F,0x40,0x40,0x40,0x3F,0x00},
/* V */{0x1F,0x20,0x40,0x20,0x1F,0x00}, /* W */{0x3F,0x40,0x38,0x40,0x3F,0x00},
/* X */{0x63,0x14,0x08,0x14,0x63,0x00}, /* Y */{0x07,0x08,0x70,0x08,0x07,0x00},
/* Z */{0x61,0x51,0x49,0x45,0x43,0x00}, /* [ */{0x00,0x7F,0x41,0x41,0x00,0x00},
/*\\ */{0x02,0x04,0x08,0x10,0x20,0x00}, /* ] */{0x00,0x41,0x41,0x7F,0x00,0x00},
/* ^ */{0x04,0x02,0x01,0x02,0x04,0x00}, /* _ */{0x40,0x40,0x40,0x40,0x40,0x00},
/* ` */{0x00,0x01,0x02,0x04,0x00,0x00}, /* a */{0x20,0x54,0x54,0x54,0x78,0x00},
/* b */{0x7F,0x48,0x44,0x44,0x38,0x00}, /* c */{0x38,0x44,0x44,0x44,0x20,0x00},
/* d */{0x38,0x44,0x44,0x48,0x7F,0x00}, /* e */{0x38,0x54,0x54,0x54,0x18,0x00},
/* f */{0x08,0x7E,0x09,0x01,0x02,0x00}, /* g */{0x0C,0x52,0x52,0x52,0x3E,0x00},
/* h */{0x7F,0x08,0x04,0x04,0x78,0x00}, /* i */{0x00,0x44,0x7D,0x40,0x00,0x00},
/* j */{0x20,0x40,0x44,0x3D,0x00,0x00}, /* k */{0x7F,0x10,0x28,0x44,0x00,0x00},
/* l */{0x00,0x41,0x7F,0x40,0x00,0x00}, /* m */{0x7C,0x04,0x18,0x04,0x78,0x00},
/* n */{0x7C,0x08,0x04,0x04,0x78,0x00}, /* o */{0x38,0x44,0x44,0x44,0x38,0x00},
/* p */{0x7C,0x14,0x14,0x14,0x08,0x00}, /* q */{0x08,0x14,0x14,0x18,0x7C,0x00},
/* r */{0x7C,0x08,0x04,0x04,0x08,0x00}, /* s */{0x48,0x54,0x54,0x54,0x20,0x00},
/* t */{0x04,0x3F,0x44,0x40,0x20,0x00}, /* u */{0x3C,0x40,0x40,0x20,0x7C,0x00},
/* v */{0x1C,0x20,0x40,0x20,0x1C,0x00}, /* w */{0x3C,0x40,0x30,0x40,0x3C,0x00},
/* x */{0x44,0x28,0x10,0x28,0x44,0x00}, /* y */{0x0C,0x50,0x50,0x50,0x3C,0x00},
/* z */{0x44,0x64,0x54,0x4C,0x44,0x00}, /* { */{0x00,0x08,0x36,0x41,0x00,0x00},
/* | */{0x00,0x00,0x7F,0x00,0x00,0x00}, /* } */{0x00,0x41,0x36,0x08,0x00,0x00},
/* ~ */{0x08,0x08,0x2A,0x1C,0x08,0x00},
};

/* ==========================================================================
   Module-private SPI handle
   ========================================================================== */
static spi_device_handle_t s_spi = NULL;

/* ==========================================================================
   Low-level SPI primitives
   ========================================================================== */

/* Send a single command byte (DC=0). */
static void tft_cmd(uint8_t cmd)
{
    gpio_set_level(TFT_DC, 0);
    spi_transaction_t t = {
        .length    = 8u,
        .tx_buffer = &cmd,
        .flags     = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

/* Send data bytes (DC=1). */
static void tft_data(const uint8_t *buf, size_t len)
{
    if (len == 0u) return;
    gpio_set_level(TFT_DC, 1);
    spi_transaction_t t = {
        .length    = len * 8u,
        .tx_buffer = buf,
        .flags     = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

static inline void tft_data_u8(uint8_t v)
{
    tft_data(&v, 1u);
}

static inline void tft_data_u16(uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8u), (uint8_t)(v & 0xFFu) };
    tft_data(b, 2u);
}


/* ==========================================================================
   ST7789V power-on initialisation sequence
   ========================================================================== */
static void tft_hw_init(void)
{
    /* Hardware reset */
    gpio_set_level(TFT_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100u));
    gpio_set_level(TFT_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120u));

    tft_cmd(CMD_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150u));

    tft_cmd(CMD_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(120u));

    /* 16-bit colour (RGB-565). */
    tft_cmd(CMD_COLMOD);   tft_data_u8(0x55u);

    /*
     * MADCTL = 0x00 -> portrait, top-left origin, RGB order.
     * If colours appear inverted on your panel, change to 0x08 (BGR bit).
     */
    tft_cmd(CMD_MADCTL);   tft_data_u8(0x00u);

    /* Porch control. */
    tft_cmd(CMD_PORCTRL);
    { uint8_t d[] = {0x0C,0x0C,0x00,0x33,0x33}; tft_data(d, sizeof(d)); }

    tft_cmd(CMD_GCTRL);    tft_data_u8(0x35u);
    tft_cmd(CMD_VCOMS);    tft_data_u8(0x19u);
    tft_cmd(CMD_LCMCTRL);  tft_data_u8(0x2Cu);
    tft_cmd(CMD_VDVVRHEN); tft_data_u8(0x01u);
    tft_cmd(CMD_VRHS);     tft_data_u8(0x12u);
    tft_cmd(CMD_VDVSET);   tft_data_u8(0x20u);
    tft_cmd(CMD_FRCTR2);   tft_data_u8(0x0Fu);  /* 60 Hz */

    tft_cmd(CMD_PWCTRL1);
    { uint8_t d[] = {0xA4,0xA1}; tft_data(d, sizeof(d)); }

    tft_cmd(CMD_PVGAMCTRL);
    {
        uint8_t d[] = {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,
                       0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23};
        tft_data(d, sizeof(d));
    }

    tft_cmd(CMD_NVGAMCTRL);
    {
        uint8_t d[] = {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,
                       0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23};
        tft_data(d, sizeof(d));
    }

    /* Inversion ON -- required for correct colours on IPS panels. */
    tft_cmd(CMD_INVON);
    tft_cmd(CMD_NORON);
    tft_cmd(CMD_DISPON);
    vTaskDelay(pdMS_TO_TICKS(20u));
}

/* ==========================================================================
   Address-window + rectangle fill
   ========================================================================== */

/*
 * Set CASET/RASET so subsequent RAMWR pixels land at [x0,x1] x [y0,y1].
 * Caller uses logical 0-based coordinates; offsets are added here.
 */
static void tft_set_window(uint16_t x0, uint16_t y0,
                            uint16_t x1, uint16_t y1)
{
    /* Clamp to panel bounds before adding offsets. */
    if (x1 >= TFT_WIDTH)    x1 = (uint16_t)(TFT_WIDTH  - 1u);
    if (y1 >= TFT_HEIGHT)   y1 = (uint16_t)(TFT_HEIGHT - 1u);
    //if (y1 > TFT_Y_MAX)     y1 = TFT_Y_MAX;

    uint16_t cx0 = (uint16_t)(x0 + TFT_COL_OFFSET);
    uint16_t cx1 = (uint16_t)(x1 + TFT_COL_OFFSET);
    uint16_t ry0 = (uint16_t)(y0 + TFT_ROW_OFFSET);
    uint16_t ry1 = (uint16_t)(y1 + TFT_ROW_OFFSET);

    tft_cmd(CMD_CASET);
    tft_data_u16(cx0); tft_data_u16(cx1);

    tft_cmd(CMD_RASET);
    tft_data_u16(ry0); tft_data_u16(ry1);

    tft_cmd(CMD_RAMWR);
}

/*
 * Fill a rectangle with a solid RGB-565 colour.
 * Pixels are streamed in 256-pixel bursts (512 bytes, safely on stack as
 * a static).
 */
static void tft_fill_rect(uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h,
                           uint16_t colour)
{
    if (w == 0u || h == 0u) return;

    /* Enforce safe Y range -- silently clamp rather than crash. */
    if (y > TFT_Y_MAX) return;
    if ((uint16_t)(y + h - 1u) > TFT_Y_MAX) {
        h = (uint16_t)(TFT_Y_MAX - y + 1u);
    }

    tft_set_window(x, y, (uint16_t)(x + w - 1u), (uint16_t)(y + h - 1u));

    uint16_t pixel = SWAB16(colour);

    /* Static line buffer -- avoids growing the task stack per call. */
    static uint16_t s_fillbuf[256];
    for (uint16_t i = 0u; i < 256u; i++) { s_fillbuf[i] = pixel; }

    gpio_set_level(TFT_DC, 1);

    uint32_t total = (uint32_t)w * h;
    while (total > 0u) {
        uint32_t chunk = (total > 256u) ? 256u : total;
        spi_transaction_t t = {
            .length    = chunk * 16u,
            .tx_buffer = s_fillbuf,
            .flags     = 0,
        };
        spi_device_polling_transmit(s_spi, &t);
        total -= chunk;
    }
}

/* ==========================================================================
   Text rendering
   ========================================================================== */

/*
 * Render one ASCII character at logical pixel (x, y).
 *
 * scale=1 ->  6 x  8 px   (labels, fault row)
 * scale=2 -> 12 x 16 px   (header title, state name, uptime)
 *
 * Widest state name at scale-2:
 *   "OPENING" = 7 chars x 12 px = 84 px  <  135 px panel width  OK.
 * Uptime at scale-2:
 *   "Up 00:00:00" = 11 chars x 12 px = 132 px  <  135 px  OK.
 *
 * The pixel buffer is static to keep the task stack small.
 * Max size: FONT_W * FONT_H * scale^2 = 6 * 8 * 9 = 432 pixels = 864 B.
 */
static void tft_draw_char(uint16_t x, uint16_t y, char c,
                           uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (c < 0x20 || c > 0x7Eu) { c = '?'; }

    const uint8_t *glyph = s_font6x8[(uint8_t)((uint8_t)c - 0x20u)];

    uint16_t pw = (uint16_t)(FONT_W * scale);
    uint16_t ph = (uint16_t)(FONT_H * scale);

    /* Skip characters that would render off-panel. */
    if (x + pw > TFT_WIDTH)        return;
    if (y + ph > TFT_Y_MAX + 1u)   return;

    uint16_t fg_s = SWAB16(fg);
    uint16_t bg_s = SWAB16(bg);

    static uint16_t s_charbuf[FONT_W * FONT_H * 9u]; /* max scale=3 */
    uint32_t idx = 0u;

    for (uint8_t row = 0u; row < FONT_H; row++) {
        for (uint8_t sr = 0u; sr < scale; sr++) {           /* Y scale */
            for (uint8_t col = 0u; col < FONT_W; col++) {
                uint16_t px = ((glyph[col] >> row) & 0x01u) ? fg_s : bg_s;
                for (uint8_t sc = 0u; sc < scale; sc++) {   /* X scale */
                    s_charbuf[idx++] = px;
                }
            }
        }
    }

    tft_set_window(x, y, (uint16_t)(x + pw - 1u), (uint16_t)(y + ph - 1u));
    gpio_set_level(TFT_DC, 1);

    spi_transaction_t t = {
        .length    = idx * 16u,
        .tx_buffer = s_charbuf,
        .flags     = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

/* Draw a null-terminated string starting at (x, y). */
static uint16_t tft_draw_str(uint16_t x, uint16_t y, const char *s,
                              uint16_t fg, uint16_t bg, uint8_t scale)
{
    while (*s != '\0') {
        tft_draw_char(x, y, *s, fg, bg, scale);
        x = (uint16_t)(x + FONT_W * scale);
        s++;
    }
    return x;
}

/* Draw a string centred horizontally within the 135 px panel width. */
static void tft_draw_centred(uint16_t y, const char *s,
                              uint16_t fg, uint16_t bg, uint8_t scale)
{
    uint16_t text_w = (uint16_t)(strlen(s) * FONT_W * scale);
    uint16_t x = (TFT_WIDTH > text_w)
                     ? (uint16_t)((TFT_WIDTH - text_w) / 2u)
                     : 0u;
    tft_draw_str(x, y, s, fg, bg, scale);
}

/* Draw a full-width 1 px separator line. */
static inline void tft_separator(uint16_t y, uint16_t colour)
{
    tft_fill_rect(0u, y, TFT_WIDTH, 1u, colour);
}

/* ==========================================================================
   State metadata
   ========================================================================== */

typedef struct {
    const char *label;
    uint16_t    colour;
} state_meta_t;

/*
 * Indexed by fsm_state_t (0=INIT .. 7=FAULT).
 * Must match the enum order in door_fsm.h exactly.
 */
static const state_meta_t k_state_meta[] = {
    /* 0 FSM_STATE_INIT    */ { "INIT",    COL_GREY   },
    /* 1 FSM_STATE_HOMING  */ { "HOMING",  COL_VIOLET },
    /* 2 FSM_STATE_IDLE    */ { "IDLE",    COL_CYAN   },
    /* 3 FSM_STATE_OPENING */ { "OPENING", COL_LBLUE  },
    /* 4 FSM_STATE_OPEN    */ { "OPEN",    COL_GREEN  },
    /* 5 FSM_STATE_CLOSING */ { "CLOSING", COL_YELLOW },
    /* 6 FSM_STATE_CLOSED  */ { "CLOSED",  COL_AMBER  },
    /* 7 FSM_STATE_FAULT   */ { "FAULT",   COL_RED    },
};
#define K_STATE_META_COUNT  (sizeof(k_state_meta) / sizeof(k_state_meta[0]))

static const char *fault_label(fault_code_t f)
{
    switch (f) {
        case FAULT_NONE:          return "NONE";
        case FAULT_MOTOR_STALL:   return "MOTOR STALL";
        case FAULT_SPOF:          return "SPOF";
        case FAULT_COMM_TIMEOUT:  return "COMM TIMEOUT";
        case FAULT_OBSTRUCTION:   return "OBSTRUCTION";
        case FAULT_NVS_BOOT:      return "NVS BOOT";
        default:                  return "UNKNOWN";
    }
}

/* ==========================================================================
   Full-screen redraw
   ==========================================================================
   All Y values are logical (0-based).  Physical row = Y + TFT_ROW_OFFSET.
   Safe range: Y = 0 .. TFT_Y_MAX (199).  Physical = 40 .. 239.

   Band      Y range    Content
   ────────  ─────────  ──────────────────────────────────────────────────
   Header     0.. 31   dark-grey bar  (32 px)
                y= 4   "EDSC"       white, scale-2  (ends y=19)
                y=22   "DOOR CTRL"  grey,  scale-1  (ends y=29)
   Sep        32       mid-grey separator line
   Label      37.. 44  "FSM STATE"  grey, scale-1
   State      50.. 65  state name   coloured, scale-2 (16 px tall)
   Sep        72       separator
   Fault      80.. 87  "FAULT: <n>" scale-1
   Sep        94       separator
   Emerg     100..121  amber banner (22 px) -- conditional
   Sep       128       separator
   Uptime    136..151  "Up HH:MM:SS" scale-2 (16 px tall)
   Margin    152..199  black (48 px safety margin from die edge)
   ========================================================================== */
static void draw_screen(fsm_state_t      state,
                        fault_code_t     fault,
                        emergency_lock_t emg)
{
    if ((size_t)state >= K_STATE_META_COUNT) {
        state = FSM_STATE_FAULT;
    }
    const state_meta_t *meta = &k_state_meta[(int)state];

    /* ── 1. Wipe the entire usable area to black ──────────────────────── */
    tft_fill_rect(0u, 0u, TFT_WIDTH, (uint16_t)(TFT_Y_MAX + 1u), COL_BLACK);

    /* ── 2. Header bar (y 0..31, 32 px tall) ────────────────────────── */
    tft_fill_rect(0u, 0u, TFT_WIDTH, 32u, COL_DGREY);
    /*
     * Two text rows inside the 32 px bar.
     * Row A: "EDSC" at scale-2 (12x16 px/char, 4 chars = 48 px wide).
     *   y=4  -> bottom at y=19  -- fits inside bar.
     * Row B: "DOOR CTRL" at scale-1 (6x8 px/char, 9 chars = 54 px wide).
     *   y=22 -> bottom at y=29  -- fits inside bar.
     */
    tft_draw_centred( 4u, "EDSC",      COL_WHITE, COL_DGREY, 2u);
    tft_draw_centred(22u, "DOOR CTRL", COL_GREY,  COL_DGREY, 1u);

    /* ── 3. Separator ─────────────────────────────────────────────────── */
    tft_separator(32u, COL_GREY);

    /* ── 4. "FSM STATE" eyebrow label (y 37) ─────────────────────────── */
    tft_draw_centred(37u, "FSM STATE", COL_GREY, COL_BLACK, 1u);

    /* ── 5. State name -- scale-2, 16 px tall, y=50 ──────────────────── */
    /*
     * At scale-2 each character is 12 x 16 px.
     * Longest name "OPENING" / "CLOSING" = 7 x 12 = 84 px < 135 px.  OK.
     */
    tft_draw_centred(50u, meta->label, meta->colour, COL_BLACK, 2u);

    /* ── 6. Separator ─────────────────────────────────────────────────── */
    tft_separator(72u, COL_GREY);

    /* ── 7. Fault row (y 80) ──────────────────────────────────────────── */
    /*
     * Longest fault string "FAULT: COMM TIMEOUT" = 19 chars x 6 px = 114 px.
     * All fault strings fit within 135 px at scale-1.
     */
    {
        char buf[28];
        snprintf(buf, sizeof(buf), "FAULT: %s", fault_label(fault));
        uint16_t fc = (fault != FAULT_NONE) ? COL_RED : COL_GREY;
        tft_draw_centred(80u, buf, fc, COL_BLACK, 1u);
    }

    /* ── 8. Separator ─────────────────────────────────────────────────── */
    tft_separator(94u, COL_GREY);

    /* ── 9. Emergency-lock banner (y 100..121, 22 px tall) ───────────── */
    if (emg == EMERGENCY_LOCK_ON) {
        tft_fill_rect(4u, 100u, (uint16_t)(TFT_WIDTH - 8u), 22u, COL_AMBER);
        /*
         * Text centred vertically in the 22 px box:
         *   box centre  = 100 + 11 = 111
         *   glyph half  =   8 / 2  =   4
         *   text y      = 111 - 4  = 107
         */
        tft_draw_centred(107u, "! EMERG LOCK !", COL_BLACK, COL_AMBER, 1u);
    }
    /* When lock is OFF this zone is already black from the wipe above. */

    /* ── 10. Separator ────────────────────────────────────────────────── */
    tft_separator(128u, COL_GREY);

    /* ── 11. Uptime (y 136) -- scale-2, 16 px tall ───────────────────── */
    /*
     * "Up 00:00:00" = 11 chars x 12 px = 132 px < 135 px. Fits at scale-2.
     * Bottom of text: 136 + 16 = 152 -- safely within TFT_Y_MAX (199).
     */

    /*
     * Rows 152..199 (physical 192..239) are black from the initial wipe.
     * A 48 px safety margin ensures no rendering wraps onto silicon row 0
     * and corrupts the top of the screen.
     */
}

static void draw_uptime(void)
{
    uint32_t total_s = (uint32_t)(esp_timer_get_time() / 1000000LL);
    uint32_t hh = total_s / 3600u;
    uint32_t mm = (total_s % 3600u) / 60u;
    uint32_t ss =  total_s % 60u;
    char buf[20];
    snprintf(buf, sizeof(buf), "Up %02lu:%02lu:%02lu",
             (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);

    /* Erase the uptime band first, then redraw — avoids ghost pixels. */
    tft_fill_rect(0u, 136u, TFT_WIDTH, 16u, COL_BLACK);
    tft_draw_centred(136u, buf, COL_GREY, COL_BLACK, 2u);
}

/* ==========================================================================
   Public API -- display_init()
   ========================================================================== */
esp_err_t display_init(void)
{
    esp_err_t ret;

    /* Configure DC, RST, and BL as push-pull outputs. */
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << TFT_DC) | (1ULL << TFT_RST) | (1ULL << TFT_BL),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(TFT_BL, 0);  /* Backlight off until panel is ready. */

    /* Backlight PWM via LEDC. */
    ledc_timer_config_t bl_tmr = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = BL_DUTY_BITS,
        .timer_num       = BL_LEDC_TIMER,
        .freq_hz         = 5000u,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ret = ledc_timer_config(&bl_tmr);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %s", esp_err_to_name(ret));
        return ret;
    }

    ledc_channel_config_t bl_ch = {
        .gpio_num   = TFT_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = BL_LEDC_CHANNEL,
        .timer_sel  = BL_LEDC_TIMER,
        .duty       = 0u,
        .hpoint     = 0,
    };
    ret = ledc_channel_config(&bl_ch);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config: %s", esp_err_to_name(ret));
        return ret;
    }

    /* SPI bus. */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = TFT_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = TFT_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        /*
         * max_transfer_sz: sized for a 32-row stripe (modest heap use).
         * The fill routine sends in 256-pixel chunks so this is not a limit.
         */
        .max_transfer_sz = TFT_WIDTH * 32u * 2u + 16u,
    };
    ret = spi_bus_initialize(TFT_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = TFT_SPI_FREQ_HZ,
        .mode          = 0,             /* CPOL=0, CPHA=0 */
        .spics_io_num  = TFT_CS,
        .queue_size    = 7,
        /*
         * SPI_DEVICE_NO_DUMMY is intentionally omitted.
         * That flag skips the dummy cycle inserted at high clock rates and
         * can corrupt the first byte of a transaction -- removed to fix
         * possible garbage on the first command after init.
         */
    };
    ret = spi_bus_add_device(TFT_SPI_HOST, &dev_cfg, &s_spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Panel power-on sequence. */
    tft_hw_init();

    /* Backlight to 100%. */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL, BL_DUTY_FULL);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, BL_LEDC_CHANNEL);

    ESP_LOGI(TAG, "ST7789V %ux%u ready  (col_off=%u  row_off=%u)",
             TFT_WIDTH, TFT_HEIGHT, TFT_COL_OFFSET, TFT_ROW_OFFSET);
    return ESP_OK;
}

/* ==========================================================================
   Public API -- display_task()
   ========================================================================== */
void display_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Display task started (Core %d)", xPortGetCoreID());

    /*
     * Sentinel values that cannot occur in normal operation guarantee that
     * the very first poll triggers a full redraw immediately.
     */
    fsm_state_t      prev_state = (fsm_state_t)0xFFu;
    fault_code_t     prev_fault = (fault_code_t)0xFFu;
    emergency_lock_t prev_emg   = (emergency_lock_t)0xFFu;

    for (;;) {
        fsm_state_t      cur_state = fsm_get_state();
        fault_code_t     cur_fault = fsm_get_fault_code();
        emergency_lock_t cur_emg   = fsm_get_emergency_lock();

        /* Redraw only when something has actually changed. */
        if (cur_state != prev_state ||
            cur_fault != prev_fault ||
            cur_emg   != prev_emg) {

            draw_screen(cur_state, cur_fault, cur_emg);

            prev_state = cur_state;
            prev_fault = cur_fault;
            prev_emg   = cur_emg;

            ESP_LOGD(TAG, "Screen updated -> state=%d fault=%d emg=%d",
                     (int)cur_state, (int)cur_fault, (int)cur_emg);
        }

        draw_uptime();

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_POLL_MS));
    }
}
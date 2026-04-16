// =============================================================
// TFT_eSPI User_Setup.h — TTGO T-Display (ESP32 + ST7789)
// Place in include/ so PlatformIO picks it up via -Iinclude
// =============================================================
#pragma once

//#define is just text replacement, not memory allocation
#define ST7789_DRIVER
#define TFT_WIDTH 135
#define TFT_HEIGHT 240

// TTGO T-Display pin mapping
#define TFT_MISO -1
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS 5
#define TFT_DC 16
#define TFT_RST 23
#define TFT_BL 4    // Backlight PWM
#define TOUCH_CS -1 // TTGO T-Display has no built-in touch screen

// Fonts to load
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_GFXFF
#define SMOOTH_FONT

#define SPI_FREQUENCY 40000000
#define SPI_READ_FREQUENCY 20000000

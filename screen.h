#pragma once
#include <cstdint>
#include <cstddef>
#include <signal.h>

// --------------------------------------
//  OLED / SPI Functions
// --------------------------------------
void sendCommand(int spi, uint8_t cmd);
void sendData(int spi, const uint8_t* data, size_t len);
void initDisplay(int spi);
void clearBuffer();
void drawPixel(int x, int y);
void drawLine(int x0, int y0, int x1, int y1);
void updateDisplay(int spi);
void gracefulExit(int sig);

// --------------------------------------
//  ADSR Functions
// --------------------------------------
float ADSR(float attack, float decay, float sustain, float release,
           bool trig, float t, float lvl);
void drawADSR(float attack, float decay, float sustain, float release);

// --------------------------------------
//  SSD1306 Helper
// --------------------------------------
void clearSSD1306(int spi);

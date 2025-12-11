#pragma once
#include <cstdint>
#include <cstddef>
#include <signal.h>

// OLED / SPI functions
void sendCommand(int spi, uint8_t cmd);
void sendData(int spi, const uint8_t* data, size_t len);
void initDisplay(int spi);
void clearBuffer();
void drawPixel(int x, int y);
void drawLine(int x0, int y0, int x1, int y1);
void updateDisplay(int spi);
void gracefulExit(int sig);

// Optional: ADSR visualization (if you want a drawADSR helper)
void drawADSR(float attack, float decay, float sustain, float release, int spi);

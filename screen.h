#pragma once

void drawLine(int x0, int y0, int x1, int y1);
void drawPixel(int x, int y);
void clearBuffer();
void updateDisplay(int spi);
void drawADSR(float attack, float decay, float sustain, float release);
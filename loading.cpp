// rx2_loading_standalone.cpp
// Standalone RX-2 loading screen — run this WHILE your main project compiles.
//
// Compile:
//   g++ rx2_loading_standalone.cpp -o rx2_loading -lpigpio -lrt -lpthread
//
// Usage (run alongside your build):
//   ./rx2_loading &
//   make
//   kill %1
//
//   OR use the build wrapper script below to handle it automatically.

#include <cmath>
#include <cstring>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <pigpio.h>

// -----------------------------------------------------------------------
//  SH1106 SPI config — match your existing setup
// -----------------------------------------------------------------------
#define SPI_CHANNEL  0
#define SPI_SPEED    8000000
#define PIN_DC       25
#define PIN_RES      24

const int WIDTH  = 127;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int spi;

// -----------------------------------------------------------------------
//  SPI helpers
// -----------------------------------------------------------------------
void sendCommand(uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

void sendData(const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

// -----------------------------------------------------------------------
//  Display init / update
// -----------------------------------------------------------------------
void initDisplay() {
    gpioWrite(PIN_RES, 0); usleep(100000);
    gpioWrite(PIN_RES, 1); usleep(100000);

    sendCommand(0xAE);
    sendCommand(0xD5); sendCommand(0x80);
    sendCommand(0xA8); sendCommand(0x3F);
    sendCommand(0xD3); sendCommand(0x00);
    sendCommand(0x40);
    sendCommand(0xAD); sendCommand(0x8B);
    sendCommand(0xA1);
    sendCommand(0xC8);
    sendCommand(0xDA); sendCommand(0x12);
    sendCommand(0x81); sendCommand(0xCF);
    sendCommand(0xD9); sendCommand(0xF1);
    sendCommand(0xDB); sendCommand(0x40);
    sendCommand(0xA4);
    sendCommand(0xA6);
    sendCommand(0xAF);
}

void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));
}

void clearScreen() {
    uint8_t empty[132];
    memset(empty, 0x00, sizeof(empty));
    for (int page = 0; page < 8; page++) {
        sendCommand(0xB0 + page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(empty, 132);
    }
}

void updateDisplay() {
    for (int page = 0; page < HEIGHT / 8; page++) {
        sendCommand(0xB0 + page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(&buffer[page * WIDTH], WIDTH);
    }
}

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y / 8) * WIDTH] |= (1 << (y % 8));
}

// -----------------------------------------------------------------------
//  Glyphs  (5×7 column-major bitmaps, LSB = top row)
// -----------------------------------------------------------------------
static const uint8_t GLYPH_R[5]    = { 0x7F, 0x09, 0x09, 0x09, 0x76 };
static const uint8_t GLYPH_X[5]    = { 0x63, 0x14, 0x08, 0x14, 0x63 };
static const uint8_t GLYPH_DASH[5] = { 0x08, 0x08, 0x08, 0x08, 0x08 };
static const uint8_t GLYPH_2[5]    = { 0x62, 0x51, 0x49, 0x49, 0x46 };

void drawBigGlyph(int x, int y, const uint8_t* glyph, int S) {
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                for (int dx = 0; dx < S; dx++)
                    for (int dy = 0; dy < S; dy++)
                        drawPixel(x + col * S + dx, y + row * S + dy);
            }
        }
    }
}

void drawRX2(int centerX, int topY) {
    const int S   = 3;
    const int GW  = 5 * S;
    const int GAP = S;

    const uint8_t* glyphs[] = { GLYPH_R, GLYPH_X, GLYPH_DASH, GLYPH_2 };
    const int n = 4;

    int totalW = n * GW + (n - 1) * GAP;
    int startX = centerX - totalW / 2;

    for (int i = 0; i < n; i++)
        drawBigGlyph(startX + i * (GW + GAP), topY, glyphs[i], S);
}

// -----------------------------------------------------------------------
//  Bouncing dots
// -----------------------------------------------------------------------
void drawFilledCircle(int cx, int cy, int r) {
    for (int px = -r; px <= r; px++) {
        int half = (int)sqrtf((float)(r * r - px * px));
        for (int py = -half; py <= half; py++)
            drawPixel(cx + px, cy + py);
    }
}

// -----------------------------------------------------------------------
//  Graceful exit — clear screen on Ctrl-C or kill
// -----------------------------------------------------------------------
volatile sig_atomic_t running = 1;

void onSignal(int) {
    running = 0;
}

// -----------------------------------------------------------------------
//  Main
// -----------------------------------------------------------------------
int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    signal(SIGINT,  onSignal);
    signal(SIGTERM, onSignal);

    gpioSetMode(PIN_DC,  PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    spi = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (spi < 0) {
        std::cerr << "SPI open failed\n";
        gpioTerminate();
        return 1;
    }

    initDisplay();
    clearBuffer();
    clearScreen();
    updateDisplay();

    // Animation state
    float phases[3] = { 0.0f, 2.094f, 4.189f };  // 120° apart
    const float SPEED   = 0.12f;
    const int   DOT_R   = 3;
    const int   DOT_GAP = 12;
    const int   BOUNCE  = 5;
    const int   BASE_Y  = 52;
    const int   CX      = 63;

    const int dotX[3] = { CX - DOT_GAP, CX, CX + DOT_GAP };

    while (running) {
        clearBuffer();

        drawRX2(CX, 10);

        for (int d = 0; d < 3; d++) {
            phases[d] += SPEED;
            int cy = BASE_Y - (int)(sinf(phases[d]) * BOUNCE);
            drawFilledCircle(dotX[d], cy, DOT_R);
        }

        updateDisplay();
        usleep(33000);  // ~30 fps — light on CPU during your compile
    }

    // Clean exit
    clearBuffer();
    clearScreen();
    updateDisplay();
    spiClose(spi);
    gpioTerminate();
    return 0;
}
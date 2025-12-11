#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <cstring>
#include <signal.h>

#define SPI_CHANNEL 0          // CE0
#define SPI_SPEED 8000000      // 8 MHz
#define PIN_DC 25              // Data/Command pin
#define PIN_RES 24             // Reset pin

const int WIDTH = 128;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int global_spi_handle = -1;   // Needed for safe exit

void clearBuffer();
void updateDisplay(int spi);
void gracefulExit(int signum);

// --------------------------------------
//  SPI Send Helpers
// --------------------------------------
void sendCommand(int spi, uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

void sendData(int spi, const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

// --------------------------------------
//  SH1106 Init
// --------------------------------------
void initDisplay(int spi) {
    gpioWrite(PIN_RES, 0);
    usleep(100000);
    gpioWrite(PIN_RES, 1);
    usleep(100000);

    sendCommand(spi, 0xAE);
    sendCommand(spi, 0xD5); sendCommand(spi, 0x80);
    sendCommand(spi, 0xA8); sendCommand(spi, 0x3F);
    sendCommand(spi, 0xD3); sendCommand(spi, 0x00);
    sendCommand(spi, 0x40);
    sendCommand(spi, 0xAD); sendCommand(spi, 0x8B);
    sendCommand(spi, 0xA1);
    sendCommand(spi, 0xC8);
    sendCommand(spi, 0xDA); sendCommand(spi, 0x12);
    sendCommand(spi, 0x81); sendCommand(spi, 0xCF);
    sendCommand(spi, 0xD9); sendCommand(spi, 0xF1);
    sendCommand(spi, 0xDB); sendCommand(spi, 0x40);
    sendCommand(spi, 0xA4);
    sendCommand(spi, 0xA6);
    sendCommand(spi, 0xAF);
}

// --------------------------------------
//  Buffer Management
// --------------------------------------
void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));
}

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] |= (1 << (y % 8));
}

// Bresenham Line
void drawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        drawPixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Send buffer to OLED
void updateDisplay(int spi) {
    for (int page = 0; page < HEIGHT/8; page++) {
        sendCommand(spi, 0xB0 + page);
        sendCommand(spi, 0x00);
        sendCommand(spi, 0x10);
        sendData(spi, &buffer[page * WIDTH], WIDTH);
    }
}

// --------------------------------------
//  ADSR Envelope Function
// --------------------------------------
float ADSR(float attack,float decay,float sustain,float release,
           bool trig,float t,float lvl) {

    float curve = 3.0f;

    if (trig) {
        if (t < attack)
            return powf(t/attack, curve) * lvl;
        else if (t < attack + decay)
            return (1.0 - powf((t-attack)/decay, 1.0/curve) * (1.0 - sustain)) * lvl;
        else
            return sustain * lvl;
    } else {
        if (t < release)
            return (1.0 - powf(t/release, 1.0/curve)) * (sustain * lvl);
        else
            return 0.0f;
    }
}

// --------------------------------------
//  Draw ADSR Curve
// --------------------------------------
void drawADSR(float attack, float decay, float sustain, float release) {
    clearBuffer();

    float sustainView = 0.2;
    float totalTime = attack + decay + sustainView + release;

    int lastY = -1;
    for (int x = 0; x < WIDTH; x++) {
        float t = (float)x / WIDTH * totalTime;

        float env;
        if (t < attack + decay + sustainView)
            env = ADSR(attack, decay, sustain, release, true, t, 1.0);
        else
            env = ADSR(attack, decay, sustain, release, false,
                       t - (attack + decay + sustainView),
                       1.0);

        int y = HEIGHT - 1 - int(env * (HEIGHT - 1));
        if (lastY >= 0) drawLine(x-1, lastY, x, y);
        lastY = y;
    }
    updateDisplay(global_spi_handle);
}

// --------------------------------------
//  Ctrl-C Cleanup
// --------------------------------------
void gracefulExit(int signum) {
    std::cout << "\nClearing OLED before exit...\n";

    clearBuffer();
    updateDisplay(global_spi_handle);

    spiClose(global_spi_handle);
    gpioTerminate();

    std::cout << "Done.\n";
    exit(0);
}

// --------------------------------------
//  Main
// --------------------------------------
int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    signal(SIGINT, gracefulExit);

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    global_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (global_spi_handle < 0) {
        std::cerr << "SPI failed\n";
        return 1;
    }

    initDisplay(global_spi_handle);


    

    return 0;
}

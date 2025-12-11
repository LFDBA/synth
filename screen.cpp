#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <cstdint>
#include <signal.h>

#define SPI_CHANNEL 0
#define SPI_SPEED 8000000
#define PIN_DC 25
#define PIN_RES 24

const int WIDTH = 128;
const int HEIGHT = 64;

int spiHandle = -1;
uint8_t buffer[WIDTH * (HEIGHT / 8)];

//
// ==== SSD1306 COMMAND HELPERS ====
//
void sendCommand(uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spiHandle, (char*)&cmd, 1);
}

void sendData(const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spiHandle, (char*)data, len);
}

//
// ==== CLEAR SCREEN (REAL SSD1306 CLEAR) ====
//
void clearSSD1306() {
    uint8_t zeros[128];
    memset(zeros, 0x00, 128);

    for(int page = 0; page < 8; page++) {
        sendCommand(0xB0 | page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(zeros, 128);
    }
}

//
// ==== SIGINT HANDLER (Ctrl+C) ====
//
void handleExit(int sig) {
    std::cout << "\nCtrl+C detected, clearing display...\n";

    // HARD RESET
    gpioWrite(PIN_RES, 0);
    usleep(80000);
    gpioWrite(PIN_RES, 1);
    usleep(80000);

    // INIT MINIMUM SO CLEAR WORKS
    sendCommand(0xAE);    // display off
    sendCommand(0x20);    // Memory mode
    sendCommand(0x00);    // horizontal
    clearSSD1306();
    sendCommand(0xAF);    // display on (blank)

    if (spiHandle >= 0) spiClose(spiHandle);
    gpioTerminate();
    exit(0);
}

//
// ==== INITIALIZE SSD1306 (SPI) ====
//
void initDisplay() {
    gpioWrite(PIN_RES, 0);
    usleep(100000);
    gpioWrite(PIN_RES, 1);
    usleep(100000);

    sendCommand(0xAE); // display off
    sendCommand(0xD5); sendCommand(0x80);
    sendCommand(0xA8); sendCommand(0x3F);
    sendCommand(0xD3); sendCommand(0x00);
    sendCommand(0x40);
    sendCommand(0x8D); sendCommand(0x14);
    sendCommand(0x20); sendCommand(0x00);
    sendCommand(0xA1);
    sendCommand(0xC8);
    sendCommand(0xDA); sendCommand(0x12);
    sendCommand(0x81); sendCommand(0x7F);
    sendCommand(0xD9); sendCommand(0xF1);
    sendCommand(0xDB); sendCommand(0x40);
    sendCommand(0xA4);
    sendCommand(0xA6);
    sendCommand(0xAF); // display on
}


//
// ==== BUFFER HELPERS ====
//
void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));
}

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y / 8) * WIDTH] |= (1 << (y % 8));
}

void drawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true) {
        drawPixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void updateDisplay() {
    for (int page = 0; page < HEIGHT/8; page++) {
        sendCommand(0xB0 | page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(&buffer[page * WIDTH], WIDTH);
    }
}

//
// ==== ADSR ====
//
float ADSR(float attack,float decay,float sustain,float release,bool trig,float t,float lvl){
    float c = 3.0;
    if(trig){
        if(t < attack) return powf(t/attack,c)*lvl;
        if(t < attack + decay) return (1 - powf((t-attack)/decay,1.0/c)*(1-sustain))*lvl;
        return sustain*lvl;
    } else {
        if(t < release) return (1 - powf(t/release,1.0/c))*(sustain*lvl);
        return 0;
    }
}

void drawADSR() {
    clearBuffer();
    float sustainVis = 0.2;
    float attack = 0.5, decay = 0.2, sustain = 0.8, release = 0.8;
    float total = attack + decay + sustainVis + release;

    int lastY = -1;
    for(int x = 0; x < WIDTH; x++) {
        float t = (float)x / WIDTH * total;
        float env;

        if(t < attack + decay + sustainVis)
            env = ADSR(attack, decay, sustain, release, true, t, 1);
        else
            env = ADSR(attack, decay, sustain, release, false, t - (attack+decay+sustainVis), 1);

        int y = HEIGHT - 1 - int(env * (HEIGHT - 1));
        if(lastY >= 0) drawLine(x-1, lastY, x, y);
        lastY = y;
    }
}


//
// ==== MAIN ====
//
int main() {
    signal(SIGINT, handleExit);   // catch Ctrl+C

    if(gpioInitialise() < 0) {
        std::cerr << "Pigpio init failed!\n";
        return 1;
    }

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    spiHandle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if(spiHandle < 0) {
        std::cerr << "SPI open failed!\n";
        return 1;
    }

    initDisplay();

    while(true) {
        drawADSR();
        updateDisplay();
        usleep(50000);
    }

    return 0; // never reached
}

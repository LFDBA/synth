#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <cstdint>

#define SPI_CHANNEL 0        // CE0
#define SPI_SPEED 8000000    // 8 MHz
#define PIN_DC 25            // Data/Command pin
#define PIN_RES 24           // Reset pin

const int WIDTH = 128;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)]; // 128 x 64 / 8

// Send command byte
void sendCommand(int spi, uint8_t cmd) {
    gpioWrite(PIN_DC, 0); // Command mode
    spiWrite(spi, (char*)&cmd, 1);
}

// Send data bytes
void sendData(int spi, const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1); // Data mode
    spiWrite(spi, (char*)data, len);
}

// Initialize SH1106
void initDisplay(int spi) {
    gpioWrite(PIN_RES, 0);
    usleep(100000); // 100ms
    gpioWrite(PIN_RES, 1);
    usleep(100000);

    sendCommand(spi, 0xAE); // Display off
    sendCommand(spi, 0xD5); // Set display clock divide
    sendCommand(spi, 0x80);
    sendCommand(spi, 0xA8); // Set multiplex
    sendCommand(spi, 0x3F);
    sendCommand(spi, 0xD3); // Set display offset
    sendCommand(spi, 0x00);
    sendCommand(spi, 0x40); // Set start line
    sendCommand(spi, 0xAD); // DC-DC on
    sendCommand(spi, 0x8B);
    sendCommand(spi, 0xA1); // Seg remap
    sendCommand(spi, 0xC8); // COM scan dec
    sendCommand(spi, 0xDA); // COM pins
    sendCommand(spi, 0x12);
    sendCommand(spi, 0x81); // Contrast
    sendCommand(spi, 0xCF);
    sendCommand(spi, 0xD9); // Precharge
    sendCommand(spi, 0xF1);
    sendCommand(spi, 0xDB); // Vcomh
    sendCommand(spi, 0x40);
    sendCommand(spi, 0xA4); // Entire display on
    sendCommand(spi, 0xA6); // Normal display
    sendCommand(spi, 0xAF); // Display on
}

// Clear buffer
void clearBuffer() {
    std::fill(buffer, buffer + sizeof(buffer), 0);
}

// Draw pixel in buffer
void drawPixel(int x, int y) {
    if(x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y / 8) * WIDTH] |= (1 << (y % 8));
}

// Draw line (Bresenham)
void drawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while(true) {
        drawPixel(x0, y0);
        if(x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if(e2 >= dy) { err += dy; x0 += sx; }
        if(e2 <= dx) { err += dx; y0 += sy; }
    }
}

// Send buffer to display
void updateDisplay(int spi) {
    for(int page = 0; page < HEIGHT/8; page++) {
        sendCommand(spi, 0xB0 + page); // Set page
        sendCommand(spi, 0x00);        // Set lower column
        sendCommand(spi, 0x10);        // Set higher column
        sendData(spi, &buffer[page * WIDTH], WIDTH);
    }
}

// ADSR function
float ADSR(float attack,float decay,float sustain,float release,bool trig,float t,float lvl){
    float curvature=3.0;
    if(trig){
        if(t<attack) return powf(t/attack,curvature)*lvl;
        else if(t<attack+decay) return (1.0 - powf((t-attack)/decay,1.0/curvature)*(1.0-sustain))*lvl;
        else return sustain*lvl;
    } else {
        if(t<release) return (1.0 - powf(t/release,1.0/curvature))*(sustain*lvl);
        else return 0.0f;
    }
}

// Draw ADSR envelope into buffer
void drawADSR() {
    clearBuffer();
    float sustainVisTime = 0.2;
    float attack = 0.5, decay = 0.2, sustain = 0.8, release = 0.8;
    float totalTime = attack + decay + sustainVisTime + release;

    int lastY = -1;
    for(int x = 0; x < WIDTH; x++) {
        float t = (float)x / WIDTH * totalTime;
        float env;
        if(t < attack + decay + sustainVisTime) {
            env = ADSR(attack, decay, sustain, release, true, t, 1.0);
        } else {
            float tRelease = t - (attack + decay + sustainVisTime);
            env = ADSR(attack, decay, sustain, release, false, tRelease, 1.0);
        }
        int y = HEIGHT - 1 - int(env * (HEIGHT-1));
        if(lastY >= 0) drawLine(x-1, lastY, x, y);
        lastY = y;
    }
}

int main() {
    if(gpioInitialise() < 0) {
        std::cerr << "Pigpio init failed!" << std::endl;
        return 1;
    }
    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    int spi = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if(spi < 0) {
        std::cerr << "SPI open failed!" << std::endl;
        return 1;
    }

    initDisplay(spi);

    while(true) {
        drawADSR();
        updateDisplay(spi);
        usleep(50000); // 50ms refresh
    }

    clearBuffer();
    updateDisplay(spi);
    spiClose(spi);
    gpioTerminate();
    return 0;
}

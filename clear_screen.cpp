#include <cmath>
#include <cstring>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <unistd.h>
#include <iostream>
#include <pigpio.h>

#define SPI_CHANNEL  0
#define SPI_SPEED    4000000
#define PIN_DC       25
#define PIN_RES      24

// Set to 0 if you swap back to the older SH1106-based module.
#define OLED_USE_SSD1309 1

const int WIDTH  = 128;
const int HEIGHT = 64;
const int PAGE_COUNT = HEIGHT / 8;
const int COLUMN_OFFSET = OLED_USE_SSD1309 ? 0 : 2;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int spi;

void sendCommand(uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

void sendData(const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

void setPageStart(int page) {
    sendCommand(0xB0 + page);
    sendCommand(COLUMN_OFFSET & 0x0F);
    sendCommand(0x10 | ((COLUMN_OFFSET >> 4) & 0x0F));
}

void initDisplay() {
    gpioWrite(PIN_RES, 0); usleep(100000);
    gpioWrite(PIN_RES, 1); usleep(100000);

    if (OLED_USE_SSD1309) {
        sendCommand(0xFD); sendCommand(0x12); // unlock SSD1309 commands
        sendCommand(0x20); sendCommand(0x00); // horizontal addressing
    }

    sendCommand(0xAE);
    sendCommand(0xD5); sendCommand(0x80);
    sendCommand(0xA8); sendCommand(0x3F);
    sendCommand(0xD3); sendCommand(0x00);
    sendCommand(0x40);
    if (!OLED_USE_SSD1309) {
        sendCommand(0xAD); sendCommand(0x8B);
    }
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
    static const uint8_t empty[WIDTH * PAGE_COUNT] = {0};

    if (OLED_USE_SSD1309) {
        sendCommand(0x21);
        sendCommand(0x00);
        sendCommand(WIDTH - 1);
        sendCommand(0x22);
        sendCommand(0x00);
        sendCommand(PAGE_COUNT - 1);
        sendData(empty, sizeof(empty));
        return;
    }

    uint8_t pageEmpty[WIDTH];
    memset(pageEmpty, 0x00, sizeof(pageEmpty));
    for (int page = 0; page < PAGE_COUNT; page++) {
        setPageStart(page);
        sendData(pageEmpty, WIDTH);
    }
}

void updateDisplay() {
    if (OLED_USE_SSD1309) {
        sendCommand(0x21);
        sendCommand(0x00);
        sendCommand(WIDTH - 1);
        sendCommand(0x22);
        sendCommand(0x00);
        sendCommand(PAGE_COUNT - 1);
        sendData(buffer, sizeof(buffer));
        return;
    }

    for (int page = 0; page < PAGE_COUNT; page++) {
        setPageStart(page);
        sendData(&buffer[page * WIDTH], WIDTH);
    }
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

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
    spiClose(spi);
    gpioTerminate();
    return 0;
}

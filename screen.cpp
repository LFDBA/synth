#include <pigpio.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>

#define SPI_CHANNEL 0
#define SPI_SPEED   4000000
#define PIN_DC      25
#define PIN_RES     24

// Set to 0 if you swap back to the older SH1106-based module.
#define OLED_USE_SSD1309 1
const int WIDTH = 128;
const int HEIGHT = 64;
const int PAGE_COUNT = HEIGHT / 8;
const int COLUMN_OFFSET = OLED_USE_SSD1309 ? 0 : 2;

int global_spi_handle = -1;

// Send a command to the display
void sendCommand(int spi, uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

// Send data to the display
void sendData(int spi, const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

void setPageStart(int spi, int page) {
    sendCommand(spi, 0xB0 + page);
    sendCommand(spi, COLUMN_OFFSET & 0x0F);
    sendCommand(spi, 0x10 | ((COLUMN_OFFSET >> 4) & 0x0F));
}

void initDisplay(int spi) {
    gpioWrite(PIN_RES, 0);
    usleep(200000);
    gpioWrite(PIN_RES, 1);
    usleep(200000);

    if (OLED_USE_SSD1309) {
        sendCommand(spi, 0xFD); sendCommand(spi, 0x12);
        sendCommand(spi, 0x20); sendCommand(spi, 0x00);
    }

    sendCommand(spi, 0xAE);
    sendCommand(spi, 0xD5); sendCommand(spi, 0x80);
    sendCommand(spi, 0xA8); sendCommand(spi, 0x3F);
    sendCommand(spi, 0xD3); sendCommand(spi, 0x00);
    sendCommand(spi, 0x40);
    if (!OLED_USE_SSD1309) {
        sendCommand(spi, 0xAD); sendCommand(spi, 0x8B);
    }
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

// Clear the currently selected OLED controller.
void clearScreen() {
    if(global_spi_handle < 0) {
        std::cerr << "SPI not initialized!\n";
        return;
    }

    static const uint8_t empty[WIDTH * PAGE_COUNT] = {0};
    if (OLED_USE_SSD1309) {
        sendCommand(global_spi_handle, 0x21);
        sendCommand(global_spi_handle, 0x00);
        sendCommand(global_spi_handle, WIDTH - 1);
        sendCommand(global_spi_handle, 0x22);
        sendCommand(global_spi_handle, 0x00);
        sendCommand(global_spi_handle, PAGE_COUNT - 1);
        sendData(global_spi_handle, empty, sizeof(empty));
        return;
    }

    uint8_t pageEmpty[WIDTH];
    memset(pageEmpty, 0x00, sizeof(pageEmpty));
    for (int page = 0; page < PAGE_COUNT; page++) {
        setPageStart(global_spi_handle, page);
        sendData(global_spi_handle, pageEmpty, WIDTH);
    }
}

int main() {
    if(gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    // Open SPI
    global_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if(global_spi_handle < 0) {
        std::cerr << "SPI open failed\n";
        gpioTerminate();
        return 1;
    }

    initDisplay(global_spi_handle);

    clearScreen();
    std::cout << "Screen cleared!\n";

    spiClose(global_spi_handle);
    gpioTerminate();
    return 0;
}

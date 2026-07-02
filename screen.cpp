#include <pigpio.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>

#define SPI_CHANNEL 0
#define SPI_SPEED   8000000
#define PIN_DC      25
#define PIN_RES     24

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

void initDisplay(int spi) {
    gpioWrite(PIN_RES, 0);
    usleep(100000);
    gpioWrite(PIN_RES, 1);
    usleep(100000);

    sendCommand(spi, 0xAE);
    sendCommand(spi, 0xFD); sendCommand(spi, 0x12);
    sendCommand(spi, 0x00);
    sendCommand(spi, 0x10);
    sendCommand(spi, 0x40);
    sendCommand(spi, 0x81); sendCommand(spi, 0xBF);
    sendCommand(spi, 0xA1);
    sendCommand(spi, 0xA6);
    sendCommand(spi, 0xA8); sendCommand(spi, 0x3F);
    sendCommand(spi, 0xC8);
    sendCommand(spi, 0xD3); sendCommand(spi, 0x00);
    sendCommand(spi, 0xD5); sendCommand(spi, 0xA0);
    sendCommand(spi, 0xD9); sendCommand(spi, 0xF1);
    sendCommand(spi, 0xDA); sendCommand(spi, 0x12);
    sendCommand(spi, 0xDB); sendCommand(spi, 0x34);
    sendCommand(spi, 0xA4);
    sendCommand(spi, 0xA6);
    sendCommand(spi, 0xAF);
}

// Full SSD1309 clear
void clearScreen() {
    if(global_spi_handle < 0) {
        std::cerr << "SPI not initialized!\n";
        return;
    }

    uint8_t empty[128];
    memset(empty, 0x00, sizeof(empty));

    for(int page = 0; page < 8; page++) {
        sendCommand(global_spi_handle, 0xB0 + page); // select page
        sendCommand(global_spi_handle, 0x00);        // lower column
        sendCommand(global_spi_handle, 0x10);        // upper column
        sendData(global_spi_handle, empty, sizeof(empty));
    }
}

// Example init and clear sequence
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

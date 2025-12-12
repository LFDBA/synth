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

// Full SH1106 clear
void clearScreen() {
    if(global_spi_handle < 0) {
        std::cerr << "SPI not initialized!\n";
        return;
    }

    // Clear the OLED's internal memory (all 8 pages)
    uint8_t empty[132];      // SH1106 has 132 columns internally
    memset(empty, 0x00, sizeof(empty));

    for(int page = 0; page < 8; page++) {
        sendCommand(global_spi_handle, 0xB0 + page); // select page
        sendCommand(global_spi_handle, 0x00);        // lower column
        sendCommand(global_spi_handle, 0x10);        // upper column
        sendData(global_spi_handle, empty, 132);     // write zeros to all columns
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

    // Reset display
    gpioWrite(PIN_RES, 0);
    usleep(200000); // hold 200ms
    gpioWrite(PIN_RES, 1);
    usleep(200000);

    // Open SPI
    global_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if(global_spi_handle < 0) {
        std::cerr << "SPI open failed\n";
        gpioTerminate();
        return 1;
    }

    clearScreen();
    std::cout << "Screen cleared!\n";

    spiClose(global_spi_handle);
    gpioTerminate();
    return 0;
}

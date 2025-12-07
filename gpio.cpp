#include <iostream>
#include <pigpio.h>
#include <unistd.h>

int main() {
    // Don't call gpioInitialise() if pigpiod is running
    // Just try to use the library functions directly

    int pin = 26;
    gpioSetMode(pin, PI_INPUT);

    while (true) {
        int level = gpioRead(pin);
        std::cout << (level ? "HIGH" : "LOW") << std::endl;
        usleep(100000); // 100ms
    }

    // No gpioTerminate() if pigpiod is running, just exit
    return 0;
}

#include <iostream>
#include <pigpio.h>
#include <unistd.h>

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    int pin = 26;
    gpioSetMode(pin, PI_INPUT);

    while (true) {
        int level = gpioRead(pin);
        std::cout << (level ? "HIGH" : "LOW") << std::endl;
        usleep(100000); // 100ms
    }

    gpioTerminate();
    return 0;
}

#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>

int main() {
    std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Set all pins as input with pull-down initially
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    while (true) {
        for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
            int outPin = pins[outIdx];
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
                // Skip previous and current output pins
                if (inIdx == outIdx || inIdx == outIdx - 1) continue;
                if (gpioRead(pins[inIdx]) == 1) {
                    int keyNumber = inIdx + outIdx * 12 + 1; // adjust as needed
                    std::cout << "Key pressed: " << keyNumber << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);
        }
    }

    gpioTerminate();
    return 0;
}

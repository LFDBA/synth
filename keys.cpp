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


    while (true) {
        for (size_t outIdx = 0; outIdx < 4; ++outIdx) {
            int outPin = pins[outIdx];

            gpioWrite(outPin, 1); // Drive output high

            for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
                // Skip the current and previous output pin
                if (inIdx == outIdx || inIdx == outIdx - 1) continue;

                if (gpioRead(pins[inIdx]) == 1) {
                    // Calculate key number exactly like in your repeated code
                    int keyNumber = inIdx + outIdx * 12 + 1;
                    std::cout << "Key pressed: " << keyNumber << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            gpioWrite(outPin, 0); // Reset output
        }
    }

    gpioTerminate();
    return 0;
}

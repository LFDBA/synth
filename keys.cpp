#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

// GPIO pins (BCM)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Debounce threshold
const int DEBOUNCE_THRESHOLD = 3;

// Track previous key states and counters
std::map<int,bool> keyStates;
std::map<int,int> keyCounters;

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Set all pins as input with pull-down initially
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
            int outPin = pins[outIdx];

            // Drive this pin high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Small delay for stabilization
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
                if (inIdx == outIdx) continue; // skip the pin currently driven

                int inPin = pins[inIdx];

                // Create a unique key number for this combination
                int keyNum = outIdx * 100 + inIdx; // or any formula that guarantees uniqueness

                bool pressed = gpioRead(inPin);

                if (pressed) {
                    keyCounters[keyNum]++;
                    if (keyCounters[keyNum] >= DEBOUNCE_THRESHOLD && !keyStates[keyNum]) {
                        keyStates[keyNum] = true;
                        std::cout << "Key pressed: " << keyNum << std::endl;
                    }
                } else {
                    keyCounters[keyNum] = 0;
                    if (keyStates[keyNum]) {
                        keyStates[keyNum] = false;
                        std::cout << "Key released: " << keyNum << std::endl;
                    }
                }
            }

            // Reset output pin to input with pull-down
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

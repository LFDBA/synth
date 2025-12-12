#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

// GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Simple debounce threshold (number of consecutive reads)
const int DEBOUNCE_THRESHOLD = 3;

// Key map: (outPin, inPin) -> key number
std::map<std::pair<int,int>, int> keyMap;

// Track stable key states
std::map<int,bool> keyStates;

// Track debounce counters per key
std::map<int,int> keyCounters;

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Initialize all pins as input with pull-down
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    // Build key map using output intervals of 4, input intervals of 1
    int keyNum = 1;
    for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
        for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
            if (inIdx == outIdx) continue; // skip same pin
            keyMap[{pins[outIdx], pins[inIdx]}] = keyNum;
            keyStates[keyNum] = false;
            keyCounters[keyNum] = 0;
            keyNum++;
        }
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        for (auto outPin : pins) {
            // Set output high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Give line some time to stabilize
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // Scan all other pins
            for (auto inPin : pins) {
                if (inPin == outPin) continue;

                int keyNum = keyMap[{outPin, inPin}];
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

            // Reset output to input with pull-down before next iteration
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            // Small delay to reduce ghosting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

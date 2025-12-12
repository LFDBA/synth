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

    // Generate unique key IDs for every (outPin, inPin) pair
    std::vector<std::pair<int,int>> keyPairs;
    for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
        for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
            if (inIdx == outIdx) continue;
            keyPairs.push_back({pins[outIdx], pins[inIdx]});
            int keyId = keyPairs.size(); // 1,2,3,... total keys
            keyStates[keyId] = false;
            keyCounters[keyId] = 0;
        }
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        size_t keyId = 0;
        for (auto& kp : keyPairs) {
            keyId++; // increment key number

            int outPin = kp.first;
            int inPin  = kp.second;

            // Drive output high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);
            std::this_thread::sleep_for(std::chrono::microseconds(50));

            bool pressed = gpioRead(inPin);

            // Debounce logic
            if (pressed) {
                keyCounters[keyId]++;
                if (keyCounters[keyId] >= DEBOUNCE_THRESHOLD && !keyStates[keyId]) {
                    keyStates[keyId] = true;
                    std::cout << "Key pressed: " << keyId << std::endl;
                }
            } else {
                keyCounters[keyId] = 0;
                if (keyStates[keyId]) {
                    keyStates[keyId] = false;
                    std::cout << "Key released: " << keyId << std::endl;
                }
            }

            // Reset output pin
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            // tiny delay to reduce ghosting
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }

    gpioTerminate();
    return 0;
}

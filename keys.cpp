#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

// List of GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Simple debounce time in milliseconds
const int debounceMs = 5;

// Key map: (outPin, inPin) -> key number
std::map<std::pair<int,int>, int> keyMap;

// Previous stable key states
std::map<int,bool> keyStates;

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Initialize pins as input with pull-down
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    // Build the key map (example: adjust to your keyboard layout)
    for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
        for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
            if (inIdx == outIdx) continue;
            int keyNum = outIdx*4 + inIdx + 1; // your mapping logic
            keyMap[{pins[outIdx], pins[inIdx]}] = keyNum;
            keyStates[keyNum] = false; // initialize all released
        }
    }

    std::cout << "Starting keyboard scan with debounce..." << std::endl;

    while (true) {
        for (auto outPin : pins) {
            // Drive current output high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Scan all other pins
            for (auto inPin : pins) {
                if (inPin == outPin) continue;

                bool pressed = gpioRead(inPin);
                int keyNum = keyMap[{outPin, inPin}];

                // Debounce: only change if state persists
                static std::map<int,int> counters; // counts consecutive reads
                if (pressed) {
                    counters[keyNum]++;
                    if (counters[keyNum] >= debounceMs && !keyStates[keyNum]) {
                        keyStates[keyNum] = true;
                        std::cout << "Key pressed: " << keyNum << std::endl;
                    }
                } else {
                    counters[keyNum] = 0;
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

            // Small delay per pin to reduce ghosting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

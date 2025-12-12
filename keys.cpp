#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

// GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Debounce threshold
const int DEBOUNCE_THRESHOLD = 3;

// Key map: only actual keyboard connections (output pin, input pin) -> key number
std::map<std::pair<int,int>, int> keyMap = {
    // Example: replace with your real keyboard wiring
    {{2,3}, 1},  {{2,5}, 2},  {{2,7}, 3},   // Output 2 + various inputs
    {{3,4}, 4},  {{3,6}, 5},  {{3,8}, 6},   // Output 3 + inputs
    {{4,5}, 7},  {{4,7}, 8},  {{4,9}, 9},
    {{17,10},10},{{17,11},11},{{17,12},12},
    // ... continue until all 36 keys are mapped
};

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

    // Initialize key states and counters
    for (auto const& kv : keyMap) {
        keyStates[kv.second] = false;
        keyCounters[kv.second] = 0;
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        // Scan each output pin
        for (auto outPin : pins) {
            // Set output high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Small delay to stabilize line
            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            // Check only input pins that are mapped with this output
            for (auto const& kv : keyMap) {
                int mappedOut = kv.first.first;
                int mappedIn  = kv.first.second;
                int keyNum    = kv.second;

                if (mappedOut != outPin) continue; // skip keys not using this output

                bool pressed = gpioRead(mappedIn);

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

            // Small delay per pin
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

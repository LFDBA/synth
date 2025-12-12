#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>

// List of GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Map output+input index to key number
int keyMap(int outIdx, int inIdx) {
    return outIdx * 4 + inIdx + 1; // Adjust to match your keyboard
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    size_t numPins = pins.size();

    // Initialize all pins as inputs with pull-down
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    // Key states: flattened 2D matrix [out][in]
    std::vector<bool> keyStates(numPins * numPins, false);

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        // Iterate each pin as output (sender)
        for (size_t outIdx = 0; outIdx < numPins; ++outIdx) {
            int outPin = pins[outIdx];

            // Set current output pin high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Scan other pins as inputs
            for (size_t inIdx = 0; inIdx < numPins; ++inIdx) {
                if (inIdx == outIdx) continue; // skip driving pin itself

                int inPin = pins[inIdx];
                gpioSetMode(inPin, PI_INPUT); // make sure it's input

                int keyNum = keyMap(outIdx, inIdx);
                bool pressed = gpioRead(inPin);

                // Update state only on change
                if (pressed && !keyStates[keyNum]) {
                    keyStates[keyNum] = true;
                    std::cout << "Key pressed: " << keyNum << std::endl;
                } else if (!pressed && keyStates[keyNum]) {
                    keyStates[keyNum] = false;
                    std::cout << "Key released: " << keyNum << std::endl;
                }
            }

            // Reset output pin to input with pull-down before next iteration
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            // Tiny delay to avoid ghosting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

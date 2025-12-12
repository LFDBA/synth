#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

// GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Debounce in consecutive scans
const int debounceScans = 3;

// Key state tracking
struct KeyState {
    int count = 0;    // consecutive scans pressed
    bool pressed = false;
};
std::map<int, KeyState> keyStates;

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Initialize all pins as inputs with pull-down
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

            // Scan all other pins
            for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
                if (inIdx == outIdx) continue; // skip the output pin

                int inPin = pins[inIdx];
                bool isHigh = gpioRead(inPin);

                // Compute unique key number
                int keyNum = outIdx * 4 + inIdx + 1; // adjust +1 if needed

                // Debounce logic
                KeyState &ks = keyStates[keyNum];
                if (isHigh) {
                    ks.count++;
                    if (!ks.pressed && ks.count >= debounceScans) {
                        ks.pressed = true;
                        std::cout << "Key pressed: " << keyNum << std::endl;
                    }
                } else {
                    ks.count = 0;
                    if (ks.pressed) {
                        ks.pressed = false;
                        std::cout << "Key released: " << keyNum << std::endl;
                    }
                }
            }

            // Reset the output pin to input with pull-down
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            // Small delay to avoid ghosting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

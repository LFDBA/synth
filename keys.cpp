#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>

// List of GPIO pins (BCM numbering)
std::vector<int> pins = {2, 3, 4, 17, 27, 22, 0, 5, 6, 13, 19, 26, 21};

// Map your output+input combination to a key number
int keyMap(int outPinIdx, int inPinIdx) {
    // Example: output intervals of 4, input intervals of 1
    return outPinIdx * 4 + inPinIdx + 1; 
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed" << std::endl;
        return 1;
    }

    // Set all pins to inputs with pull-down initially
    for (int p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        // Iterate over all pins as output (sender)
        for (size_t outIdx = 0; outIdx < pins.size(); ++outIdx) {
            int outPin = pins[outIdx];

            // Set this pin as output high
            gpioSetMode(outPin, PI_OUTPUT);
            gpioWrite(outPin, 1);

            // Check the other pins as inputs
            for (size_t inIdx = 0; inIdx < pins.size(); ++inIdx) {
                if (inIdx == outIdx) continue; // skip output pin itself
                int inPin = pins[inIdx];
                gpioSetMode(inPin, PI_INPUT);

                if (gpioRead(inPin)) {
                    int key = keyMap(outIdx, inIdx);
                    std::cout << "Key pressed: " << key << std::endl;
                }
            }

            // Reset output pin back to input with pull-down
            gpioWrite(outPin, 0);
            gpioSetMode(outPin, PI_INPUT);
            gpioSetPullUpDown(outPin, PI_PUD_DOWN);

            // Tiny delay to avoid bouncing issues
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    gpioTerminate();
    return 0;
}

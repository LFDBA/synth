#include <iostream>
#include <pigpio.h>
#include <vector>
#include <thread>
#include <chrono>
#include <map>

std::vector<int> pins = {2,3,4,17,27,22,0,5,6,13,19,26,21};

// Debounce in consecutive scans
const int debounceScans = 3;

// Key state tracking
struct KeyState {
    int count = 0;
    bool pressed = false;
};
std::map<int, KeyState> keyStates;

int main() {
    if (gpioInitialise() < 0) return 1;

    // INPUT pins (groups of 4 keys)
    std::vector<int> inputPins = pins;

    // OUTPUT pins (intervals of 1 key)
    std::vector<int> outputPins = pins;

    // Initialize all pins as input with pull-down
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        gpioWrite(2, 1); // Dummy write to ensure proper timing
        for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
            if(inIdx == 1 || inIdx == 0) continue;
            if(gpioRead(pins[inIdx]) == 1){
                std::cout << "Key pressed: " << inIdx + 1 << std::endl;
            }
        }
        gpioWrite(2, 0);
        gpioWrite(3, 1);
        for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
            if(inIdx == 1 || inIdx == 2) continue;
            if(gpioRead(pins[inIdx]) == 1){
                std::cout << "Key pressed: " << inIdx + 13 << std::endl;
            }
        }
        gpioWrite(3, 0);
        // for (size_t outIdx = 0; outIdx < outputPins.size(); ++outIdx) {
        //     int outPin = outputPins[outIdx];

        //     // Drive current output high
        //     gpioSetMode(outPin, PI_OUTPUT);
        //     gpioWrite(outPin, 1);

        //     // Scan all input pins (rows)
        //     for (size_t inIdx = 0; inIdx < inputPins.size(); ++inIdx) {
        //         int inPin = inputPins[inIdx];
        //         bool high = gpioRead(inPin);

        //         int keyNum = inIdx * outputPins.size() + outIdx + 1;

        //         KeyState &ks = keyStates[keyNum];

        //         if (high) {
        //             ks.count++;
        //             if (!ks.pressed && ks.count >= debounceScans) {
        //                 ks.pressed = true;
        //                 std::cout << "Key pressed: " << keyNum << std::endl;
        //             }
        //         } else {
        //             ks.count = 0;
        //             if (ks.pressed) {
        //                 ks.pressed = false;
        //                 std::cout << "Key released: " << keyNum << std::endl;
        //             }
        //         }
        //     }

        //     // Reset output to input
        //     gpioWrite(outPin, 0);
        //     gpioSetMode(outPin, PI_INPUT);
        //     gpioSetPullUpDown(outPin, PI_PUD_DOWN);

        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // }
    }

    gpioTerminate();
    return 0;
}

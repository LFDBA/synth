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
        for(size_t i = 0; i < pins.size(); ++i)
            gpioWrite(pins[i], 1); // Reset all to low
        
            for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
                if(inIdx == i) continue;
                if(gpioRead(pins[inIdx]) == 1){
                    std::cout << "Key pressed: " << inIdx + 1 << std::endl;
                }
            }
            gpioWrite(pins[i], 0); // Set back to input
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
    }

    gpioTerminate();
    return 0;
}

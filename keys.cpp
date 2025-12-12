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
        // gpioWrite(2, 1); // Dummy write to ensure proper timing
        // for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
        //     if(inIdx == 1 || inIdx == 0) continue;
        //     if(gpioRead(pins[inIdx]) == 1){
        //         std::cout << "Key pressed: " << inIdx + 1 << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::microseconds(50));
        // }
        // gpioWrite(2, 0);
        // gpioWrite(3, 1);
        // for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
        //     if(inIdx == 1 || inIdx == 2) continue;
        //     if(gpioRead(pins[inIdx]) == 1){
        //         std::cout << "Key pressed: " << inIdx + 13 << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::microseconds(50));
        // }
        // gpioWrite(3, 0);
        // gpioWrite(4, 1);
        // for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
        //     if(inIdx == 2 || inIdx == 3) continue;
        //     if(gpioRead(pins[inIdx]) == 1){
        //         std::cout << "Key pressed: " << inIdx + 25 << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::microseconds(50));
        // }
        // gpioWrite(4, 0);
        // gpioWrite(17, 1);
        // for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
        //     if(inIdx == 3 || inIdx == 4) continue;
        //     if(gpioRead(pins[inIdx]) == 1){
        //         std::cout << "Key pressed: " << inIdx + 37 << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::microseconds(50));
        // }
        // gpioWrite(17, 0);
        // gpioWrite(27, 1);
        // for(size_t inIdx = 0; inIdx < pins.size(); ++inIdx){
        //     if(inIdx == 4 || inIdx == 5) continue;
        //     if(gpioRead(pins[inIdx]) == 1){
        //         std::cout << "Key pressed: " << inIdx + 49 << std::endl;
        //     }
        //     std::this_thread::sleep_for(std::chrono::microseconds(50));
        // }
        // gpioWrite(27, 0);
        
        
        for(size_t i = 0; i < 5; ++i){
            gpioWrite(pins[i], 1);
            for(size_t j = 0; j < pins.size(); ++j){
                if(j == i || j == i - 1) continue;
                if(gpioRead(pins[j]) == 1){
                    std::cout << "Key pressed: " << j + (i * 12) + 1 << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
            gpioWrite(pins[i], 0);
        }
        
        
    }

    gpioTerminate();
    return 0;
}

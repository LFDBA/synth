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

int mapKeyNumber(int k) {
    // k is 1–36
    int idx = k - 1;     // convert to 0-based
    int col = idx % 6;   // which “group”
    int row = idx / 6;   // which position within the group
    return 6 + col + row * 12;
}


int main() {
    if (gpioInitialise() < 0) return 1;

    // Initialize all pins as input with pull-down
    for (auto p : pins) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_DOWN);
    }

    std::cout << "Starting keyboard scan..." << std::endl;

    while (true) {
        for(size_t i = 0; i < 5; ++i){  // drive outputs (your exact original range)
            gpioWrite(pins[i], 1);

            for(size_t j = 0; j < pins.size(); ++j){
                if(j == i || j == i - 1) continue;

                int keyNum = j + 1 + (i * 12);  // keep your original numbering
                bool isHigh = gpioRead(pins[j]) == 1;

                // Debounce + press/release tracking
                if(isHigh){
                    keyStates[keyNum].count++;
                    if(keyStates[keyNum].count >= debounceScans && !keyStates[keyNum].pressed){
                        keyStates[keyNum].pressed = true;
                        std::cout << "Key pressed: " << mapKeyNumber(keyNum) << std::endl;
                    }
                } else {
                    keyStates[keyNum].count = 0;
                    if(keyStates[keyNum].pressed){
                        keyStates[keyNum].pressed = false;
                        std::cout << "Key released: " << mapKeyNumber(keyNum) << std::endl;
                    }
                }

                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            gpioWrite(pins[i], 0);
        }
    }

    gpioTerminate();
    return 0;
}

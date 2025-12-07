#include <iostream>
#include <pigpio.h>
#include <unistd.h>

int main() {
    // Connect to existing pigpio daemon
    if (!pigpio_start(NULL, NULL)) {  // pigpio daemon must be running
        std::cerr << "Failed to connect to pigpio daemon\n";
        return 1;
    }

    int pin = 26;
    set_mode(pin, PI_INPUT);

    while (true) {
        int level = gpio_read(pin);
        std::cout << (level ? "HIGH" : "LOW") << std::endl;
        usleep(100000); // 100ms
    }

    pigpio_stop(); // disconnect
    return 0;
}

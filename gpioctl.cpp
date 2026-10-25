#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>

#include <pigpio.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;

void handle_signal(int) {
    stop_requested = 1;
}

bool parse_integer(const char* text, int& value) {
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < 0 || parsed > 53) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}
}

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: gpioctl set <pin> <0|1> | gpioctl blink <pin>\n";
        return 2;
    }

    int pin = 0;
    if (!parse_integer(argv[2], pin)) {
        std::cerr << "Invalid GPIO pin: " << argv[2] << '\n';
        return 2;
    }

    const std::string command = argv[1];
    int value = 0;
    const bool blink = command == "blink";
    if (command == "set") {
        if (argc != 4 || (std::string(argv[3]) != "0" && std::string(argv[3]) != "1")) {
            std::cerr << "Usage: gpioctl set <pin> <0|1>\n";
            return 2;
        }
        value = argv[3][0] - '0';
    } else if (blink) {
        if (argc != 3) {
            std::cerr << "Usage: gpioctl blink <pin>\n";
            return 2;
        }
    } else {
        std::cerr << "Unknown command: " << command << '\n';
        return 2;
    }

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    int status = gpioSetMode(static_cast<unsigned>(pin), PI_OUTPUT);
    if (status == 0 && blink) {
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
        while (!stop_requested) {
            gpioWrite(static_cast<unsigned>(pin), 1);
            gpioDelay(500000);
            if (stop_requested) {
                break;
            }
            gpioWrite(static_cast<unsigned>(pin), 0);
            gpioDelay(500000);
        }
        status = gpioWrite(static_cast<unsigned>(pin), 0);
    } else if (status == 0) {
        status = gpioWrite(static_cast<unsigned>(pin), value);
    }

    gpioTerminate();
    if (status != 0) {
        std::cerr << "Failed to set GPIO " << pin << '\n';
        return 1;
    }
    return 0;
}
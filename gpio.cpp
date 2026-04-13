/*
 * gpio_resistance_check.cpp
 *
 * Detects low resistance between any two GPIO pins on a Raspberry Pi.
 *
 * Method:
 *   For every pin pair (A, B):
 *     1. Drive pin A HIGH (output)
 *     2. Set pin B as INPUT with internal pull-DOWN
 *     3. If B reads HIGH, current is flowing → low resistance detected
 *   Restore all pins to safe inputs between each test.
 *
 * Compile:
 *   g++ -o gpio_resistance_check gpio_resistance_check.cpp -lpigpio -lrt -lpthread
 *
 * Run (requires root for pigpio):
 *   sudo ./gpio_resistance_check
 *
 * Optional flags:
 *   --once        Run a single scan and exit (default: continuous loop)
 *   --interval N  Milliseconds between full scans in loop mode (default: 500)
 */

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <pigpio.h>

// ── Configurable GPIO pin list ────────────────────────────────────────────────
// BCM numbers. Remove any pins you want to exclude (e.g. I2C, UART, SPI).
static const std::vector<int> PINS = {
    2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25,
    26, 27
};

// ── Resistance threshold ──────────────────────────────────────────────────────
// How many consecutive HIGH reads on the sense pin count as "low resistance".
// Increase to reduce false positives on long/noisy wire runs.
static const int CONFIRM_READS = 3;
static const int CONFIRM_DELAY_US = 500; // µs between confirmation reads

// ── Helpers ───────────────────────────────────────────────────────────────────

// Return all pins to safe high-impedance inputs with no pull.
static void releaseAll() {
    for (int p : PINS) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_OFF);
    }
}

// Drive `driver` HIGH; sense `sensor` with pull-down.
// Returns true if sensor reads HIGH (low resistance path detected).
static bool testPair(int driver, int sensor) {
    // Set driver as output HIGH
    gpioSetMode(driver, PI_OUTPUT);
    gpioWrite(driver, 1);

    // Set sensor as input with pull-down
    gpioSetMode(sensor, PI_INPUT);
    gpioSetPullUpDown(sensor, PI_PUD_DOWN);

    // Small settle time
    gpioDelay(200); // µs

    // Confirm with multiple reads to reject transient spikes
    int highCount = 0;
    for (int i = 0; i < CONFIRM_READS; ++i) {
        if (gpioRead(sensor) == 1) ++highCount;
        gpioDelay(CONFIRM_DELAY_US);
    }

    // Restore immediately before returning
    gpioWrite(driver, 0);
    gpioSetMode(driver, PI_INPUT);
    gpioSetPullUpDown(driver, PI_PUD_OFF);
    gpioSetPullUpDown(sensor, PI_PUD_OFF);

    return (highCount == CONFIRM_READS); // all reads must agree
}

// ── Full scan ─────────────────────────────────────────────────────────────────

struct PinPair { int a, b; };

static std::vector<PinPair> fullScan() {
    std::vector<PinPair> found;
    const size_t n = PINS.size();

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            int a = PINS[i], b = PINS[j];

            // Test A→B
            bool ab = testPair(a, b);
            // Test B→A  (catches asymmetric leakage / diode-like paths)
            bool ba = testPair(b, a);

            if (ab || ba) {
                found.push_back({a, b});
            }
        }
    }
    return found;
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    bool runOnce    = false;
    int  intervalMs = 500;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--once") {
            runOnce = true;
        } else if (arg == "--interval" && i + 1 < argc) {
            intervalMs = std::stoi(argv[++i]);
        }
    }

    if (gpioInitialise() < 0) {
        std::cerr << "[ERROR] pigpio initialisation failed. Are you running as root?\n";
        return 1;
    }

    releaseAll();

    std::cout << "GPIO Resistance Checker\n"
              << "Monitoring " << PINS.size() << " pins ("
              << (PINS.size() * (PINS.size() - 1) / 2) << " pairs)\n"
              << (runOnce ? "Mode: single scan\n" : "Mode: continuous (Ctrl-C to stop)\n")
              << std::string(50, '-') << "\n";

    auto runScan = [&]() {
        auto t0 = std::chrono::steady_clock::now();
        auto pairs = fullScan();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count();

        // Timestamp
        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char ts[32];
        std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&now));

        if (pairs.empty()) {
            std::cout << "[" << ts << "] No low-resistance pairs detected"
                      << "  (" << ms << " ms scan)\n";
        } else {
            std::cout << "[" << ts << "] *** " << pairs.size()
                      << " LOW-RESISTANCE PAIR(S) FOUND ***  (" << ms << " ms scan)\n";
            for (auto& p : pairs) {
                std::cout << "    GPIO " << p.a << " <──> GPIO " << p.b << "\n";
            }
        }
        std::cout.flush();
    };

    if (runOnce) {
        runScan();
    } else {
        while (true) {
            runScan();
            std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
        }
    }

    releaseAll();
    gpioTerminate();
    return 0;
}
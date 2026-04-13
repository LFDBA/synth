/*
 * gpio_resistance_check.cpp  –  High-sensitivity edition
 *
 * Uses an RC-timing method rather than a simple digital HIGH/LOW read.
 *
 * -- How it works ------------------------------------------------------------
 *
 *  Basic digital read (old version):
 *    Drive A HIGH → read B with pull-down.
 *    Only detects R < ~10 kΩ (voltage divider with 50 kΩ pull-down).
 *
 *  RC timing method (this version):
 *    1. Pre-charge sense pin B to 3.3 V (OUTPUT HIGH briefly).
 *    2. Drive A HIGH (external resistance holds B up).
 *    3. Set B as INPUT with pull-DOWN (starts discharging).
 *    4. Measure microseconds until B reads LOW.
 *
 *    Without external R: B decays quickly through pull-down alone (~50 kΩ).
 *    With R between A and B: A holds B up → decay is SLOWER.
 *    Higher R → slower decay → detectable up to 500 kΩ – 2 MΩ.
 *
 *    V(t) = Vss + (3.3 − Vss)·exp(−t/τ)
 *      Vss  = 3.3 × Rpull / (Rext + Rpull)
 *      τ    = C_stray × (Rext ∥ Rpull)
 *    Crossing time increases monotonically with Rext.
 *
 * -- Calibration -------------------------------------------------------------
 *    At startup, CALIBRATION_RUNS decay measurements are taken per pair
 *    with no driver active (floating baseline).
 *    Detection = mean + sensitivity × stddev.
 *    Higher --sensitivity → detects higher R, more potential false positives.
 *
 * -- Sensitivity guide -------------------------------------------------------
 *    --sensitivity 2   conservative  ~<  50 kΩ
 *    --sensitivity 5   default       ~< 200 kΩ
 *    --sensitivity 10  high          ~< 500 kΩ
 *    --sensitivity 20  very high     ~< 1–2 MΩ  (needs stable, short wiring)
 *
 * -- Compile ------------------------------------------------------------------
 *    g++ -O2 -o gpio_resistance_check gpio_resistance_check.cpp \
 *        -lpigpio -lrt -lpthread
 *
 * -- Run (pigpio needs root) ---------------------------------------------------
 *    sudo ./gpio_resistance_check
 *    sudo ./gpio_resistance_check --sensitivity 10
 *    sudo ./gpio_resistance_check --once --sensitivity 5
 *    sudo ./gpio_resistance_check --interval 1000 --sensitivity 3
 *    sudo ./gpio_resistance_check --verbose          (print every pair's σ)
 */

#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <pigpio.h>

// -- Driver pin ----------------------------------------------------------------
// Only this pin ever drives HIGH. All other pins are sense-only.
static const int DRIVER_PIN = 6;

// -- Sense pin list (BCM numbers, must NOT include DRIVER_PIN) -----------------
// Remove pins you want to protect (UART=14/15, I2C=2/3, SPI=7-11, etc.)
static const std::vector<int> PINS = {
    2, 3, 4, 5,     7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25,
    26, 27
};

// -- Tuning --------------------------------------------------------------------
static const int      CALIBRATION_RUNS = 30;    // samples per pair at startup
static const int      MEASURE_RUNS     = 10;    // samples per pair each scan
static const uint32_t TIMEOUT_US       = 5000;  // µs cap (= effectively open circuit)
static const int      PRECHARGE_US     = 10;    // µs to hold pre-charge

// -- Helpers -------------------------------------------------------------------

static void releaseAll() {
    gpioSetMode(DRIVER_PIN, PI_INPUT);
    gpioSetPullUpDown(DRIVER_PIN, PI_PUD_OFF);
    for (int p : PINS) {
        gpioSetMode(p, PI_INPUT);
        gpioSetPullUpDown(p, PI_PUD_OFF);
    }
}

// Pre-charge sense, drive driver HIGH, time the decay to LOW.
// Returns µs until sense goes LOW, capped at TIMEOUT_US.
static uint32_t measureDecay(int driver, int sense) {
    // Pre-charge sense pin to 3.3 V
    gpioSetMode(sense, PI_OUTPUT);
    gpioWrite(sense, 1);
    gpioDelay(PRECHARGE_US);

    // Drive driver HIGH (fights against pull-down via external R, if present)
    gpioSetMode(driver, PI_OUTPUT);
    gpioWrite(driver, 1);

    // Switch sense to input + pull-down → begins RC decay
    gpioSetMode(sense, PI_INPUT);
    gpioSetPullUpDown(sense, PI_PUD_DOWN);

    uint32_t t0      = gpioTick();
    uint32_t elapsed = 0;
    while (gpioRead(sense) == 1) {
        elapsed = gpioTick() - t0;
        if (elapsed >= TIMEOUT_US) { elapsed = TIMEOUT_US; break; }
    }
    if (elapsed == 0) elapsed = gpioTick() - t0;

    // Restore both pins
    gpioWrite(driver, 0);
    gpioSetMode(driver,  PI_INPUT);
    gpioSetPullUpDown(driver, PI_PUD_OFF);
    gpioSetPullUpDown(sense,  PI_PUD_OFF);

    return elapsed;
}

// Collect `runs` measurements → compute mean and stddev
static void sampleDecay(int driver, int sense, int runs,
                        double& mean, double& stddev) {
    double sum = 0, sumSq = 0;
    for (int i = 0; i < runs; ++i) {
        double t = static_cast<double>(measureDecay(driver, sense));
        sum   += t;
        sumSq += t * t;
        gpioDelay(200);
    }
    mean   = sum / runs;
    stddev = std::sqrt(sumSq / runs - mean * mean);
    if (stddev < 0.5) stddev = 0.5; // floor prevents divide-by-zero
}

// -- Pair data -----------------------------------------------------------------

struct PinPair {
    int    sense;           // the sense pin (driver is always DRIVER_PIN)
    double base_mean, base_sd;
};

// -- Main ----------------------------------------------------------------------

int main(int argc, char* argv[]) {
    bool   runOnce    = false;
    bool   verbose    = false;
    int    intervalMs = 500;
    double sensitivity = 5.0;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if      (arg == "--once")                        runOnce    = true;
        else if (arg == "--verbose")                     verbose    = true;
        else if (arg == "--interval"    && i+1 < argc)  intervalMs  = std::stoi(argv[++i]);
        else if (arg == "--sensitivity" && i+1 < argc)  sensitivity = std::stod(argv[++i]);
    }

    if (gpioInitialise() < 0) {
        std::cerr << "[ERROR] pigpio init failed – run as root?\n";
        return 1;
    }
    releaseAll();

    const size_t n = PINS.size();

    std::cout << "GPIO High-Sensitivity Resistance Checker\n"
              << "Driver: GPIO " << DRIVER_PIN
              << "  Sense pins: " << n
              << "  Sensitivity: " << sensitivity << "×σ\n"
              << std::string(56, '-') << "\n";

    // -- Calibration ----------------------------------------------------------
    std::cout << "Calibrating (" << CALIBRATION_RUNS << " samples/pin) …\n";

    std::vector<PinPair> pairs;
    pairs.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        PinPair p;
        p.sense = PINS[i];

        // Calibrate with driver floating (INPUT, no pull) so we capture
        // the natural decay of each sense pin in isolation.
        gpioSetMode(DRIVER_PIN, PI_INPUT);
        gpioSetPullUpDown(DRIVER_PIN, PI_PUD_OFF);
        sampleDecay(DRIVER_PIN, p.sense, CALIBRATION_RUNS, p.base_mean, p.base_sd);

        pairs.push_back(p);

        if ((i + 1) % 5 == 0 || i + 1 == n)
            std::cout << "\r  " << (i + 1) << "/" << n << std::flush;
    }
    std::cout << "\nCalibration complete.\n"
              << std::string(56, '-') << "\n";

    // -- Scan -----------------------------------------------------------------
    auto runScan = [&]() {
        auto t0 = std::chrono::steady_clock::now();
        std::vector<std::pair<const PinPair*, double>> found;

        for (const auto& p : pairs) {
            double m, s;
            sampleDecay(DRIVER_PIN, p.sense, MEASURE_RUNS, m, s);

            double sigma = (m - p.base_mean) / p.base_sd;

            if (verbose) {
                std::cout << "  GPIO " << DRIVER_PIN
                          << " → GPIO" << std::setw(2) << p.sense
                          << "  " << std::fixed << std::setprecision(1)
                          << m << "µs (+" << sigma << "σ)\n";
            }

            if (sigma >= sensitivity)
                found.push_back({&p, sigma});
        }

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::steady_clock::now() - t0).count();

        auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        char ts[32];
        std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&now));

        if (found.empty()) {
            std::cout << "[" << ts << "] No pins detected  (" << ms << " ms)\n";
        } else {
            std::cout << "[" << ts << "] *** " << found.size()
                      << " PIN(S) DETECTED ***  (" << ms << " ms)\n";
            for (auto& [pp, sig] : found) {
                std::cout << "    GPIO " << DRIVER_PIN << " <--> GPIO " << pp->sense
                          << "   +" << std::fixed << std::setprecision(1)
                          << sig << "σ above baseline\n";
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
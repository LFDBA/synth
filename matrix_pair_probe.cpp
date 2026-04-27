/*
 * matrix_pair_probe.cpp
 *
 * Live GPIO pair tester for the current synth keyboard wiring.
 *
 * What it does:
 * - Scans every configured matrix pin against every other configured matrix pin.
 * - Reports when a pair becomes electrically connected.
 * - Labels row/column connections with the matrix keyID and note name.
 * - Labels row/row or col/col hits as suspicious shorts.
 *
 * Compile:
 *   g++ -O2 -std=c++17 matrix_pair_probe.cpp -o matrix_pair_probe \
 *       -lpigpio -lrt -lpthread
 *
 * Run:
 *   sudo ./matrix_pair_probe
 *   sudo ./matrix_pair_probe --once
 *   sudo ./matrix_pair_probe --interval 100
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <pigpio.h>

static const std::vector<int> DEFAULT_SCAN_PINS = {
    4, 5, 6, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27
};
static const std::vector<int> ROW_PINS = {4, 5, 6, 12};
static const std::vector<int> COL_PINS = {13, 17, 14, 19, 20, 23, 22};
static const std::array<const char*, 12> NOTE_NAMES = {
    "E", "F", "F#", "G", "G#", "A", "Bb", "B", "C", "C#", "D", "D#"
};

static const unsigned int DRIVE_SETTLE_US = 80;
static std::vector<int> scanPins = DEFAULT_SCAN_PINS;

struct PairKey {
    int a;
    int b;

    bool operator<(const PairKey& other) const {
        if (a != other.a) return a < other.a;
        return b < other.b;
    }
};

struct MatrixInfo {
    bool isMatrix = false;
    bool isRowRowShort = false;
    bool isColColShort = false;
    int rowPin = -1;
    int colPin = -1;
    int keyId = -1;
    std::string note;
};

static int findIndex(const std::vector<int>& pins, int pin) {
    auto it = std::find(pins.begin(), pins.end(), pin);
    if (it == pins.end()) return -1;
    return static_cast<int>(it - pins.begin());
}

static PairKey makePair(int x, int y) {
    if (x > y) std::swap(x, y);
    return {x, y};
}

static void releaseAllPins() {
    for (int pin : scanPins) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_OFF);
    }
}

static MatrixInfo getMatrixInfo(const PairKey& pair) {
    MatrixInfo info;

    int rowA = findIndex(ROW_PINS, pair.a);
    int rowB = findIndex(ROW_PINS, pair.b);
    int colA = findIndex(COL_PINS, pair.a);
    int colB = findIndex(COL_PINS, pair.b);

    if (rowA >= 0 && colB >= 0) {
        info.isMatrix = true;
        info.rowPin = pair.a;
        info.colPin = pair.b;
        info.keyId = rowA * static_cast<int>(COL_PINS.size()) + colB;
        info.note = NOTE_NAMES[info.keyId % NOTE_NAMES.size()];
        return info;
    }

    if (rowB >= 0 && colA >= 0) {
        info.isMatrix = true;
        info.rowPin = pair.b;
        info.colPin = pair.a;
        info.keyId = rowB * static_cast<int>(COL_PINS.size()) + colA;
        info.note = NOTE_NAMES[info.keyId % NOTE_NAMES.size()];
        return info;
    }

    if (rowA >= 0 && rowB >= 0) info.isRowRowShort = true;
    if (colA >= 0 && colB >= 0) info.isColColShort = true;
    return info;
}

static std::string describePair(const PairKey& pair) {
    MatrixInfo info = getMatrixInfo(pair);
    std::string label = "GPIO" + std::to_string(pair.a) + " <-> GPIO" + std::to_string(pair.b);

    if (info.isMatrix) {
        label += " | matrix keyID " + std::to_string(info.keyId);
        label += " | note " + info.note;
        label += " | row GPIO" + std::to_string(info.rowPin);
        label += " | col GPIO" + std::to_string(info.colPin);
        return label;
    }

    if (info.isRowRowShort) return label + " | suspicious row-row short";
    if (info.isColColShort) return label + " | suspicious col-col short";
    return label + " | non-matrix pair";
}

static std::set<PairKey> scanConnectedPairs() {
    std::set<PairKey> detected;

    releaseAllPins();

    for (int driver : scanPins) {
        for (int pin : scanPins) {
            if (pin == driver) continue;
            gpioSetMode(pin, PI_INPUT);
            gpioSetPullUpDown(pin, PI_PUD_DOWN);
        }

        gpioSetMode(driver, PI_OUTPUT);
        gpioWrite(driver, 1);
        gpioDelay(DRIVE_SETTLE_US);

        for (int sense : scanPins) {
            if (sense == driver) continue;
            if (gpioRead(sense) == 1) detected.insert(makePair(driver, sense));
        }

        gpioWrite(driver, 0);
        gpioSetMode(driver, PI_INPUT);
        gpioSetPullUpDown(driver, PI_PUD_OFF);
    }

    releaseAllPins();
    return detected;
}

static std::vector<int> parsePinList(const std::string& csv) {
    std::vector<int> pins;
    std::string token;

    for (char ch : csv) {
        if (ch == ',') {
            if (!token.empty()) {
                pins.push_back(std::stoi(token));
                token.clear();
            }
        } else if (!std::isspace(static_cast<unsigned char>(ch))) {
            token.push_back(ch);
        }
    }

    if (!token.empty()) pins.push_back(std::stoi(token));

    std::sort(pins.begin(), pins.end());
    pins.erase(std::unique(pins.begin(), pins.end()), pins.end());
    return pins;
}

int main(int argc, char* argv[]) {
    bool runOnce = false;
    int intervalMs = 150;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--once") {
            runOnce = true;
        } else if (arg == "--interval" && i + 1 < argc) {
            intervalMs = std::max(10, std::stoi(argv[++i]));
        } else if (arg == "--pins" && i + 1 < argc) {
            scanPins = parsePinList(argv[++i]);
        }
    }

    if (scanPins.size() < 2) {
        std::cerr << "Need at least two pins to scan.\n";
        return 1;
    }

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed. Run with sudo.\n";
        return 1;
    }

    releaseAllPins();

    std::cout << "Matrix pair probe\n";
    std::cout << "Watching pins:";
    for (int pin : scanPins) std::cout << " " << pin;
    std::cout << "\n";
    std::cout << "Touch or press one connection at a time.\n";
    std::cout << "New hits print as ACTIVE, releases print as CLEAR.\n";

    std::set<PairKey> previous;

    while (true) {
        std::set<PairKey> current = scanConnectedPairs();

        for (const PairKey& pair : current) {
            if (previous.count(pair) == 0) {
                std::cout << "ACTIVE  " << describePair(pair) << "\n";
            }
        }

        for (const PairKey& pair : previous) {
            if (current.count(pair) == 0) {
                std::cout << "CLEAR   " << describePair(pair) << "\n";
            }
        }

        std::cout.flush();

        if (runOnce) break;

        previous = std::move(current);
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    releaseAllPins();
    gpioTerminate();
    return 0;
}

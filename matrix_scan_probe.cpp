/*
 * matrix_scan_probe.cpp
 *
 * Matrix-faithful keyboard probe.
 *
 * Unlike the all-pairs probe, this mirrors the real synth scan:
 * - drives one row HIGH at a time
 * - reads a chosen ordered list of column pins
 * - reports which logical keyID / note each row+column hit maps to
 *
 * This is the tool to use when you suspect a moved or swapped column wire.
 *
 * Compile:
 *   g++ -O2 -std=c++17 matrix_scan_probe.cpp -o matrix_scan_probe \
 *       -lpigpio -lrt -lpthread
 *
 * Run:
 *   sudo ./matrix_scan_probe
 *   sudo ./matrix_scan_probe --interval 80
 *   sudo ./matrix_scan_probe --cols 13,17,18,19,20,23,22
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <iostream>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <pigpio.h>

static const std::vector<int> DEFAULT_ROWS = {4, 5, 6, 12};
static const std::vector<int> DEFAULT_COLS = {13, 17, 18, 19, 20, 23, 22};
static const std::array<const char*, 12> NOTE_NAMES = {
    "E", "F", "F#", "G", "G#", "A", "Bb", "B", "C", "C#", "D", "D#"
};
static const unsigned int SETTLE_US = 80;

struct Hit {
    int rowPin = -1;
    int colPin = -1;
    int rowIndex = -1;
    int colIndex = -1;
    int keyId = -1;
    std::string note;

    bool operator<(const Hit& other) const {
        if (rowPin != other.rowPin) return rowPin < other.rowPin;
        return colPin < other.colPin;
    }
};

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
    return pins;
}

static std::vector<int> buildUnionPins(const std::vector<int>& rows, const std::vector<int>& cols) {
    std::vector<int> pins = rows;
    pins.insert(pins.end(), cols.begin(), cols.end());
    std::sort(pins.begin(), pins.end());
    pins.erase(std::unique(pins.begin(), pins.end()), pins.end());
    return pins;
}

static void releasePins(const std::vector<int>& pins) {
    for (int pin : pins) {
        gpioSetMode(pin, PI_INPUT);
        gpioSetPullUpDown(pin, PI_PUD_OFF);
    }
}

static std::set<Hit> scanMatrix(const std::vector<int>& rows, const std::vector<int>& cols) {
    std::vector<int> allPins = buildUnionPins(rows, cols);
    std::set<Hit> hits;

    releasePins(allPins);

    for (int colPin : cols) {
        gpioSetMode(colPin, PI_INPUT);
        gpioSetPullUpDown(colPin, PI_PUD_DOWN);
    }

    for (int rowIndex = 0; rowIndex < static_cast<int>(rows.size()); ++rowIndex) {
        int rowPin = rows[rowIndex];

        gpioSetMode(rowPin, PI_OUTPUT);
        gpioWrite(rowPin, 1);
        gpioDelay(SETTLE_US);

        for (int colIndex = 0; colIndex < static_cast<int>(cols.size()); ++colIndex) {
            int colPin = cols[colIndex];
            if (gpioRead(colPin) == 1) {
                Hit hit;
                hit.rowPin = rowPin;
                hit.colPin = colPin;
                hit.rowIndex = rowIndex;
                hit.colIndex = colIndex;
                hit.keyId = rowIndex * static_cast<int>(cols.size()) + colIndex;
                hit.note = NOTE_NAMES[hit.keyId % NOTE_NAMES.size()];
                hits.insert(hit);
            }
        }

        gpioWrite(rowPin, 0);
        gpioSetMode(rowPin, PI_INPUT);
        gpioSetPullUpDown(rowPin, PI_PUD_DOWN);
    }

    releasePins(allPins);
    return hits;
}

static std::string describeHit(const Hit& hit) {
    return "row GPIO" + std::to_string(hit.rowPin) +
           " <-> col GPIO" + std::to_string(hit.colPin) +
           " | rowIdx " + std::to_string(hit.rowIndex) +
           " | colIdx " + std::to_string(hit.colIndex) +
           " | keyID " + std::to_string(hit.keyId) +
           " | note " + hit.note;
}

int main(int argc, char* argv[]) {
    std::vector<int> rows = DEFAULT_ROWS;
    std::vector<int> cols = DEFAULT_COLS;
    int intervalMs = 120;
    bool runOnce = false;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--rows" && i + 1 < argc) {
            rows = parsePinList(argv[++i]);
        } else if (arg == "--cols" && i + 1 < argc) {
            cols = parsePinList(argv[++i]);
        } else if (arg == "--interval" && i + 1 < argc) {
            intervalMs = std::max(10, std::stoi(argv[++i]));
        } else if (arg == "--once") {
            runOnce = true;
        }
    }

    if (rows.empty() || cols.empty()) {
        std::cerr << "Need at least one row and one column pin.\n";
        return 1;
    }

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed. Run with sudo.\n";
        return 1;
    }

    releasePins(buildUnionPins(rows, cols));

    std::cout << "Matrix scan probe\nRows:";
    for (int pin : rows) std::cout << " " << pin;
    std::cout << "\nCols:";
    for (int pin : cols) std::cout << " " << pin;
    std::cout << "\nPress one key at a time.\n";

    std::set<Hit> previous;

    while (true) {
        std::set<Hit> current = scanMatrix(rows, cols);

        for (const Hit& hit : current) {
            if (previous.count(hit) == 0) {
                std::cout << "ACTIVE  " << describeHit(hit) << "\n";
            }
        }

        for (const Hit& hit : previous) {
            if (current.count(hit) == 0) {
                std::cout << "CLEAR   " << describeHit(hit) << "\n";
            }
        }

        if (current.size() > 1) {
            std::cout << "INFO    multiple columns/keys active in one scan:";
            for (const Hit& hit : current) {
                std::cout << " [" << hit.note << "@" << hit.colPin << "]";
            }
            std::cout << "\n";
        }

        std::cout.flush();

        if (runOnce) break;

        previous = std::move(current);
        std::this_thread::sleep_for(std::chrono::milliseconds(intervalMs));
    }

    releasePins(buildUnionPins(rows, cols));
    gpioTerminate();
    return 0;
}

#include <iostream>
#include <cmath>
#include <array>
#include <algorithm>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <linux/input.h>
#include <rtaudio/RtAudio.h>
#include "Reverb.h"
#include "font5x7.h"
#include <ncurses.h>  
#include <pigpio.h>
#include <cstdint>
#include <cstring>
#include <signal.h>
#include <atomic> 
#include <thread>
#include <chrono>
#include <map>
#include <cstdlib> 
#include <ctime>
#include <fstream>
#include <cerrno>
using namespace std::chrono;
#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"
#include <vector>
#include <cstdint>
#include <iostream>


std::vector<int> pins = {4, 5, 6, 12, 13, 17, 14, 19, 20, 22, 23};
std::vector<int> rowPins = {4, 5, 6, 12}; 
std::vector<int> colPins = {13, 17, 14, 19, 20, 23, 22};

void initMatrix() {
    for (int r : rowPins) {
        gpioSetMode(r, PI_OUTPUT);
        gpioWrite(r, 0);
        std::cout << "ROW OUTPUT: " << r << "\n";
    }
    for (int c : colPins) {
        gpioSetMode(c, PI_INPUT);
        gpioSetPullUpDown(c, PI_PUD_DOWN);
        std::cout << "COL INPUT: " << c << " idle=" << gpioRead(c) << "\n";
    }
}

unsigned long lastClickTime = 0;
const unsigned long doubleClickDelay = 400; // ms
bool singleClickPending = false;
unsigned long buttonPressStartTime = 0;
bool writeHoldTriggered = false;
constexpr unsigned long WRITE_HOLD_DELAY_MS = 300;
int writePendingClickCount = 0;
float fatness;
float sampleRate = 48000.0f;
float noiseVolume = 0.5f;
float noiseFilterCutoff = 20000.0f;
float noiseAdsrAmount = 1.0f;
float maxOutputLevel = 0.5f;
float outputLevel = maxOutputLevel;
float clipAmount = 0.0f;
constexpr float MIN_CLIP_DRIVE = 1.0f;
constexpr float MAX_CLIP_DRIVE = 10.0f;
constexpr float LOW_POLY_VOICE_HZ = 150.0f;
constexpr float NOISE_RENDER_GAIN = 10.0f;
float lowPolyLowpassState = 0.0f;

// Debounce in consecutive scans
const int debounceScans = 8;


int selectedPreset = 0;
int presetListSelection = 0;
int presetOptionSelection = 0;
int presetListSelectionOffset = 0;
int lastPresetListKnobSelection = -1;
int presetReorderAccumulator = 0;
int maxTurnVal = 80;
constexpr int MAX_PRESET_NAME_LEN = 12;
constexpr int PRESET_REORDER_KNOB_STEP = 40;
constexpr int MAX_WRITE_NOTES = 48;
constexpr float WRITE_MIN_BPM = 40.0f;
constexpr float WRITE_MAX_BPM = 240.0f;
constexpr float WRITE_NOTE_GATE = 0.8f;
constexpr float WRITE_FILTER_CENTER_WIDTH = 0.04f;
std::string presetNameInput;
const char* PRESET_FILE_PATH = "../presets.dat";
std::vector<int> writeNotes;
float writePlaybackVolume = 1.0f;
float writeTempoBpm = 120.0f;
float writeFilterControl = 0.5f;
bool writePlaybackActive = false;
bool writePlaybackLooping = false;
size_t writePlaybackIndex = 0;
unsigned long writePlaybackStepStartMs = 0;
int writePlaybackVoiceIndex = -1;
float writeFilterLowpassState = 0.0f;
float writeFilterHighpassState = 0.0f;

// Global audio objects
RtAudio dac;
RtAudio::StreamParameters oParams;
unsigned int currentDeviceId;
std::atomic<bool> deviceSwitching(false);

enum NoiseType {
    NOISE_NONE,
    WHITE_NOISE,
    BLACK_NOISE,
    BROWN_NOISE,
    PINK_NOISE,
    RED_NOISE,
};
NoiseType noiseType = NOISE_NONE;


#include <cmath>
#include <cstdlib>



class NoiseGenerator {
public:
    NoiseGenerator(NoiseType type = NOISE_NONE) 
        : type(type), lastBrown(0.0f), pinkStore{0}, filteredSample(0.0f), cutoffHz(20000.0f), lowPassAlpha(1.0f) {
        updateLowPassAlpha();
    }

    float next() {
        float sample = 0.0f;

        switch(type) {
            case WHITE_NOISE:
                sample = randFloat(-1.0f, 1.0f) * (noiseVolume / 12);
                break;

            case PINK_NOISE:
                sample = nextPink() * (noiseVolume / 60);
                break;

            case BROWN_NOISE:
                sample = nextBrown() * (noiseVolume / 40);
                break;

            case RED_NOISE:
                sample = nextBrown() * (noiseVolume / 40); // treat same as brown for simplicity
                break;

            case BLACK_NOISE:
                sample = nextBlack() * (noiseVolume / 3.0f);
                break;

            case NOISE_NONE:
            default:
                sample = 0.0f;
                break;
        }

        return applyLowPass(sample);
    }

    void setType(NoiseType t) { type = t; }
    void setCutoff(float cutoff) {
        cutoffHz = std::clamp(cutoff, 20.0f, sampleRate * 0.45f);
        updateLowPassAlpha();
    }

private:
    NoiseType type = NOISE_NONE;
    float lastBrown;
    float pinkStore[7]; // simple pink filter
    float filteredSample;
    float cutoffHz;
    float lowPassAlpha;

    // helper: uniform float -1..1
    float randFloat(float min, float max) {
        return min + (float)rand() / RAND_MAX * (max - min);
    }

    // brown noise: integrate white
    float nextBrown() {
        float white = randFloat(-0.05f, 0.05f); // small step
        lastBrown += white;
        // optional: clamp to prevent runaway
        if(lastBrown > 1.0f) lastBrown = 1.0f;
        if(lastBrown < -1.0f) lastBrown = -1.0f;
        return lastBrown;
    }

    // pink noise: Voss-McCartney simple implementation
    float nextPink() {
        float white = randFloat(-1.0f, 1.0f);
        // Simple 1/f filter
        pinkStore[0] = 0.99886f * pinkStore[0] + white * 0.0555179f;
        pinkStore[1] = 0.99332f * pinkStore[1] + white * 0.0750759f;
        pinkStore[2] = 0.96900f * pinkStore[2] + white * 0.1538520f;
        pinkStore[3] = 0.86650f * pinkStore[3] + white * 0.3104856f;
        pinkStore[4] = 0.55000f * pinkStore[4] + white * 0.5329522f;
        pinkStore[5] = -0.7616f * pinkStore[5] - white * 0.0168980f;
        pinkStore[6] = white * 0.115926f;  // final small component
        float output = 0.0f;
        for(int i = 0; i < 7; i++) output += pinkStore[i];
        return output;
    }


    // black noise: mostly zero, occasional spike
    float nextBlack() {
        if(rand() % 100 < 2)  // ~2% chance spike
            return randFloat(-1.0f, 1.0f);
        return 0.0f;
    }

    float applyLowPass(float input) {
        filteredSample += lowPassAlpha * (input - filteredSample);
        return filteredSample;
    }

    void updateLowPassAlpha() {
        lowPassAlpha = 1.0f - std::exp((-2.0f * float(M_PI) * cutoffHz) / sampleRate);
    }
};


// Key state tracking
struct KeyState {
    int count = 0;
    bool pressed = false;
};
std::map<int, KeyState> keyStates;

enum Mode {
    MAIN_MENU,
    VOICE_TONE_MENU,
    TONE_MENU,
    WAVE_MENU,
    ADSR_MENU,
    REVERB_MENU,
    NOISE_MENU,
    HARMONIST_MENU,
    WRITE_MENU,
    PRESET_MENU
};
Mode menu = TONE_MENU;
int lastMenuRead = 0;


// ======================================================
//                     Screen Setup
// ======================================================
#define SPI_CHANNEL 0          // CE0
#define SPI_SPEED 8000000      // 8 MHz
#define PIN_DC 25              // Data/Command pin
#define PIN_RES 24             // Reset pin

const int WIDTH = 127;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int global_spi_handle = -1;   // Needed for safe exit

struct Preset;

void clearBuffer();
void updateDisplay(int spi);
void gracefulExit(int signum);
void handlePresetNameKeyPress(int keyID);
void savePresetsToFile();
Preset captureCurrentPreset(const std::string& name);
void configureWritePatchEngines();

// --------------------------------------
//  SPI Send Helpers
// --------------------------------------
void sendCommand(int spi, uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

void sendData(int spi, const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

// --------------------------------------
//  SH1106 Init
// --------------------------------------
void initDisplay(int spi) {
    gpioWrite(PIN_RES, 0);
    usleep(100000);
    gpioWrite(PIN_RES, 1);
    usleep(100000);

    sendCommand(spi, 0xAE);
    sendCommand(spi, 0xD5); sendCommand(spi, 0x80);
    sendCommand(spi, 0xA8); sendCommand(spi, 0x3F);
    sendCommand(spi, 0xD3); sendCommand(spi, 0x00);
    sendCommand(spi, 0x40);
    sendCommand(spi, 0xAD); sendCommand(spi, 0x8B);
    sendCommand(spi, 0xA1);
    sendCommand(spi, 0xC8);
    sendCommand(spi, 0xDA); sendCommand(spi, 0x12);
    sendCommand(spi, 0x81); sendCommand(spi, 0xCF);
    sendCommand(spi, 0xD9); sendCommand(spi, 0xF1);
    sendCommand(spi, 0xDB); sendCommand(spi, 0x40);
    sendCommand(spi, 0xA4);
    sendCommand(spi, 0xA6);
    sendCommand(spi, 0xAF);
}

// --------------------------------------
//  Buffer Management
// --------------------------------------
void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));

}

void clearScreen() {
    // Clear the OLED's internal memory (all 8 pages)
    uint8_t empty[132];      // SH1106 has 132 columns internally
    memset(empty, 0x00, sizeof(empty));

    for(int page = 0; page < 8; page++) {
        sendCommand(global_spi_handle, 0xB0 + page); // select page
        sendCommand(global_spi_handle, 0x00);        // lower column
        sendCommand(global_spi_handle, 0x10);        // upper column
        sendData(global_spi_handle, empty, 132);     // write zeros to all columns
    }
}

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] |= (1 << (y % 8));
}

void clearPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] &= ~(1 << (y % 8));
}

// Bresenham Line
void drawLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        drawPixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
// Helper: draw a rectangle outline using your drawLine
void drawRect(int x, int y, int w, int h) {
    drawLine(x,     y,     x + w - 1, y);         // top
    drawLine(x,     y + h - 1, x + w - 1, y + h - 1); // bottom
    drawLine(x,     y,     x,         y + h - 1); // left
    drawLine(x + w - 1, y, x + w - 1, y + h - 1); // right
}

// Draw rectangle by center position
void drawRectCentered(int cx, int cy, int w, int h) {
    int x = cx - w / 2;
    int y = cy - h / 2;
    drawRect(x, y, w, h);
}


void drawCircle(int cx, int cy, int radius) {
    int x = radius;
    int y = 0;
    int err = 0;

    while (x >= y) {
        drawPixel(cx + x, cy + y);
        drawPixel(cx + y, cy + x);
        drawPixel(cx - y, cy + x);
        drawPixel(cx - x, cy + y);
        drawPixel(cx - x, cy - y);
        drawPixel(cx - y, cy - x);
        drawPixel(cx + y, cy - x);
        drawPixel(cx + x, cy - y);

        y += 1;
        if (err <= 0) {
            err += 2*y + 1;
        } 
        if (err > 0) {
            x -= 1;
            err -= 2*x + 1;
        }
    }
}

void drawFilledCircle(int cx, int cy, int radius) {
    for (int px = -radius; px <= radius; px++) {
        int half = static_cast<int>(sqrtf(float(radius * radius - px * px)));
        for (int py = -half; py <= half; py++) {
            drawPixel(cx + px, cy + py);
        }
    }
}

void drawFilledCircleSparse(int cx, int cy, int radius, float removePercent) {
    removePercent = std::clamp(removePercent, 0.0f, 100.0f);
    float keepChance = 1.0f - (removePercent / 100.0f);

    for (int px = -radius; px <= radius; px++) {
        int half = static_cast<int>(sqrtf(float(radius * radius - px * px)));
        for (int py = -half; py <= half; py++) {
            float r = (float)rand() / RAND_MAX;
            if (r < keepChance) {
                drawPixel(cx + px, cy + py);
            }
        }
    }
}

int harmonyIntervalToX(int interval, int radius) {
    interval = std::clamp(interval, -12, 12);
    float t = float(interval + 24) / 48.0f;
    float x = float(radius) + t * float((WIDTH - 1 - radius) - radius);
    return std::clamp(
        int(x),
        radius,
        WIDTH - 1 - radius
    );
}


// Draw a single character at (x, y)
void drawChar(int x, int y, char c) {
    if (c < 'A' || c > 'Z') return; // ignore non-capitals
    const uint8_t* glyph = font5x7[c - 'A'];

    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) drawPixel(x + col, y + row);
        }
    }
}


// Draw a full text string at (x, y)
void drawText(int x, int y, const char* text) {
    int cursorX = x;
    int cursorY = y;

    while (*text) {
        char c = *text++;

        if (c == '\n') {
            cursorX = x;
            cursorY += 8;   // line spacing
            continue;
        }

        drawChar(cursorX, cursorY, c);
        cursorX += 6;      // 5px glyph + 1px spacing
    }
}

int getTextWidth(const std::string& text) {
    return int(text.size()) * 6;
}

void drawTextCenteredX(int centerX, int y, const std::string& text) {
    drawText(centerX - getTextWidth(text) / 2, y, text.c_str());
}

void drawPixelInverse(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] ^= (1 << (y % 8)); // XOR inverts pixel
}


// Draw a filled rectangle
void drawRectFilled(int x, int y, int w, int h) {
    for (int i = 0; i < w; ++i)
        drawLine(x + i, y, x + i, y + h - 1);
}

void drawRoundedRectFilled(int x, int y, int w, int h, int r) {
    // center rectangle
    drawRect(x + r, y, w - 2*r, h);
    drawRect(x, y + r, w, h - 2*r);

    // four corner circles
    drawCircle(x + r,     y + r,     r);
    drawCircle(x + w-r-1, y + r,     r);
    drawCircle(x + r,     y + h-r-1, r);
    drawCircle(x + w-r-1, y + h-r-1, r);
}

void drawEditIndicator() {
    const int outerSize = 7;
    const int innerSize = 3;
    const int margin = 3;
    const int x = WIDTH - outerSize - margin;
    const int y = margin;

    drawRectFilled(x, y, outerSize, outerSize);

    int holeX = x + (outerSize - innerSize) / 2;
    int holeY = y + (outerSize - innerSize) / 2;
    for (int px = 0; px < innerSize; px++) {
        for (int py = 0; py < innerSize; py++) {
            clearPixel(holeX + px, holeY + py);
        }
    }
}

void drawSantaBelly(int cx, int cy, int size, float fatness) {
    fatness = std::max(0.0f, std::min(1.0f, fatness));

    int circleRadius = size / 2;

    // Rectangle dimensions grow as fatness increases
    int rectW = size + int(fatness * size);
    int rectH = size;

    // Corner radius shrinks as it becomes boxy
    int cornerR = int((1.0f - fatness) * circleRadius);

    if (fatness < 0.05f) {
        // Pure circle
        drawCircle(cx, cy, circleRadius);
    } else {
        // Rounded rectangle
        drawRoundedRectFilled(
            cx - rectW/2,
            cy - rectH/2,
            rectW,
            rectH,
            cornerR
        );
    }
}

void drawSanta(int cx, int cy, float fatness) {
    // Belly
    drawSantaBelly(cx, cy + 10, 24, fatness);

    // Head
    drawCircle(cx, cy - 12, 8);

    // Hat
    drawRectFilled(cx - 10, cy - 22, 20, 6);
    drawLine(cx -5, cy-22, cx+7, cy-28);
    drawLine(cx+5, cy-22, cx+9, cy-28);
    drawCircle(cx + 8, cy - 30, 3); // pompom

    // Beard
    drawCircle(cx - 4, cy - 6, 5);
    drawCircle(cx + 4, cy - 6, 5);

    // Eyes
    drawPixel(cx - 2, cy - 14);
    drawPixel(cx + 2, cy - 14);

    // Belt
    drawRectFilled(cx - 14, cy + 8, 28, 3);
}


// Draw a menu item with selection highlight
void drawMenuItem(int x, int y, int w, int h, const char* text, bool selected=false) {
    // Draw border
    drawLine(x, y, x + w - 1, y);         // top
    drawLine(x, y + h - 1, x + w - 1, y + h - 1); // bottom
    drawLine(x, y, x, y + h - 1);         // left
    drawLine(x + w - 1, y, x + w - 1, y + h - 1); // right

    // Fill if selected
    if (selected) drawRectFilled(x+1, y+1, w-2, h-2);

    // Draw text centered
    int textLen = 0;
    while (text[textLen]) textLen++;
    int charWidth = 6; // 5px + 1 spacing
    int textWidth = charWidth * textLen;
    int textX = x + (w - textWidth)/2;
    int textY = y + (h - 7)/2; // 7px font height

    if (selected) {
        for (int i = 0; i < textLen; i++) {
            char c = text[i];
            const uint8_t* glyph = font5x7[c - 'A'];
            for (int col = 0; col < 5; col++) {
                uint8_t bits = glyph[col];
                for (int row = 0; row < 7; row++) {
                    if (bits & (1 << row))
                        drawPixelInverse(textX + i*6 + col, textY + row);
                }
            }
        }
    } else {
        drawText(textX, textY, text);
    }

}

int menuSelection = 1;
// Draw the full menu
void drawMenu() {
    int menuX = 10;
    int menuY = 5;
    int menuW = 108;  // leave 10px margin on each side (128-108=20)
    int menuH = 11;   // enough for 7px font + padding
    int gap = 4;
    clearBuffer();
    if(menuSelection == 1) {
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "TONE", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "WAVE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "ADSR");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "REVERB");
    }
    else if(menuSelection == 2){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "TONE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "WAVE", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "ADSR");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "REVERB");
    }
    else if(menuSelection == 3){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "TONE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "WAVE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "ADSR", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "REVERB");
    }
    else if(menuSelection == 4){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "TONE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "WAVE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "ADSR");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "REVERB", true);
    }
    else if(menuSelection == 5){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "NOISE", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "HARMONIST");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "WRITE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "PRESETS");
    }
    else if(menuSelection == 6){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "NOISE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "HARMONIST", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "WRITE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "PRESETS");
    }
    else if(menuSelection == 7){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "NOISE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "HARMONIST");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "WRITE", true);
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "PRESETS");
    }
    else if(menuSelection == 8){
        drawMenuItem(menuX, menuY + (menuH + gap) * 0, menuW, menuH, "NOISE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 1, menuW, menuH, "HARMONIST");
        drawMenuItem(menuX, menuY + (menuH + gap) * 2, menuW, menuH, "WRITE");
        drawMenuItem(menuX, menuY + (menuH + gap) * 3, menuW, menuH, "PRESETS", true);
    }
}


unsigned char* img;
// Send buffer to OLED
void updateDisplay(int spi) {
    for (int page = 0; page < HEIGHT/8; page++) {
        sendCommand(spi, 0xB0 + page);
        sendCommand(spi, 0x00);
        sendCommand(spi, 0x10);
        sendData(spi, &buffer[page * WIDTH], WIDTH);
    }
}


// --------------------------------------
//  Ctrl-C Cleanup
// --------------------------------------
void gracefulExit(int signum) {
    std::cout << "\nClearing OLED before exit...\n";

    clearBuffer();
    clearScreen();
    updateDisplay(global_spi_handle);

    spiClose(global_spi_handle);
    gpioTerminate();

    std::cout << "Done.\n";
    exit(0);
}










// ======================================================
//                        Utils
// ======================================================
float norm(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

constexpr float MIN_VISUAL_LEVEL = 0.02f;

float sanitizeDisplaySample(float sample) {
    return std::isfinite(sample) ? sample : 0.0f;
}

int visualJitterAmount(float sample, float referenceLevel, float maxAmt) {
    if (maxAmt <= 0.0f || !std::isfinite(sample)) return 0;

    referenceLevel = std::max(referenceLevel, MIN_VISUAL_LEVEL);
    float magnitude = std::clamp(std::fabs(sample), 0.0f, referenceLevel);
    return std::max(0, int(norm(magnitude, 0.0f, referenceLevel, 0.0f, maxAmt)));
}

int randomOffset(int amt) {
    amt = std::max(0, amt);
    if (amt == 0) return 0;
    return std::rand() % (amt * 2 + 1) - amt;
}

int mapKeyNumber(int k) {
    // switch (k) {
    //     case 6: return 5;
    //     case 5: return 6;
    //     case 13: return 12;
    //     case 12: return 13;
    //     case 20: return 19;
    //     case 19: return 20;
    //     default:
    //         break;
    // }
    return k;
}

// ======================================================
//                     Constants
// ======================================================

const int numVoices = 24;
constexpr int MAX_HARMONIES = 3;

bool normVoices = true; // Normalize by active voices
int fd = -1;
int editIndex = 0;
bool edit = false;

Reverb reverb(sampleRate);

// Menus

// Input device variables
int p1=40, p2=40, p3=40, p4=40;        // start at midpoint
int lastP1=40, lastP2=40, lastP3=40, lastP4=40;
float knobPosition = 0.9f;

// Waveform editor
const int WAVE_RES = 12;
float wavePoints[WAVE_RES];
static std::vector<float> customTable;
static bool waveNeedsRebuild = true;
const int TABLE_SIZE = 8192;
float curvature = 1.0f;
int octave = 1;

// ======================================================
//                     Voice Struct
// ======================================================
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;        // key held
    bool releasing = false;     // is in release phase
    bool usesWritePatch = false;
    float envTime = 0.0f;       // time since note-on or release start
    float releaseStartLevel = 0.0f;
    float oscVolume = 1.0f;
    int keyID = -1;             // store which key triggered this voice
    std::array<float, MAX_HARMONIES> harmonyPhases{};
    std::array<float, MAX_HARMONIES> harmonyFrequencies{};
};
struct HarmonySetting {
    int interval = 7;   // semitone offset from root
    float detune = 0;   // small frequency offset for chorus effect
    float level = 0.35f;
};



Voice voices[numVoices];
HarmonySetting harmonySettings[MAX_HARMONIES] = {
    {7, 0.0f, 0.35f},
    {12, 0.0f, 0.35f},
    {19, 0.0f, 0.35f}
};
int harmonyCount = 0;
bool custom = false;

struct Preset {
    std::string name;
    float outputLevel = 0.0f;
    int octave = 0;
    int bufferLength = 512;
    float clipAmount = 0.0f;
    float knobPosition = 0.0f;
    bool custom = false;
    float curvature = 1.0f;
    std::array<float, WAVE_RES> wavePoints{};
    float attack = 0.0f;
    float decay = 0.0f;
    float sustain = 0.0f;
    float release = 0.0f;
    float rDry = 1.0f;
    float rWet = 0.0f;
    float rSize = 1.0f;
    float rDecay = 0.5f;
    float noiseVolume = 0.0f;
    float noiseFilterCutoff = 20000.0f;
    float noiseAdsrAmount = 1.0f;
    NoiseType noiseType = NOISE_NONE;
    int harmonyCount = 0;
    std::array<HarmonySetting, MAX_HARMONIES> harmonySettings{};
};

std::vector<Preset> presets;
Preset writePatch;
bool writePatchCaptured = false;
std::vector<float> writeCustomTable;
Reverb writeReverb(sampleRate);
NoiseGenerator writeNoise(NOISE_NONE);
float writeLowPolyLowpassState = 0.0f;

enum class PresetScreen {
    LIST,
    OPTIONS,
    NAMING
};

PresetScreen presetScreen = PresetScreen::LIST;

// ADSR
float attack = 0.5f;
float decay = 0.5f;
float sustain = 0.8f;
float release = 0.0f;

// ======================================================
//                  Basic Oscillators
// ======================================================
float sineWave(float phase)      { return sinf(2.0f*M_PI*phase); }
float squareWave(float phase)    { return (phase<0.5f)?1.0f:-1.0f; }
float sawWave(float phase)       { return 2.0f*(phase-0.5f); }
float triangleWave(float phase)  { return 4.0f*fabsf(phase-0.5f)-1.0f; }

// ======================================================
//                  Morph knob mapping
// ======================================================
float getMorphValue(int knobPos) {
    switch(knobPos) {
        case 1: return 0.0f;   
        case 2: return 0.125f; 
        case 3: return 0.25f;  
        case 4: return 0.375f; 
        case 5: return 0.5f;   
        case 6: return 0.625f; 
        case 7: return 0.75f;  
        case 8: return 0.875f; 
        default: return 0.0f;
    }
}

float softClip(float x) {
    float amount = std::clamp(clipAmount, 0.0f, 1.0f);
    if (amount <= 0.001f) return x;

    float drive = MIN_CLIP_DRIVE + std::pow(amount, 1.2f) * (MAX_CLIP_DRIVE - MIN_CLIP_DRIVE);
    float wet = std::clamp(amount * 1.1f, 0.0f, 1.0f);
    float shaped = std::tanh(x * drive);
    float outputTrim = 1.0f / (1.0f + amount * 0.35f);
    return (x + (shaped - x) * wet) * outputTrim;
}

float getOctaveClipDrive() {
    float lowOctaveAmount = float(std::clamp(3 - octave, 0, 3));
    return 1.0f + lowOctaveAmount * 0.05f;
}

float getOctaveOutputBoost(int lowVoiceCount) {
    float lowOctaveAmount = float(std::clamp(3 - octave, 0, 3));
    float boost = 1.0f + lowOctaveAmount * 0.12f;
    if (lowVoiceCount <= 1) return boost;
    return 1.0f + (boost - 1.0f) / (1.0f + 0.75f * float(lowVoiceCount - 1));
}

float getClipAmountFromKnob(int knobValue) {
    float rawAmount = std::clamp(norm(knobValue, 0.0f, maxTurnVal, 0.0f, 1.0f), 0.0f, 1.0f);
    return std::pow(rawAmount, 0.85f);
}

float getLowVoiceMixCompensation(int lowVoiceCount) {
    if (lowVoiceCount <= 1) return 1.0f;
    return 1.0f / (1.0f + 0.16f * float(lowVoiceCount - 1));
}

float applyPolyBassCleanup(float input, int lowVoiceCount) {
    float cleanupAmount = std::clamp(0.22f * float(lowVoiceCount - 1), 0.0f, 0.65f);
    float cutoff = 55.0f + 18.0f * float(std::max(lowVoiceCount - 1, 0));
    float alpha = 1.0f - std::exp((-2.0f * float(M_PI) * cutoff) / sampleRate);
    lowPolyLowpassState += alpha * (input - lowPolyLowpassState);
    float highPassed = input - lowPolyLowpassState;
    return input + (highPassed - input) * cleanupAmount;
}

// ======================================================
//                     Custom Wave (Fixed-Point Editor)
// ======================================================
inline float lerp(float a, float b, float t) { return a + (b-a)*t; }
inline float curveInterp(float a, float b, float t, float curv) {
    if(curv!=1.0f) t=powf(t,curv);
    return lerp(a,b,t);
}

void rebuildCustomTableFromPoints(const std::array<float, WAVE_RES>& points, float patchCurvature, std::vector<float>& table) {
    table.resize(TABLE_SIZE);
    for (int i = 0; i < TABLE_SIZE; i++) {
        float t = float(i) / (TABLE_SIZE - 1);
        float idxF = t * (WAVE_RES - 1);
        int idx = int(idxF);
        float frac = idxF - idx;
        float v1 = points[idx];
        float v2 = points[std::min(idx + 1, WAVE_RES - 1)];
        table[i] = curveInterp(v1, v2, frac, patchCurvature);
    }
}

void rebuildWaveTable() {
    std::array<float, WAVE_RES> points{};
    for (int i = 0; i < WAVE_RES; i++) points[i] = wavePoints[i];
    rebuildCustomTableFromPoints(points, curvature, customTable);
    waveNeedsRebuild=false;
}

void updateWave() { waveNeedsRebuild=true; }

void initWavePoints() {
    for(int i=0;i<WAVE_RES;i++) wavePoints[i] = norm(i,0,WAVE_RES-1,-2.0f,2.0f);
    customTable.resize(TABLE_SIZE);
    waveNeedsRebuild=true;
}

// ======================================================
//                  Note to Hz
// ======================================================
float noteToHz(int noteNumber) {
    float fC0 = 16.35f;
    return fC0*pow(2.0f,float((noteNumber-8)+(octave+2)*12)/12.0f);
}

float noteToHzForOctave(int noteNumber, int octaveValue) {
    float fC0 = 16.35f;
    return fC0 * pow(2.0f, float((noteNumber - 8) + (octaveValue + 2) * 12) / 12.0f);
}
float hzToNote(float freq) {
    float fC0 = 16.35f;
    return 12.0f*log2f(freq/fC0)-24.0f;
}

void refreshVoicePitchCache(Voice& voice) {
    if (voice.keyID < 0) return;

    const bool useWritePatch = voice.usesWritePatch && writePatchCaptured;
    const int voiceOctave = useWritePatch ? writePatch.octave : octave;
    const HarmonySetting* settings = useWritePatch ? writePatch.harmonySettings.data() : harmonySettings;
    voice.frequency = noteToHzForOctave(voice.keyID, voiceOctave);
    for (int h = 0; h < MAX_HARMONIES; h++) {
        const HarmonySetting& setting = settings[h];
        voice.harmonyFrequencies[h] = noteToHzForOctave(voice.keyID + setting.interval, voiceOctave) * (1.0f + setting.detune);
    }
}

void refreshPlayingVoiceFrequencies() {
    for (int v = 0; v < numVoices; v++) {
        if (!voices[v].usesWritePatch && (voices[v].active || voices[v].releasing) && voices[v].keyID >= 0) {
            refreshVoicePitchCache(voices[v]);
        }
    }
}

void setOctave(int newOctave) {
    if (newOctave == octave) return;
    octave = newOctave;
    refreshPlayingVoiceFrequencies();
}

// ======================================================
//                  ADSR Envelope
// ======================================================
std::array<float, 2> ADSR(float attack,float decay,float sustain,float release,bool trig,float t,float lvl, float current=0.0f) {
    float curvature=3.0f;
    if(trig){
        if(t<attack) {
            return {powf(t/attack,curvature)*lvl, 1.0f}; 
        }
        else if(t<attack+decay) {
            return { (1.0f - powf((t-attack)/decay,1.0f/curvature)*(1.0f-sustain))*lvl, 1.0f };
        }
        else {
            return {sustain*lvl, 1.0f};
        }
    }else{
        if(t<release) {

            return { (1.0f - powf(t/release,1.0f/curvature))*(current*lvl), 0.0f };
        
            // return (1.0f - powf(t/release,1.0f/curvature))*(sustain*lvl);
        }
        else {
            return {0.0f, 0.0f};
            
        }
    }
}

constexpr float ADSR_DISPLAY_SUSTAIN_TIME = 0.2f;

float sampleAdsrPreview(float t) {
    float triggerTime = attack + decay + ADSR_DISPLAY_SUSTAIN_TIME;
    if (t < triggerTime) {
        return ADSR(attack, decay, sustain, release, true, t, 1.0f, sustain)[0];
    }
    return ADSR(attack, decay, sustain, release, false, t - triggerTime, 1.0f, sustain)[0];
}

bool getAdsrMarkerPosition(const Voice& voice, float totalTime, int& markerX, int& markerY) {
    if (!voice.active && !voice.releasing) return false;

    float triggerViewTime = attack + decay + ADSR_DISPLAY_SUSTAIN_TIME;
    float markerTime = 0.0f;

    if (voice.releasing) {
        markerTime = std::min(triggerViewTime + voice.envTime, totalTime);
    } else {
        markerTime = std::min(voice.envTime, triggerViewTime);
    }

    float markerEnv = std::clamp(sampleAdsrPreview(markerTime), 0.0f, 1.0f);
    markerX = std::clamp(int((markerTime / totalTime) * (WIDTH - 1)), 0, WIDTH - 1);
    markerY = HEIGHT - 1 - int(markerEnv * (HEIGHT - 1));
    return true;
}

int getCurrentHarmonyIndex() {
    return std::clamp(harmonyCount - 1, 0, MAX_HARMONIES - 1);
}


// ======================================
//         Output Waveform Draw
// ======================================

constexpr int MAX_BUF_LEN = 4096;   // upper limit (power-of-two not required)
std::atomic<int> BUF_LEN{512};     // logical buffer length (adjustable at runtime)
std::atomic<int> NOISE_DRAW_LEN{256};
float sampleBuffer[MAX_BUF_LEN];   // physical storage (preallocated)
std::atomic<int> bufIndex{0};      // next write position (atomic)
const int DRAW_WIDTH = WIDTH;                   // current write position

// Push new sample into circular buffer
// Push new sample into circular buffer (safe from audio thread)
inline void pushSample(float s) {
    s = sanitizeDisplaySample(s);

    // fetch and increment index atomically
    int idx = bufIndex.fetch_add(1, std::memory_order_relaxed);
    int len = BUF_LEN.load(std::memory_order_acquire);
    if (len <= 0) len = 1; // safety
    idx = idx % len; // wrap within current logical length
    sampleBuffer[idx] = s;
    // keep bufIndex bounded to avoid wrap integer overflow over long runs
    if (idx == len - 1) {
        // reset atomic counter occasionally (not strict realtime-critical)
        bufIndex.store(0, std::memory_order_relaxed);
    }
}

// Helper for mapping integers (keeps your existing behavior)
inline int iMap(int val,int inMin,int inMax,int outMin,int outMax){
    return (val - inMin)*(outMax - outMin)/(inMax - inMin) + outMin;
}

// Draw waveform to OLED
void drawOutput() {
    clearBuffer();

    int len = BUF_LEN.load(std::memory_order_acquire);
    if (len <= 0) len = 1;
    if (len > MAX_BUF_LEN) len = MAX_BUF_LEN;

    // Read a snapshot of write index to compute continuous ordering
    int writePos = bufIndex.load(std::memory_order_acquire);
    // If writePos points to next write position, oldest sample is writePos % len
    int start = (writePos) % len; // oldest

    // Find max absolute value for normalization (scan logical length)
    float maxVal = 1e-6f;
    for (int i = 0; i < len; ++i) {
        float v = sanitizeDisplaySample(sampleBuffer[(start + i) % len]);
        float av = std::fabs(v);
        if (av > maxVal) maxVal = av;
    }
    if (!std::isfinite(maxVal) || maxVal < 1e-6f) maxVal = 1.0f;

    // Draw line across screen mapping DISPLAY X -> buffer samples
    for (int x = 0; x < DRAW_WIDTH; ++x) {
        // Map x to a sample index within len: newest should appear at right
        // We map x in 0..DRAW_WIDTH-1 to buffer positions from oldest -> newest
        int bufPos = iMap(x, 0, DRAW_WIDTH - 1, 0, len - 1);
        int idx = (start + bufPos) % len;
        float sample = sanitizeDisplaySample(sampleBuffer[idx]);
        float normSample = std::clamp(sample / maxVal, -1.0f, 1.0f);
        int y = std::clamp(HEIGHT/2 - int(normSample * (HEIGHT/2 - 1)), 0, HEIGHT - 1);

        if (x > 0) {
            // previous sample
            int prevBufPos = iMap(x - 1, 0, DRAW_WIDTH - 1, 0, len - 1);
            int prevIdx = (start + prevBufPos) % len;
            float prevSample = sanitizeDisplaySample(sampleBuffer[prevIdx]);
            float prevNorm = std::clamp(prevSample / maxVal, -1.0f, 1.0f);
            int y0 = std::clamp(HEIGHT/2 - int(prevNorm * (HEIGHT/2 - 1)), 0, HEIGHT - 1);
            drawLine(x - 1, y0, x, y);
        } else {
            // first column: put a small dot
            drawPixel(0, y);
        }
    }
}


NoiseGenerator noise(noiseType);

float softClipWithAmount(float x, float amount) {
    amount = std::clamp(amount, 0.0f, 1.0f);
    if (amount <= 0.001f) return x;

    float drive = MIN_CLIP_DRIVE + std::pow(amount, 1.2f) * (MAX_CLIP_DRIVE - MIN_CLIP_DRIVE);
    float wet = std::clamp(amount * 1.1f, 0.0f, 1.0f);
    float shaped = std::tanh(x * drive);
    float outputTrim = 1.0f / (1.0f + amount * 0.35f);
    return (x + (shaped - x) * wet) * outputTrim;
}

float getOctaveClipDriveForValue(int octaveValue) {
    float lowOctaveAmount = float(std::clamp(3 - octaveValue, 0, 3));
    return 1.0f + lowOctaveAmount * 0.05f;
}

float getOctaveOutputBoostForValue(int octaveValue, int lowVoiceCount) {
    float lowOctaveAmount = float(std::clamp(3 - octaveValue, 0, 3));
    float boost = 1.0f + lowOctaveAmount * 0.12f;
    if (lowVoiceCount <= 1) return boost;
    return 1.0f + (boost - 1.0f) / (1.0f + 0.75f * float(lowVoiceCount - 1));
}

float applyPolyBassCleanupWithState(float input, int lowVoiceCount, float& lowpassState) {
    float cleanupAmount = std::clamp(0.22f * float(lowVoiceCount - 1), 0.0f, 0.65f);
    float cutoff = 55.0f + 18.0f * float(std::max(lowVoiceCount - 1, 0));
    float alpha = 1.0f - std::exp((-2.0f * float(M_PI) * cutoff) / sampleRate);
    lowpassState += alpha * (input - lowpassState);
    float highPassed = input - lowpassState;
    return input + (highPassed - input) * cleanupAmount;
}

float getPatchOutputLevel(const Preset* patch) {
    return patch ? patch->outputLevel : outputLevel;
}

float getPatchClipAmount(const Preset* patch) {
    return patch ? patch->clipAmount : clipAmount;
}

int getPatchOctave(const Preset* patch) {
    return patch ? patch->octave : octave;
}

float getPatchReleaseTime(const Preset* patch) {
    return patch ? patch->release : release;
}

int getPatchHarmonyCount(const Preset* patch) {
    return patch ? patch->harmonyCount : harmonyCount;
}

const HarmonySetting& getPatchHarmonySetting(const Preset* patch, int harmonyIndex) {
    return patch ? patch->harmonySettings[harmonyIndex] : harmonySettings[harmonyIndex];
}

float getNoiseEnvelopeForPatch(float env, const Preset* patch) {
    NoiseType patchNoiseType = patch ? patch->noiseType : noiseType;
    float patchNoiseVolume = patch ? patch->noiseVolume : noiseVolume;
    float patchNoiseAdsrAmount = patch ? patch->noiseAdsrAmount : noiseAdsrAmount;
    if (patchNoiseType == NOISE_NONE || patchNoiseVolume <= 0.0f) return 0.0f;
    return lerp(1.0f, env, std::clamp(patchNoiseAdsrAmount, 0.0f, 1.0f));
}

void accumulateNoiseEnergy(float& noiseEnergy, float env, const Preset* patch) {
    float noiseEnv = getNoiseEnvelopeForPatch(env, patch);
    if (noiseEnv <= 0.0f) return;
    noiseEnergy += noiseEnv * noiseEnv;
}

float renderNoiseMix(float noiseEnergy, const Preset* patch, NoiseGenerator& generator) {
    if (noiseEnergy <= 0.0f) return 0.0f;
    return generator.next() * std::sqrt(noiseEnergy) * NOISE_RENDER_GAIN;
}

float renderOscillatorSample(float phase, const Preset* patch) {
    float oscSample;
    bool patchCustom = patch ? patch->custom : custom;

    if (patchCustom) {
        int idx = int(phase * TABLE_SIZE);
        if (idx >= TABLE_SIZE) idx = TABLE_SIZE - 1;
        const std::vector<float>& table = patch ? writeCustomTable : customTable;
        if (table.empty()) return 0.0f;
        oscSample = table[idx];
    } else {
        float seg = (patch ? patch->knobPosition : knobPosition) * 4.0f;
        int idx = int(seg);
        float blend = seg - idx;
        float w1, w2;
        switch (idx) {
            case 0: w1 = sineWave(phase); w2 = squareWave(phase); break;
            case 1: w1 = squareWave(phase); w2 = sawWave(phase); break;
            case 2: w1 = sawWave(phase); w2 = triangleWave(phase); break;
            case 3: w1 = triangleWave(phase); w2 = sineWave(phase); break;
            default: w1 = w2 = 0.0f;
        }
        oscSample = ((1.0f - blend) * w1 + blend * w2);
    }

    return oscSample;
}

float getVoiceEnvelope(const Voice& voice, float level, const Preset* patch) {
    return ADSR(
        patch ? patch->attack : attack,
        patch ? patch->decay : decay,
        patch ? patch->sustain : sustain,
        patch ? patch->release : release,
        voice.active,
        voice.envTime,
        level,
        voice.releaseStartLevel
    )[0];
}

float renderVoiceSample(Voice& voice, float& voiceEnv) {
    float sample = 0.0f;
    voiceEnv = 0.0f;

    if (voice.active || voice.releasing) {
        voice.envTime += 1.0f / sampleRate;
        const Preset* patch = (voice.usesWritePatch && writePatchCaptured) ? &writePatch : nullptr;
        voiceEnv = getVoiceEnvelope(voice, voice.oscVolume, patch);
        float oscSample = renderOscillatorSample(voice.phase, patch);
        sample = oscSample * voiceEnv;

        voice.phase += voice.frequency / sampleRate;
        if (voice.phase >= 1.0f) voice.phase -= 1.0f;

        if (voice.releasing && voice.envTime >= getPatchReleaseTime(patch)) {
            voice.releasing = false;
            voice.usesWritePatch = false;
            voice.envTime = 0.0f;
            voice.phase = 0.0f;
            voice.releaseStartLevel = 0.0f;
            voice.harmonyPhases.fill(0.0f);
        }
    }

    return sample;
}

float renderHarmonySample(Voice& voice, int harmonyIndex, float voiceEnv) {
    if (!(voice.active || voice.releasing)) return 0.0f;
    const Preset* patch = (voice.usesWritePatch && writePatchCaptured) ? &writePatch : nullptr;
    if (harmonyIndex < 0 || harmonyIndex >= getPatchHarmonyCount(patch)) return 0.0f;

    const HarmonySetting& setting = getPatchHarmonySetting(patch, harmonyIndex);
    if (setting.level <= 0.0f) return 0.0f;

    float env = voiceEnv * setting.level;
    float oscSample = renderOscillatorSample(voice.harmonyPhases[harmonyIndex], patch);
    float sample = oscSample * env;

    voice.harmonyPhases[harmonyIndex] += voice.harmonyFrequencies[harmonyIndex] / sampleRate;
    if (voice.harmonyPhases[harmonyIndex] >= 1.0f) {
        voice.harmonyPhases[harmonyIndex] -= 1.0f;
    }

    return sample;
}

float processPatchMix(float patchMix, int activeVoices, int lowVoiceCount, float noiseEnergy,
                      const Preset* patch, NoiseGenerator& generator, Reverb& patchReverb,
                      float& lowpassState) {
    patchMix += renderNoiseMix(noiseEnergy, patch, generator);

    if (normVoices && activeVoices > 1) {
        patchMix /= activeVoices / 1.7f;
    }

    patchMix *= getLowVoiceMixCompensation(lowVoiceCount);
    patchMix = applyPolyBassCleanupWithState(patchMix, lowVoiceCount, lowpassState);
    patchMix = softClipWithAmount(
        patchMix * getPatchOutputLevel(patch) * getOctaveClipDriveForValue(getPatchOctave(patch)),
        getPatchClipAmount(patch)
    );
    return patchReverb.process(patchMix) * getOctaveOutputBoostForValue(getPatchOctave(patch), lowVoiceCount);
}

float applyWriteFilter(float input) {
    float centered = std::clamp(writeFilterControl, 0.0f, 1.0f) - 0.5f;
    if (std::fabs(centered) <= WRITE_FILTER_CENTER_WIDTH) {
        return input;
    }

    if (centered < 0.0f) {
        float t = std::clamp((writeFilterControl) / (0.5f - WRITE_FILTER_CENTER_WIDTH), 0.0f, 1.0f);
        float cutoff = 30.0f * std::pow((sampleRate * 0.45f) / 30.0f, t);
        float alpha = 1.0f - std::exp((-2.0f * float(M_PI) * cutoff) / sampleRate);
        writeFilterLowpassState += alpha * (input - writeFilterLowpassState);
        return writeFilterLowpassState;
    }

    float t = std::clamp((writeFilterControl - (0.5f + WRITE_FILTER_CENTER_WIDTH)) / (0.5f - WRITE_FILTER_CENTER_WIDTH), 0.0f, 1.0f);
    float cutoff = 25.0f * std::pow((8000.0f) / 25.0f, t);
    float alpha = 1.0f - std::exp((-2.0f * float(M_PI) * cutoff) / sampleRate);
    writeFilterHighpassState += alpha * (input - writeFilterHighpassState);
    return input - writeFilterHighpassState;
}
// ======================================================
//                  Audio Callback
// ======================================================

float mix = 0.0f;
int audioCallback(void *outputBuffer, void* /*inputBuffer*/, unsigned int nBufferFrames,
                  double /*streamTime*/, RtAudioStreamStatus /*status*/, void* /*userData*/) 
{
    float *output = static_cast<float*>(outputBuffer);

    if(custom && waveNeedsRebuild) rebuildWaveTable();

    for (unsigned int i = 0; i < nBufferFrames; i++) {
        float liveMix = 0.0f;
        float writeMix = 0.0f;
        int liveActiveVoices = 0;
        int liveLowVoiceCount = 0;
        float liveNoiseEnergy = 0.0f;
        int writeActiveVoices = 0;
        int writeLowVoiceCount = 0;
        float writeNoiseEnergy = 0.0f;

        for (int v = 0; v < numVoices; v++) {
            Voice& voice = voices[v];
            if (!(voice.active || voice.releasing)) continue;
            const bool useWritePatch = voice.usesWritePatch && writePatchCaptured;

            if (useWritePatch) {
                writeActiveVoices++;
                if (voice.frequency < LOW_POLY_VOICE_HZ) writeLowVoiceCount++;
            } else {
                liveActiveVoices++;
                if (voice.frequency < LOW_POLY_VOICE_HZ) liveLowVoiceCount++;
            }

            float voiceEnv = 0.0f;
            float voiceSample = renderVoiceSample(voice, voiceEnv);
            const Preset* patch = useWritePatch ? &writePatch : nullptr;

            if (useWritePatch) {
                writeMix += voiceSample;
                accumulateNoiseEnergy(writeNoiseEnergy, voiceEnv, patch);
            } else {
                liveMix += voiceSample;
                accumulateNoiseEnergy(liveNoiseEnergy, voiceEnv, patch);
            }

            for (int h = 0; h < getPatchHarmonyCount(patch); h++) {
                float harmonySample = renderHarmonySample(voice, h, voiceEnv);
                float harmonyEnv = voiceEnv * getPatchHarmonySetting(patch, h).level;
                if (useWritePatch) {
                    writeMix += harmonySample;
                    accumulateNoiseEnergy(writeNoiseEnergy, harmonyEnv, patch);
                } else {
                    liveMix += harmonySample;
                    accumulateNoiseEnergy(liveNoiseEnergy, harmonyEnv, patch);
                }
            }
        }

        float processedLiveMix = processPatchMix(
            liveMix, liveActiveVoices, liveLowVoiceCount, liveNoiseEnergy,
            nullptr, noise, reverb, lowPolyLowpassState
        );
        float processedWriteMix = processPatchMix(
            writeMix, writeActiveVoices, writeLowVoiceCount, writeNoiseEnergy,
            writePatchCaptured ? &writePatch : nullptr, writeNoise, writeReverb, writeLowPolyLowpassState
        );
        processedWriteMix = applyWriteFilter(processedWriteMix) * writePlaybackVolume;

        mix = processedLiveMix + processedWriteMix;
        mix = sanitizeDisplaySample(mix);

        pushSample(mix);
        output[2 * i] = mix;
        output[2 * i + 1] = mix;
    }

    return 0;
}


// ======================================================
//                Non-blocking keyboard input via ncurses
// ======================================================
void initKeyboard() {
    initscr();            // start ncurses
    cbreak();             // disable line buffering
    noecho();             // don't echo keys
    nodelay(stdscr, TRUE);// non-blocking getch
    keypad(stdscr, TRUE); // enable special keys
}

void closeKeyboard() {
    endwin();
}

int getKeyPress() {
    int ch = getch();
    if(ch != ERR) return ch;
    return -1;
}

int startVoiceForMappedKey(int mappedKeyID, bool useWritePatch = false) {
    for (int v = 0; v < numVoices; v++) {
        if (!voices[v].active && !voices[v].releasing) {
            voices[v].active = true;
            voices[v].releasing = false;
            voices[v].usesWritePatch = useWritePatch;
            voices[v].keyID = mappedKeyID;
            voices[v].envTime = 0.0f;
            voices[v].releaseStartLevel = 0.0f;
            voices[v].phase = (float)rand() / RAND_MAX;
            for (int h = 0; h < MAX_HARMONIES; h++) {
                voices[v].harmonyPhases[h] = (float)rand() / RAND_MAX;
            }
            refreshVoicePitchCache(voices[v]);
            return v;
        }
    }

    return -1;
}

void releaseVoiceByIndex(int voiceIndex) {
    if (voiceIndex < 0 || voiceIndex >= numVoices) return;

    Voice& voice = voices[voiceIndex];
    if (!voice.active) return;

    const Preset* patch = (voice.usesWritePatch && writePatchCaptured) ? &writePatch : nullptr;
    voice.active = false;
    voice.releasing = true;
    voice.releaseStartLevel = ADSR(
        patch ? patch->attack : attack,
        patch ? patch->decay : decay,
        patch ? patch->sustain : sustain,
        patch ? patch->release : release,
        true,
        voice.envTime,
        1.0f,
        patch ? patch->sustain : sustain
    )[0];
    voice.envTime = 0.0f;
}

void stopWritePlayback() {
    if (writePlaybackVoiceIndex >= 0) {
        releaseVoiceByIndex(writePlaybackVoiceIndex);
        writePlaybackVoiceIndex = -1;
    }

    writePlaybackActive = false;
    writePlaybackLooping = false;
    writePlaybackIndex = 0;
}

void startWritePlayback(unsigned long nowMs, bool loopPlayback = false) {
    stopWritePlayback();
    if (writeNotes.empty() || !writePatchCaptured) return;

    configureWritePatchEngines();
    writePlaybackActive = true;
    writePlaybackLooping = loopPlayback;
    writePlaybackIndex = 0;
    writePlaybackStepStartMs = nowMs;
    writePlaybackVoiceIndex = startVoiceForMappedKey(mapKeyNumber(writeNotes[writePlaybackIndex]), true);
}

void updateWriteControls() {
    if(abs(p1-lastP1)>1) writePlaybackVolume = std::clamp(writePlaybackVolume + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p2-lastP2)>1) writeFilterControl = std::clamp(writeFilterControl + norm(p2-lastP2, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p4-lastP4)>1) writeTempoBpm = std::clamp(writeTempoBpm + norm(p4-lastP4, -maxTurnVal, maxTurnVal, -20.0f, 20.0f), WRITE_MIN_BPM, WRITE_MAX_BPM * 2.5f);
}

void updateWritePlayback(unsigned long nowMs) {
    if (!writePlaybackActive) return;

    float stepMs = 60000.0f / std::max(writeTempoBpm, 1.0f);
    float gateMs = stepMs * WRITE_NOTE_GATE;
    float elapsedMs = float(nowMs - writePlaybackStepStartMs);

    if (writePlaybackVoiceIndex >= 0 && elapsedMs >= gateMs) {
        releaseVoiceByIndex(writePlaybackVoiceIndex);
        writePlaybackVoiceIndex = -1;
    }

    if (elapsedMs < stepMs) return;

    writePlaybackIndex++;
    if (writePlaybackIndex >= writeNotes.size()) {
        // if (!writePlaybackLooping) {
        //     stopWritePlayback();
        //     return;
        // }
        writePlaybackIndex = 0;
    }

    writePlaybackStepStartMs = nowMs;
    writePlaybackVoiceIndex = startVoiceForMappedKey(mapKeyNumber(writeNotes[writePlaybackIndex]), true);
}

void configureWritePatchEngines() {
    if (!writePatchCaptured) return;

    writeReverb.simple = SimpleReverb(0.08f, 0.7f, sampleRate);
    writeReverb.schroeder = SchroederReverb(sampleRate);
    writeReverb.mode = reverb.mode;
    writeReverb.setDryWet(writePatch.rDry, writePatch.rWet);
    writeReverb.setRoomSize(writePatch.rSize);
    writeReverb.setDecay(writePatch.rDecay);

    writeNoise = NoiseGenerator(writePatch.noiseType);
    writeNoise.setType(writePatch.noiseType);
    writeNoise.setCutoff(writePatch.noiseFilterCutoff);
    writeLowPolyLowpassState = 0.0f;
    writeFilterLowpassState = 0.0f;
    writeFilterHighpassState = 0.0f;

    rebuildCustomTableFromPoints(writePatch.wavePoints, writePatch.curvature, writeCustomTable);
}

void captureWritePatch() {
    writePatch = captureCurrentPreset("WRITE");
    writePatchCaptured = true;
    configureWritePatchEngines();
}

void handleWriteSingleClick() {
    if (!edit) {
        writeNotes.clear();
        stopWritePlayback();
        captureWritePatch();
        // Snapshot on enter
        lastP1 = p1; lastP2 = p2; lastP3 = p3; lastP4 = p4;
    }
    edit = !edit;
}

void handleWriteTripleClick(unsigned long nowMs) {
    if (edit || writeNotes.empty()) return;
    startWritePlayback(nowMs, true);
}

void drawWrite() {
    clearBuffer();
}




void onKeyPress(int keyID) {
    std::cout << "Key Pressed: " << keyID << std::endl;
    if (menu == WRITE_MENU && edit && int(writeNotes.size()) < MAX_WRITE_NOTES) {
        writeNotes.push_back(keyID);
    }
    startVoiceForMappedKey(mapKeyNumber(keyID), false);
}

void onKeyRelease(int keyID) {
    keyID = mapKeyNumber(keyID);
    for (int v = 0; v < numVoices; v++) {
        if (!voices[v].usesWritePatch && voices[v].keyID == keyID && voices[v].active) {
            voices[v].active = false;
            voices[v].releasing = true;
            voices[v].releaseStartLevel = ADSR(
                attack,
                decay,
                sustain,
                release,
                true,
                voices[v].envTime,
                1.0f,
                sustain
            )[0];
            voices[v].envTime = 0.0f;
        }
    }
}   

void updateKeyStates() {
    for (size_t r = 0; r < rowPins.size(); ++r) {
        gpioSetMode(rowPins[r], PI_OUTPUT);
        gpioWrite(rowPins[r], 1);
        std::this_thread::sleep_for(std::chrono::microseconds(50)); // bump to 50µs too

        for (size_t c = 0; c < colPins.size(); ++c) {
            int keyID = (r * colPins.size()) + c;
            bool isPhysicalPressed = (gpioRead(colPins[c]) == 1);
            if (isPhysicalPressed) {
                if (keyStates[keyID].count < debounceScans) {
                    keyStates[keyID].count++;
                } else if (!keyStates[keyID].pressed) {
                    // KEY PRESS EVENT
                    keyStates[keyID].pressed = true;
                    if (menu == PRESET_MENU && presetScreen == PresetScreen::NAMING) handlePresetNameKeyPress(keyID);
                    else onKeyPress(keyID); 
                }
            } else {
                if (keyStates[keyID].count > 0) {
                    keyStates[keyID].count--;
                } else if (keyStates[keyID].pressed) {
                    // KEY RELEASE EVENT
                    keyStates[keyID].pressed = false;
                    if (!(menu == PRESET_MENU && presetScreen == PresetScreen::NAMING)) onKeyRelease(keyID);
                }
            }
        }
        gpioWrite(rowPins[r], 0);
        gpioSetMode(rowPins[r], PI_INPUT);
        gpioSetPullUpDown(rowPins[r], PI_PUD_DOWN);
    }
}

// ======================================================
//               Read Input Device
// ======================================================
bool initSerial(const char* port="/dev/ttyUSB0") {
    if (system("stty -F /dev/ttyUSB0 115200 raw") != 0) {
        std::cerr << "stty command failed (non-fatal)\n";
    }
    const std::vector<const char*> candidatePorts = {
        port,
        "/dev/ttyACM1",
        "/dev/ttyACM0",
        "/dev/serial0",
        "/dev/ttyAMA0",
        "/dev/ttyS0",
    };
    std::vector<std::string> triedPorts;

    for (const char* candidate : candidatePorts) {
        if (!candidate || candidate[0] == '\0') continue;
        if (std::find(triedPorts.begin(), triedPorts.end(), candidate) != triedPorts.end()) continue;
        triedPorts.emplace_back(candidate);

        int newFd = open(candidate, O_RDONLY | O_NOCTTY | O_NONBLOCK);
        if (newFd < 0) continue;

        termios tty{};
        if (tcgetattr(newFd, &tty) != 0) {
            close(newFd);
            continue;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(newFd, TCSANOW, &tty) != 0) {
            close(newFd);
            continue;
        }

        fd = newFd;
        std::cerr << "Serial connected on " << candidate << "\n";

        if ((std::string(candidate) == "/dev/serial0" ||
             std::string(candidate) == "/dev/ttyAMA0" ||
             std::string(candidate) == "/dev/ttyS0") &&
            std::find(colPins.begin(), colPins.end(), 14) != colPins.end()) {
            std::cerr << "Warning: GPIO14 is still in colPins and can clash with Pi UART wiring.\n";
        }

        return true;
    }

    fd = -1;
    std::cerr << "No serial device found; continuing without serial input.\n";
    return false;
}

void getInp() {
    if (fd < 0) return;

    static std::string line = "";
    char buf[64];
    int n = read(fd, buf, sizeof(buf));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return;
        std::cerr << "Serial read failed, disabling serial input.\n";
        close(fd);
        fd = -1;
        return;
    }
    if (n == 0) return;

    for (int i = 0; i < n; i++) {
        char c = buf[i];
        if (c == '\n') {
            if (!line.empty()) {
                std::stringstream ss(line);
                std::string label;
                int delta;
                if (ss >> label >> delta) {
                    int* target = nullptr;
                    if      (label == "p1") target = &p1;
                    else if (label == "p2") target = &p2;
                    else if (label == "p3") target = &p3;
                    else if (label == "p4") target = &p4;

                    if (target) {
                        *target = std::clamp(*target + delta, 0, maxTurnVal);
                    }
                }
            }
            line.clear();
        } else if (c != '\r') {
            line += c;
        }
    }
}
void setBufferLength(int newLen) {
    if (newLen < 32) newLen = 32;
    if (newLen > MAX_BUF_LEN) newLen = MAX_BUF_LEN;

    int oldLen = BUF_LEN.load(std::memory_order_acquire);
    if (newLen == oldLen) return;

    int currentPos = bufIndex.load(std::memory_order_acquire) % newLen;
    bufIndex.store(currentPos, std::memory_order_release);
    BUF_LEN.store(newLen, std::memory_order_release);
}

void editTone(){
    if(abs(p1-lastP1)>1) outputLevel = std::clamp(outputLevel + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -maxOutputLevel, maxOutputLevel), 0.0f, maxOutputLevel);
    if(abs(p2-lastP2)>1) setOctave(std::clamp(octave + (p2-lastP2 > 0 ? 1 : -1), 0, 3));
    if(abs(p3-lastP3)>1) {
        int newLen = std::clamp(BUF_LEN.load() + (p3-lastP3) * 16, 32, MAX_BUF_LEN);
        setBufferLength(newLen);
    }
    if(abs(p4-lastP4)>1) clipAmount = std::clamp(clipAmount + norm(p4-lastP4, -maxTurnVal, maxTurnVal, -1.0f, 1.0f), 0.0f, 1.0f);
}

void editWave(){
    if(abs(p1-lastP1)>1) editIndex = std::clamp(editIndex + (p1-lastP1), 0, WAVE_RES-1);
    if(abs(p2-lastP2)>1){
        wavePoints[editIndex] = std::clamp(wavePoints[editIndex] + norm(p2-lastP2, -maxTurnVal, maxTurnVal, -1.0f, 1.0f), -2.0f, 2.0f);
        waveNeedsRebuild = true;
        custom = true;
    }
    if(abs(p3-lastP3)>1) curvature = std::clamp(curvature + norm(p3-lastP3, -maxTurnVal, maxTurnVal, -5.0f, 5.0f), 0.1f, 5.0f);
    if(abs(p4-lastP4)>1) {
        knobPosition = std::clamp(knobPosition + norm(p4-lastP4, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 0.9f);
        custom = false;
        waveNeedsRebuild = true;
    }
    updateWave();
}

void editADSR(){
    if(abs(p1-lastP1)>1) attack  = std::clamp(attack  + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -0.5f, 0.5f), 0.0f, 5.0f);
    if(abs(p2-lastP2)>1) decay   = std::clamp(decay   + norm(p2-lastP2, -maxTurnVal, maxTurnVal, -0.5f, 0.5f), 0.0f, 5.0f);
    if(abs(p3-lastP3)>1) sustain = std::clamp(sustain + norm(p3-lastP3, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p4-lastP4)>1) release = std::clamp(release + norm(p4-lastP4, -maxTurnVal, maxTurnVal, -0.5f, 0.5f), 0.0f, 5.0f);
}
float rDry = 1.0f;
float rWet = 0.0f;
float rSize = 1.0f;
float rDecay = 0.5f;
void editReverb() {
    if(abs(p1-lastP1)>1){
        rDry = std::clamp(rDry + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
        rWet = 1.0f - rDry;
        reverb.setDryWet(rDry, rWet);
    }
    if(abs(p2-lastP2)>1){
        rSize = std::clamp(rSize + norm(p2-lastP2, -maxTurnVal, maxTurnVal, -0.2f, 0.2f), 0.1f, 1.5f);
        reverb.setRoomSize(rSize);
    }
    if(abs(p3-lastP3)>1){
        rDecay = std::clamp(rDecay + norm(p3-lastP3, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.1f, 1.0f);
        reverb.setDecay(rDecay);
    }
}

void editNoise(){
    if(abs(p1-lastP1)>1) noiseVolume = std::clamp(noiseVolume + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p2-lastP2)>1) {
        float cutoffT = std::clamp(norm(noiseFilterCutoff, 40.0f, sampleRate*0.45f, 0.0f, 1.0f) + norm(p2-lastP2, -maxTurnVal, maxTurnVal, -0.05f, 0.05f), 0.0f, 1.0f);
        noiseFilterCutoff = 40.0f * std::pow((sampleRate * 0.45f) / 40.0f, cutoffT);
        noise.setCutoff(noiseFilterCutoff);
    }
    if(abs(p3-lastP3)>1) noiseAdsrAmount = std::clamp(noiseAdsrAmount + norm(p3-lastP3, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p4-lastP4)>1) {
        int nt = std::clamp(int(noiseType) + (p4-lastP4 > 0 ? 1 : -1), 0, 5);
        noiseType = static_cast<NoiseType>(nt);
        noise.setType(noiseType);
    }
}

void editHarmonist() {
    if(abs(p2-lastP2)>1) {
        harmonyCount = std::clamp(harmonyCount + (p2-lastP2 > 0 ? 1 : -1), 0, MAX_HARMONIES);
    }

    if (harmonyCount <= 0) return;

    int currentHarmony = getCurrentHarmonyIndex();
    bool pitchChanged = false;

    if(abs(p1-lastP1)>1) harmonySettings[currentHarmony].level = std::clamp(harmonySettings[currentHarmony].level + norm(p1-lastP1, -maxTurnVal, maxTurnVal, -0.1f, 0.1f), 0.0f, 1.0f);
    if(abs(p3-lastP3)>1) {
        harmonySettings[currentHarmony].interval = std::clamp(harmonySettings[currentHarmony].interval + (p3-lastP3 > 0 ? 1 : -1), -13, 13);
        pitchChanged = true;
    }
    if(abs(p4-lastP4)>1) {
        harmonySettings[currentHarmony].detune = std::clamp(harmonySettings[currentHarmony].detune + norm(p4-lastP4, -maxTurnVal, maxTurnVal, -0.003f, 0.003f), -0.03f, 0.03f);
        pitchChanged = true;
    }

    if (pitchChanged) refreshPlayingVoiceFrequencies();
}

void selectMenu() {
    menuSelection = norm(p4, 0, maxTurnVal, 1, 8);
}

int knobIndex(int itemCount) {
    if (itemCount <= 1) return 0;
    return std::clamp(int(norm(p4, 0.0f, maxTurnVal, 0.0f, float(itemCount))), 0, itemCount - 1);
}

int getPresetListSelectionIndex() {
    int itemCount = int(presets.size()) + 1;
    int baseSelection = knobIndex(itemCount);
    if (baseSelection != lastPresetListKnobSelection) {
        presetListSelectionOffset = 0;
        lastPresetListKnobSelection = baseSelection;
    }

    presetListSelection = std::clamp(baseSelection + presetListSelectionOffset, 0, itemCount - 1);
    presetListSelectionOffset = presetListSelection - baseSelection;
    return presetListSelection;
}

bool movePresetListSelection(int direction) {
    if (direction == 0 || presets.empty()) return false;

    int fromIndex = getPresetListSelectionIndex();
    if (fromIndex < 0 || fromIndex >= int(presets.size())) return false;

    int toIndex = std::clamp(fromIndex + direction, 0, int(presets.size()) - 1);
    if (toIndex == fromIndex) return false;

    std::swap(presets[fromIndex], presets[toIndex]);
    savePresetsToFile();

    int baseSelection = knobIndex(int(presets.size()) + 1);
    presetListSelection = toIndex;
    presetListSelectionOffset = presetListSelection - baseSelection;
    lastPresetListKnobSelection = baseSelection;
    selectedPreset = presetListSelection;
    return true;
}

void editPresetListOrder() {
    if (presetScreen != PresetScreen::LIST) {
        presetReorderAccumulator = 0;
        return;
    }

    int delta = p1 - lastP1;
    if (std::abs(delta) <= 1) return;

    presetReorderAccumulator += delta;

    while (presetReorderAccumulator >= PRESET_REORDER_KNOB_STEP) {
        if (!movePresetListSelection(1)) {
            presetReorderAccumulator = 0;
            return;
        }
        presetReorderAccumulator -= PRESET_REORDER_KNOB_STEP;
    }

    while (presetReorderAccumulator <= -PRESET_REORDER_KNOB_STEP) {
        if (!movePresetListSelection(-1)) {
            presetReorderAccumulator = 0;
            return;
        }
        presetReorderAccumulator += PRESET_REORDER_KNOB_STEP;
    }
}



Preset captureCurrentPreset(const std::string& name) {
    Preset preset;
    preset.name = name;
    preset.outputLevel = outputLevel;
    preset.octave = octave;
    preset.bufferLength = BUF_LEN.load(std::memory_order_acquire);
    preset.clipAmount = clipAmount;
    preset.knobPosition = knobPosition;
    preset.custom = custom;
    preset.curvature = curvature;
    for (int i = 0; i < WAVE_RES; i++) {
        preset.wavePoints[i] = wavePoints[i];
    }
    preset.attack = attack;
    preset.decay = decay;
    preset.sustain = sustain;
    preset.release = release;
    preset.rDry = rDry;
    preset.rWet = rWet;
    preset.rSize = rSize;
    preset.rDecay = rDecay;
    preset.noiseVolume = noiseVolume;
    preset.noiseFilterCutoff = noiseFilterCutoff;
    preset.noiseAdsrAmount = noiseAdsrAmount;
    preset.noiseType = noiseType;
    preset.harmonyCount = harmonyCount;
    for (int i = 0; i < MAX_HARMONIES; i++) {
        preset.harmonySettings[i] = harmonySettings[i];
    }
    return preset;
}

void applyPreset(const Preset& preset) {
    outputLevel = preset.outputLevel;
    octave = preset.octave;
    setBufferLength(preset.bufferLength);
    clipAmount = preset.clipAmount;
    knobPosition = preset.knobPosition;
    custom = preset.custom;
    curvature = preset.curvature;
    for (int i = 0; i < WAVE_RES; i++) {
        wavePoints[i] = preset.wavePoints[i];
    }
    waveNeedsRebuild = true;

    attack = preset.attack;
    decay = preset.decay;
    sustain = preset.sustain;
    release = preset.release;

    rDry = preset.rDry;
    rWet = preset.rWet;
    rSize = preset.rSize;
    rDecay = preset.rDecay;
    reverb.setDryWet(rDry, rWet);
    reverb.setRoomSize(rSize);
    reverb.setDecay(rDecay);

    noiseVolume = preset.noiseVolume;
    noiseFilterCutoff = preset.noiseFilterCutoff;
    noiseAdsrAmount = preset.noiseAdsrAmount;
    noiseType = preset.noiseType;
    noise.setCutoff(noiseFilterCutoff);
    noise.setType(noiseType);

    harmonyCount = preset.harmonyCount;
    for (int i = 0; i < MAX_HARMONIES; i++) {
        harmonySettings[i] = preset.harmonySettings[i];
    }
    refreshPlayingVoiceFrequencies();
}

void savePresetsToFile() {
    std::ofstream out(PRESET_FILE_PATH, std::ios::trunc);
    if (!out) return;

    out << presets.size() << "\n";
    for (const Preset& preset : presets) {
        out << preset.name << "\n";
        out << preset.outputLevel << " " << preset.octave << " " << preset.bufferLength << " "
            << preset.clipAmount << " " << preset.knobPosition << " " << preset.custom << " "
            << preset.curvature << "\n";
        for (int i = 0; i < WAVE_RES; i++) {
            if (i > 0) out << " ";
            out << preset.wavePoints[i];
        }
        out << "\n";
        out << preset.attack << " " << preset.decay << " " << preset.sustain << " " << preset.release << "\n";
        out << preset.rDry << " " << preset.rWet << " " << preset.rSize << " " << preset.rDecay << "\n";
        out << preset.noiseVolume << " " << preset.noiseFilterCutoff << " " << preset.noiseAdsrAmount << " "
            << int(preset.noiseType) << "\n";
        out << preset.harmonyCount << "\n";
        for (int i = 0; i < MAX_HARMONIES; i++) {
            out << preset.harmonySettings[i].interval << " "
                << preset.harmonySettings[i].detune << " "
                << preset.harmonySettings[i].level << "\n";
        }
    }
}

void loadPresetsFromFile() {
    presets.clear();

    std::ifstream in(PRESET_FILE_PATH);
    if (!in) return;

    int presetCount = 0;
    if (!(in >> presetCount)) return;

    for (int p = 0; p < presetCount; p++) {
        Preset preset;
        if (!(in >> preset.name)) break;
        int customFlag = 0;
        int noiseTypeValue = 0;

        if (!(in >> preset.outputLevel >> preset.octave >> preset.bufferLength
              >> preset.clipAmount >> preset.knobPosition >> customFlag >> preset.curvature)) break;
        preset.custom = (customFlag != 0);

        for (int i = 0; i < WAVE_RES; i++) {
            if (!(in >> preset.wavePoints[i])) return;
        }

        if (!(in >> preset.attack >> preset.decay >> preset.sustain >> preset.release)) break;
        if (!(in >> preset.rDry >> preset.rWet >> preset.rSize >> preset.rDecay)) break;
        if (!(in >> preset.noiseVolume >> preset.noiseFilterCutoff >> preset.noiseAdsrAmount >> noiseTypeValue)) break;
        preset.noiseType = static_cast<NoiseType>(noiseTypeValue);
        if (!(in >> preset.harmonyCount)) break;

        for (int i = 0; i < MAX_HARMONIES; i++) {
            if (!(in >> preset.harmonySettings[i].interval
                  >> preset.harmonySettings[i].detune
                  >> preset.harmonySettings[i].level)) return;
        }

        preset.harmonyCount = std::clamp(preset.harmonyCount, 0, MAX_HARMONIES);
        presets.push_back(preset);
    }
}

std::string defaultPresetName() {
    return std::string("PRESET") + char('A' + (int(presets.size()) % 24));
}

void beginPresetNaming() {
    presetNameInput.clear();
    presetScreen = PresetScreen::NAMING;
}

void finishPresetNaming() {
    std::string name = presetNameInput.empty() ? defaultPresetName() : presetNameInput;
    presets.push_back(captureCurrentPreset(name));
    savePresetsToFile();
    selectedPreset = int(presets.size()) - 1;
    presetListSelection = selectedPreset;
    presetScreen = PresetScreen::LIST;
}

void loadSelectedPreset() {
    if (selectedPreset < 0 || selectedPreset >= int(presets.size())) return;
    applyPreset(presets[selectedPreset]);
    menu = MAIN_MENU;
    presetListSelection = selectedPreset;
    presetScreen = PresetScreen::LIST;
}

void deleteSelectedPreset() {
    if (selectedPreset < 0 || selectedPreset >= int(presets.size())) return;
    presets.erase(presets.begin() + selectedPreset);
    savePresetsToFile();
    if (selectedPreset >= int(presets.size())) {
        selectedPreset = std::max(0, int(presets.size()) - 1);
    }
    presetListSelection = std::min(presetListSelection, int(presets.size()));
    presetScreen = PresetScreen::LIST;
}

void handlePresetSingleClick() {
    if (presetScreen == PresetScreen::LIST) {
        selectedPreset = presetListSelection;
        if (selectedPreset >= int(presets.size())) beginPresetNaming();
        else presetScreen = PresetScreen::OPTIONS;
        return;
    }

    if (presetScreen == PresetScreen::OPTIONS) {
        if (presetOptionSelection == 0) deleteSelectedPreset();
        else loadSelectedPreset();
        return;
    }

    if (presetScreen == PresetScreen::NAMING) {
        finishPresetNaming();
    }
}

void handlePresetNameKeyPress(int keyID) {
    if (presetScreen != PresetScreen::NAMING) return;
    if (keyID < 0 || keyID > 23) return;
    if (keyID == 23) {
        if (!presetNameInput.empty()) presetNameInput.pop_back();
        return;
    }
    if (int(presetNameInput.size()) >= MAX_PRESET_NAME_LEN) return;
    presetNameInput.push_back(char('A' + keyID));
}



void drawADSR() {
    clearBuffer();

    float totalTime = std::max(attack + decay + ADSR_DISPLAY_SUSTAIN_TIME + release, 0.001f);
    int lastY = -1;
    for (int x = 0; x < WIDTH; x++) {
        float t = (float)x / (WIDTH - 1) * totalTime;
        float env = sampleAdsrPreview(t);

        int y = HEIGHT - 1 - int(env * (HEIGHT - 1));
        if (lastY >= 0) drawLine(x-1, lastY, x, y);
        lastY = y;
    }

    for (int v = 0; v < numVoices; v++) {
        int markerX = 0;
        int markerY = 0;
        if (getAdsrMarkerPosition(voices[v], totalTime, markerX, markerY)) {
            drawFilledCircle(markerX, markerY, 2);
        }
    }
}



void drawWave() {
    clearBuffer();

    const int width  = WIDTH;
    const int height = HEIGHT;

    // First pass: compute samples
    std::vector<float> samples(width);
    float maxAbs = 0.0f;

    for (int x = 0; x < width; x++) {
        float oscSample;

        if(custom){
            int idx = int((float)x / (width - 1) * (TABLE_SIZE - 1));
            if(idx >= TABLE_SIZE) idx = TABLE_SIZE - 1;
            oscSample = customTable[idx];
        } else {
            float seg = knobPosition * 4.0f;
            int idx = int(seg);
            float blend = seg - idx;
            float phase = float(x) / (width - 1); // 0..1
            float w1, w2;

            switch(idx){
                case 0: w1 = sineWave(phase); w2 = squareWave(phase); break;
                case 1: w1 = squareWave(phase); w2 = sawWave(phase); break;
                case 2: w1 = sawWave(phase); w2 = triangleWave(phase); break;
                case 3: w1 = triangleWave(phase); w2 = sineWave(phase); break;
                default: w1 = w2 = 0.0f;
            }

            oscSample = (1.0f - blend) * w1 + blend * w2;
        }

        samples[x] = oscSample;
        if(fabs(oscSample) > maxAbs) maxAbs = fabs(oscSample);
    }

    if(maxAbs < 1e-6f) maxAbs = 1.0f; // avoid divide by zero

    // Second pass: draw lines
    int lastY = height / 2 - int(samples[0] / maxAbs * (height/2 - 1));
    for(int x = 1; x < width; x++) {
        int y = height / 2 - int(samples[x] / maxAbs * (height/2 - 1));
        drawLine(x-1, lastY, x, y);
        lastY = y;
    }
    drawLine(editedIndex, 0, editedIndex, height-1); // vertical line for edited point
}
void drawReverb() {
    clearBuffer();
    int dCay = 14-norm(rWet, 0.0f, 1.0f, 0.0f, 14.0f);
    int dSize = norm(rSize, 0.1f, 1.5f, 10.0f, 63.0f-dCay);
    int dWet = norm(rDecay, 0.1f, 1.0f, 0.0f, dSize);
    int jitterX = 0;
    int jitterY = 0;
    int amt = visualJitterAmount(mix, outputLevel, 10.0f);
    
    drawRectCentered(64, 32, dSize, dSize);
    drawCircle(64, 32, dWet/2);
    
    for(int i = 0; i < dCay/2; i++){
        jitterX = randomOffset(amt);
        jitterY = randomOffset(amt);
        
        
        drawRectCentered(64+jitterX, 32+jitterY, dSize+pow(i, 2), dSize+pow(i,2));
    }
}


int width, height, channels;

void drawNoise() {
    clearBuffer();

    // Only free img if it was previously allocated
    if(img) {
        stbi_image_free(img);
        img = nullptr;
    }

    // Load the correct image
    const char* filename = nullptr;
    switch(noiseType) {
        case PINK_NOISE: filename = "pink.png"; break;
        case BLACK_NOISE: filename = "black.png"; break;
        case BROWN_NOISE: filename = "brown.png"; break;
        case WHITE_NOISE: filename = "white.png"; break;
        case RED_NOISE: filename = "red.png"; break;
        case NOISE_NONE: filename = "none.png"; break;
    }

    if(filename) {
        img = stbi_load(filename, &width, &height, &channels, 1);
        if(!img) {
            fprintf(stderr, "Failed to load %s\n", filename);
            return;
        }
    } else {
        return;
    }

    // Fill buffer
    for(int page = 0; page < 8; page++){
        for(int x = 0; x < WIDTH; x++){
            uint8_t byte = 0;
            for(int bit = 0; bit < 8; bit++){
                int y = page * 8 + bit;
                if(y >= height || x >= width) continue; // safety check
                int pixel = img[y * width + x]; // safe
                if(pixel == 0) byte |= (1 << bit);
            }
            buffer[page * WIDTH + x] = byte;
        }
    }

    int storedLen = BUF_LEN.load(std::memory_order_acquire);
    int noiseLen = NOISE_DRAW_LEN.load(std::memory_order_acquire);
    int len = std::min(storedLen, noiseLen);
    if (len <= 0) len = 1;
    if (len > MAX_BUF_LEN) len = MAX_BUF_LEN;

    int writePos = bufIndex.load(std::memory_order_acquire);
    int start = writePos % len; // oldest sample

    float cx = WIDTH / 2.0f;
    float cy = HEIGHT / 2.0f;
    float R = 20.0f;          // base circle radius
    float scale = 30.0f;     // scale factor for sample amplitudes
    float maxR = std::min(std::min(cx, float(WIDTH - 1) - cx), std::min(cy, float(HEIGHT - 1) - cy));
    
    for (int i = 0; i < DRAW_WIDTH; ++i) {
        // map x to buffer index
        int bufPos = iMap(i, 0, DRAW_WIDTH - 1, 0, len - 1);
        int idx = (start + bufPos) % len;
        float sample = sanitizeDisplaySample(sampleBuffer[idx]);  // raw value

        float angle = 2.0f * M_PI * i / DRAW_WIDTH;
        float r = std::clamp(R + sample * scale, 1.0f, maxR);

        float x = cx + r * cos(angle);
        float y = cy + r * sin(angle);

        if (i > 0) {
            // previous point
            int prevBufPos = iMap(i - 1, 0, DRAW_WIDTH - 1, 0, len - 1);
            int prevIdx = (start + prevBufPos) % len;
            float prevSample = sanitizeDisplaySample(sampleBuffer[prevIdx]);

            float prevAngle = 2.0f * M_PI * (i - 1) / DRAW_WIDTH;
            float prevR = std::clamp(R + prevSample * scale, 1.0f, maxR);
            float x0 = cx + prevR * cos(prevAngle);
            float y0 = cy + prevR * sin(prevAngle);

            drawLine(int(x0), int(y0), int(x), int(y));
        } else {
            drawPixel(int(x), int(y));
        }
    }
    float cuttoffT = std::clamp(norm(p2, 0.0f, maxTurnVal, -1.0f, 101.0f), 0.0f, 100.0f);
    drawFilledCircleSparse(int(cx), int(cy), 10, cuttoffT);

    stbi_image_free(img);
    img = nullptr;
}

int jitter(int x, int amt) {
    return x + randomOffset(amt);
}
void drawHarmonist() {
    clearBuffer();
    int amt = visualJitterAmount(mix, outputLevel, 3.0f);

    if (harmonyCount == 0) {
        drawCircle(jitter(64, amt), jitter(32, amt), 10);
        return;
    }

    std::array<int, MAX_HARMONIES> pointX{};
    std::array<int, MAX_HARMONIES> pointY{};
    std::array<int, MAX_HARMONIES> pointRadius{};

    for (int i = 0; i < harmonyCount; i++) {
        int baseY = 32;
        int baseRadius = 7;

        if (harmonyCount == 2) {
            baseY = (i == 0) ? 20 : 46;
            baseRadius = 6;
        } else if (harmonyCount == 3) {
            baseY = (i == 0) ? 18 : ((i == 1) ? 32 : 46);
            baseRadius = 3;
        }

        int radius = baseRadius + std::abs(harmonySettings[i].level * 5);
        int x = jitter(harmonyIntervalToX(harmonySettings[i].interval, radius), amt);
        int y = jitter(baseY, amt);

        pointX[i] = x;
        pointY[i] = y;
        pointRadius[i] = radius;

        drawFilledCircleSparse(x, y, radius, std::abs(harmonySettings[i].detune * 3000));
    }

    for (int i = 0; i < harmonyCount; i++) {
        int interval = std::abs(harmonySettings[i].interval);
        bool isHarmonicInterval = interval == 3 || interval == 4 || interval == 5 || interval == 7 || interval == 12;
        if (!isHarmonicInterval) continue;

        int lineHalfLength = pointRadius[i] + 2;
        drawLine(pointX[i], pointY[i] - lineHalfLength, pointX[i], pointY[i] + lineHalfLength);
    }
    
}

void drawPresetList() {
    clearBuffer();

    int itemCount = int(presets.size()) + 1;
    presetListSelection = getPresetListSelectionIndex();

    drawTextCenteredX(WIDTH / 2, 2, "PRESETS");

    const int visibleItems = 4;
    const int menuX = 10;
    const int menuY = 14;
    const int menuW = 108;
    const int menuH = 11;
    const int gap = 2;
    int firstVisible = 0;

    if (presetListSelection >= visibleItems) {
        firstVisible = presetListSelection - visibleItems + 1;
    }

    for (int row = 0; row < visibleItems; row++) {
        int index = firstVisible + row;
        if (index >= itemCount) break;

        std::string label = (index < int(presets.size())) ? presets[index].name : "ADD";
        drawMenuItem(menuX, menuY + row * (menuH + gap), menuW, menuH, label.c_str(), index == presetListSelection);
    }
}

void drawPresetNaming() {
    clearBuffer();

    drawTextCenteredX(WIDTH / 2, 4, "TYPE TITLE");
    drawRect(10, 18, 108, 14);

    std::string displayName = presetNameInput.empty() ? "A TO W" : presetNameInput;
    drawTextCenteredX(WIDTH / 2, 22, displayName);
    drawTextCenteredX(WIDTH / 2, 38, "X BACKSPACE");
    drawTextCenteredX(WIDTH / 2, 46, "PRESS BTN");
    drawTextCenteredX(WIDTH / 2, 54, "TO ADD");
}

void drawPresetOptions() {
    clearBuffer();
    if (selectedPreset < 0 || selectedPreset >= int(presets.size())) {
        presetScreen = PresetScreen::LIST;
        drawPresetList();
        return;
    }

    presetOptionSelection = knobIndex(2);
    drawTextCenteredX(WIDTH / 2, 10, presets[selectedPreset].name);
    if (presetOptionSelection == 0) {
        drawMenuItem(8, 29, 53, 14, "DELETE", true);
        drawMenuItem(72, 29, 53, 14, "ENTER");
    } else {
        drawMenuItem(8, 29, 53, 14, "DELETE");
        drawMenuItem(72, 29, 53, 14, "ENTER", true);
    }
}

// ======================================================
//                  Device Switching
// ======================================================

void switchToDevice(unsigned int newDeviceId) {
    if (deviceSwitching.load()) return; // Already switching
    deviceSwitching.store(true);

    try {
        // Stop the current stream
        if (dac.isStreamRunning()) {
            dac.stopStream();
        }
        if (dac.isStreamOpen()) {
            dac.closeStream();
        }

        // Update device ID
        oParams.deviceId = newDeviceId;
        currentDeviceId = newDeviceId;

        // Restart with new device
        unsigned int bufferFrames = 1024;
        dac.openStream(&oParams, nullptr, RTAUDIO_FLOAT32, sampleRate, &bufferFrames, &audioCallback);
        dac.startStream();

        std::cout << "Switched to audio device: " << newDeviceId << std::endl;
    } catch (RtAudioError &e) {
        std::cerr << "Failed to switch audio device: " << e.getMessage() << std::endl;
        // Try to restart with old device
        try {
            unsigned int bufferFrames = 1024;
            dac.openStream(&oParams, nullptr, RTAUDIO_FLOAT32, sampleRate, &bufferFrames, &audioCallback);
            dac.startStream();
        } catch (RtAudioError &e2) {
            std::cerr << "Failed to restart audio: " << e2.getMessage() << std::endl;
        }
    }

    deviceSwitching.store(false);
}

void monitorAudioDevices() {
    unsigned int lastDeviceCount = dac.getDeviceCount();

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(5)); // Check every 5 seconds

        unsigned int currentDeviceCount = dac.getDeviceCount();
        if (currentDeviceCount > lastDeviceCount) {
            // New device(s) detected
            std::cout << "New audio device(s) detected. Device count: " << currentDeviceCount << std::endl;

            // Find the highest device ID (assuming new devices have higher IDs)
            unsigned int newDeviceId = currentDeviceCount - 1;

            // Check if it's an output device
            RtAudio::DeviceInfo info = dac.getDeviceInfo(newDeviceId);
            if (info.outputChannels > 0) {
                std::cout << "Switching to new output device: " << info.name << std::endl;
                switchToDevice(newDeviceId);
            }

            lastDeviceCount = currentDeviceCount;
        } else if (currentDeviceCount < lastDeviceCount) {
            // Device removed, but we'll keep current if possible
            lastDeviceCount = currentDeviceCount;
        }
    }
}



// ======================================================
//                        MAIN
// ======================================================
int main() {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));

    initSerial();

    reverb.mode = ReverbType::SCHROEDER;
    reverb.setDryWet(1.0f,0.0f);
    reverb.setRoomSize(10.0f);
    reverb.setDecay(0.9f);

    initWavePoints();
    loadPresetsFromFile();
    if (presets.empty()) {
        presets.push_back(captureCurrentPreset("DEFAULT"));
        savePresetsToFile();
    }


    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }
    initMatrix();
    signal(SIGINT, gracefulExit);

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    

    // Initialize button pin (pin 16) with pull-down
    gpioSetMode(16, PI_INPUT);
    gpioSetPullUpDown(16, PI_PUD_DOWN);

    global_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (global_spi_handle < 0) {
        std::cerr << "SPI failed\n";
        return 1;
    }

    initDisplay(global_spi_handle);
    clearBuffer();
    clearScreen();
    updateDisplay(global_spi_handle);


    // RtAudio setup
    oParams.deviceId = dac.getDefaultOutputDevice();
    currentDeviceId = oParams.deviceId;
    oParams.nChannels = 2;
    unsigned int bufferFrames = 1024;

    try {
        dac.openStream(&oParams,nullptr,RTAUDIO_FLOAT32,
                       sampleRate,&bufferFrames,&audioCallback);
        dac.startStream();
    } catch(RtAudioError &e){
        e.printMessage();
        return 1;
    }

    // Start background device monitoring thread
    std::thread deviceMonitor(monitorAudioDevices);
    deviceMonitor.detach(); // Run in background

    std::cout << "Polyphonic Synth Ready.\n";
    std::cout << "Press keys z,x,c,v to trigger voices, 1–3 for menus.\n";
    
    

    while(true){
        getInp(); // microcontroller input
        unsigned long now = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        int currentRead = gpioRead(16);

        if (currentRead == 1 && lastMenuRead == 0) { // button pressed
            buttonPressStartTime = now;
            writeHoldTriggered = false;

            // Notify Arduino on every button press
            if (fd >= 0) {
                const char* resetMsg = "reset\n";
                if (write(fd, resetMsg, strlen(resetMsg)) < 0) {
                    std::cerr << "Serial write failed\n";
                }
            }

            if (menu == WRITE_MENU) {
                if (singleClickPending && (now - lastClickTime <= doubleClickDelay)) {
                    writePendingClickCount = std::min(writePendingClickCount + 1, 3);
                    lastClickTime = now;
                } else {
                    singleClickPending = true;
                    writePendingClickCount = 1;
                    lastClickTime = now;
                }
            } else {
                if (singleClickPending && (now - lastClickTime <= doubleClickDelay)) {
                    // Double click detected
                    singleClickPending = false;
                    lastClickTime = 0;
                    writeHoldTriggered = false;
                    writePendingClickCount = 0;

                    std::cout << "Double click detected!" << std::endl;
                    // TODO: Handle double click (e.g., special menu)
                    menu = static_cast<Mode>(MAIN_MENU); 
                    presetScreen = PresetScreen::LIST;
                    presetNameInput.clear();

                } else {
                    // First click, maybe a single click
                    singleClickPending = true;
                    lastClickTime = now;
                }
            }
        }

        if (currentRead == 0 && lastMenuRead == 1 && writeHoldTriggered) {
            stopWritePlayback();
            writeHoldTriggered = false;
        }

        lastMenuRead = currentRead;

        if (menu == WRITE_MENU) {
            updateWriteControls();
            if (!edit && !writeNotes.empty() && currentRead == 1 && singleClickPending &&
                writePendingClickCount <= 1 && !writeHoldTriggered && !writePlaybackLooping &&
                (now - buttonPressStartTime >= WRITE_HOLD_DELAY_MS)) {
                singleClickPending = false;
                writePendingClickCount = 0;
                writeHoldTriggered = true;
                startWritePlayback(now, false);
            }
        } else {
            writeHoldTriggered = false;
            writePendingClickCount = 0;
        }
        updateWritePlayback(now);

        if(lastP1==-1){ lastP1=p1; lastP2=p2; lastP3=p3; lastP4=p4; }
        // menu edits
        if (menu==MAIN_MENU) {
            edit = false;
            selectMenu();
            drawMenu();
        }
        if(menu==TONE_MENU) {
            if(edit) editTone();
            drawOutput();
        }
        if(menu==WAVE_MENU) {
            if(edit) editWave();
            drawWave();
        }

        if(menu==ADSR_MENU) {
            if(edit) editADSR();
            drawADSR();
        }
        if(menu==REVERB_MENU) {
            if(edit) editReverb();
            drawReverb();
        }
        if(menu==NOISE_MENU) {
            if(edit) editNoise();
            drawNoise();
        }
        if(menu==HARMONIST_MENU) {
            if(edit) editHarmonist();
            drawHarmonist();
        }
        if(menu==WRITE_MENU) {
            drawWrite();
        }
        if(menu==PRESET_MENU) {
            edit = false;
            editPresetListOrder();
            if (presetScreen == PresetScreen::LIST) drawPresetList();
            else if (presetScreen == PresetScreen::NAMING) drawPresetNaming();
            else drawPresetOptions();
        }

        if (edit) {
            drawEditIndicator();
        }

        updateDisplay(global_spi_handle);


        updateKeyStates(); // scan keyboard matrix

        if (singleClickPending) {
            bool waitingForWriteRelease = (menu == WRITE_MENU && currentRead == 1 && !writeHoldTriggered);
            if (!waitingForWriteRelease && (now - lastClickTime) > doubleClickDelay) {
                // Single click confirmed
                singleClickPending = false;

                // Original single-click behavior
                if(menu == MAIN_MENU) {
                    edit = false;
                    menu = static_cast<Mode>(menuSelection+1);
                    if (menu == PRESET_MENU) {
                        presetScreen = PresetScreen::LIST;
                        presetListSelection = std::min(presetListSelection, int(presets.size()));
                    }
                }
                else if (menu == PRESET_MENU) handlePresetSingleClick();
                else if (menu == WRITE_MENU) {
                    if (writePendingClickCount >= 3) {
                        if (!writePlaybackLooping) handleWriteTripleClick(now);
                    }
                    else if (writePendingClickCount == 2) {
                        writeHoldTriggered = false;
                        std::cout << "Double click detected!" << std::endl;
                        menu = static_cast<Mode>(MAIN_MENU);
                        presetScreen = PresetScreen::LIST;
                        presetNameInput.clear();
                    } else if (writePlaybackLooping) {
                        stopWritePlayback();
                    } else {
                        handleWriteSingleClick();
                    }
                    writePendingClickCount = 0;
                }
                else {
                    edit = !edit;
                    if (edit) {
                        // Snapshot encoder positions so params don't jump on first turn
                        lastP1 = p1;
                        lastP2 = p2;
                        lastP3 = p3;
                        lastP4 = p4;
                    }
                }

                std::cout << "Single click detected!" << std::endl;
            }
        }
        usleep(1000);
        lastP1=p1; lastP2=p2; lastP3=p3; lastP4=p4;
    }
    try{ dac.stopStream(); } catch(RtAudioError &e){}
    if(dac.isStreamOpen()) dac.closeStream();
    clearScreen();
    return 0;
}

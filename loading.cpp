// px1_loading_standalone.cpp
// Standalone PX-1 loading screen — run this WHILE your main project compiles.
//
// Compile:
//   g++ px1_loading_standalone.cpp -o px1_loading -lpigpio -lpng16 -lrt -lpthread
//
// Usage (run alongside your build):
//   ./px1_loading &
//   make
//   kill %1
//
//   OR use the build wrapper script below to handle it automatically.

#include <cmath>
#include <cstring>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include <vector>
#include <cstdio>
#include <unistd.h>
#include <iostream>
#include <pigpio.h>
#include <png.h>

// -----------------------------------------------------------------------
//  SH1106 SPI config — match your existing setup
// -----------------------------------------------------------------------
#define SPI_CHANNEL  0
#define SPI_SPEED    8000000
#define PIN_DC       25
#define PIN_RES      24

const int WIDTH  = 127;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int spi;

struct MonoSprite {
    int width = 0;
    int height = 0;
    int minX = 0;
    int minY = 0;
    int maxX = -1;
    int maxY = -1;
    bool loaded = false;
    std::vector<uint8_t> mask;
};

MonoSprite headphonesSprite;

// -----------------------------------------------------------------------
//  SPI helpers
// -----------------------------------------------------------------------
void sendCommand(uint8_t cmd) {
    gpioWrite(PIN_DC, 0);
    spiWrite(spi, (char*)&cmd, 1);
}

void sendData(const uint8_t* data, size_t len) {
    gpioWrite(PIN_DC, 1);
    spiWrite(spi, (char*)data, len);
}

// -----------------------------------------------------------------------
//  Display init / update
// -----------------------------------------------------------------------
void initDisplay() {
    gpioWrite(PIN_RES, 0); usleep(100000);
    gpioWrite(PIN_RES, 1); usleep(100000);

    sendCommand(0xAE);
    sendCommand(0xD5); sendCommand(0x80);
    sendCommand(0xA8); sendCommand(0x3F);
    sendCommand(0xD3); sendCommand(0x00);
    sendCommand(0x40);
    sendCommand(0xAD); sendCommand(0x8B);
    sendCommand(0xA1);
    sendCommand(0xC8);
    sendCommand(0xDA); sendCommand(0x12);
    sendCommand(0x81); sendCommand(0xCF);
    sendCommand(0xD9); sendCommand(0xF1);
    sendCommand(0xDB); sendCommand(0x40);
    sendCommand(0xA4);
    sendCommand(0xA6);
    sendCommand(0xAF);
}

void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));
}

void clearScreen() {
    uint8_t empty[132];
    memset(empty, 0x00, sizeof(empty));
    for (int page = 0; page < 8; page++) {
        sendCommand(0xB0 + page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(empty, 132);
    }
}

void updateDisplay() {
    for (int page = 0; page < HEIGHT / 8; page++) {
        sendCommand(0xB0 + page);
        sendCommand(0x00);
        sendCommand(0x10);
        sendData(&buffer[page * WIDTH], WIDTH);
    }
}

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y / 8) * WIDTH] |= (1 << (y % 8));
}

// -----------------------------------------------------------------------
//  Glyphs  (5×7 column-major bitmaps, LSB = top row)
// -----------------------------------------------------------------------
static const uint8_t GLYPH_P[5]    = { 0x7F, 0x09, 0x09, 0x09, 0x06 };
static const uint8_t GLYPH_X[5]    = { 0x63, 0x14, 0x08, 0x14, 0x63 };
static const uint8_t GLYPH_DASH[5] = { 0x08, 0x08, 0x08, 0x08, 0x08 };
static const uint8_t GLYPH_1[5]    = { 0x00, 0x42, 0x7F, 0x40, 0x00 };
static const uint8_t GLYPH_C[5]    = { 0x3E, 0x41, 0x41, 0x41, 0x22 };
static const uint8_t GLYPH_D[5]    = { 0x7F, 0x41, 0x41, 0x22, 0x1C };
static const uint8_t GLYPH_E[5]    = { 0x7F, 0x49, 0x49, 0x49, 0x41 };
static const uint8_t GLYPH_M[5]    = { 0x7F, 0x02, 0x0C, 0x02, 0x7F };
static const uint8_t GLYPH_N[5]    = { 0x7F, 0x04, 0x08, 0x10, 0x7F };
static const uint8_t GLYPH_O[5]    = { 0x3E, 0x41, 0x41, 0x41, 0x3E };
static const uint8_t GLYPH_R[5]    = { 0x7F, 0x09, 0x19, 0x29, 0x46 };

void drawBigGlyph(int x, int y, const uint8_t* glyph, int S) {
    for (int col = 0; col < 5; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1 << row)) {
                for (int dx = 0; dx < S; dx++)
                    for (int dy = 0; dy < S; dy++)
                        drawPixel(x + col * S + dx, y + row * S + dy);
            }
        }
    }
}

void drawPX1(int centerX, int topY) {
    const int S   = 3;
    const int GW  = 5 * S;
    const int GAP = S;

    const uint8_t* glyphs[] = { GLYPH_P, GLYPH_X, GLYPH_DASH, GLYPH_1 };
    const int n = 4;

    int totalW = n * GW + (n - 1) * GAP;
    int startX = centerX - totalW / 2;

    for (int i = 0; i < n; i++)
        drawBigGlyph(startX + i * (GW + GAP), topY, glyphs[i], S);
}

void drawLine(int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        drawPixel(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void drawRect(int x, int y, int w, int h) {
    for (int i = 0; i < w; i++) {
        drawPixel(x + i, y);
        drawPixel(x + i, y + h - 1);
    }
    for (int i = 0; i < h; i++) {
        drawPixel(x, y + i);
        drawPixel(x + w - 1, y + i);
    }
}

void drawArc(int cx, int cy, int radius, float startDeg, float endDeg, int thickness) {
    const float pi = 3.14159265f;
    for (int t = 0; t < thickness; t++) {
        int r = radius - t;
        for (float deg = startDeg; deg <= endDeg; deg += 1.5f) {
            float rad = deg * pi / 180.0f;
            int x = cx + int(std::cos(rad) * r);
            int y = cy + int(std::sin(rad) * r);
            drawPixel(x, y);
        }
    }
}

bool loadPngSprite(const char* filename, MonoSprite& sprite) {
    FILE* fp = std::fopen(filename, "rb");
    if (!fp) {
        std::cerr << "Failed to open " << filename << "\n";
        return false;
    }

    png_byte header[8];
    if (std::fread(header, 1, sizeof(header), fp) != sizeof(header) ||
        png_sig_cmp(header, 0, sizeof(header))) {
        std::fclose(fp);
        std::cerr << "Invalid PNG: " << filename << "\n";
        return false;
    }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        std::fclose(fp);
        return false;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_read_struct(&png, nullptr, nullptr);
        std::fclose(fp);
        return false;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_read_struct(&png, &info, nullptr);
        std::fclose(fp);
        std::cerr << "Failed reading PNG: " << filename << "\n";
        return false;
    }

    png_init_io(png, fp);
    png_set_sig_bytes(png, sizeof(header));
    png_read_info(png, info);

    int width = static_cast<int>(png_get_image_width(png, info));
    int height = static_cast<int>(png_get_image_height(png, info));
    png_byte colorType = png_get_color_type(png, info);
    png_byte bitDepth = png_get_bit_depth(png, info);

    if (bitDepth == 16) png_set_strip_16(png);
    if (colorType == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if (colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8) png_set_expand_gray_1_2_4_to_8(png);
    if (png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
    if (colorType == PNG_COLOR_TYPE_RGB ||
        colorType == PNG_COLOR_TYPE_GRAY ||
        colorType == PNG_COLOR_TYPE_PALETTE) {
        png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
    }
    if (colorType == PNG_COLOR_TYPE_GRAY || colorType == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png);
    }

    png_read_update_info(png, info);

    std::vector<uint8_t> rgba(width * height * 4);
    std::vector<png_bytep> rows(height);
    for (int y = 0; y < height; y++) {
        rows[y] = reinterpret_cast<png_bytep>(rgba.data() + y * width * 4);
    }
    png_read_image(png, rows.data());

    png_destroy_read_struct(&png, &info, nullptr);
    std::fclose(fp);

    sprite.width = width;
    sprite.height = height;
    sprite.minX = width;
    sprite.minY = height;
    sprite.maxX = -1;
    sprite.maxY = -1;
    sprite.loaded = false;
    sprite.mask.assign(width * height, 0);

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int idx = (y * width + x) * 4;
            uint8_t r = rgba[idx + 0];
            uint8_t g = rgba[idx + 1];
            uint8_t b = rgba[idx + 2];
            uint8_t a = rgba[idx + 3];
            int luminance = (299 * r + 587 * g + 114 * b) / 1000;
            bool isOn = a > 24 && luminance < 220;

            if (!isOn) continue;

            sprite.mask[y * width + x] = 1;
            sprite.minX = std::min(sprite.minX, x);
            sprite.minY = std::min(sprite.minY, y);
            sprite.maxX = std::max(sprite.maxX, x);
            sprite.maxY = std::max(sprite.maxY, y);
        }
    }

    sprite.loaded = sprite.maxX >= sprite.minX && sprite.maxY >= sprite.minY;
    if (!sprite.loaded) {
        std::cerr << "PNG sprite had no visible pixels: " << filename << "\n";
    }

    return sprite.loaded;
}

void drawLoadedSpriteCentered(const MonoSprite& sprite, int centerX, int topY, int bottomY) {
    if (!sprite.loaded) return;

    int spriteWidth = sprite.maxX - sprite.minX + 1;
    int spriteHeight = sprite.maxY - sprite.minY + 1;
    int availableHeight = std::max(0, bottomY - topY);
    int startX = centerX - spriteWidth / 2;
    int startY = topY + std::max(0, (availableHeight - spriteHeight) / 2);

    for (int y = 0; y < spriteHeight; y++) {
        for (int x = 0; x < spriteWidth; x++) {
            int srcX = sprite.minX + x;
            int srcY = sprite.minY + y;
            if (sprite.mask[srcY * sprite.width + srcX]) {
                drawPixel(startX + x, startY + y);
            }
        }
    }
}

void drawRecommendedText(int centerX, int y) {
    const uint8_t* glyphs[] = {
        GLYPH_R, GLYPH_E, GLYPH_C, GLYPH_O, GLYPH_M, GLYPH_M,
        GLYPH_E, GLYPH_N, GLYPH_D, GLYPH_E, GLYPH_D
    };
    const int glyphCount = sizeof(glyphs) / sizeof(glyphs[0]);
    const int scale = 1;
    const int glyphWidth = 5 * scale;
    const int gap = 1;
    int totalWidth = glyphCount * glyphWidth + (glyphCount - 1) * gap;
    int startX = centerX - totalWidth / 2;

    for (int i = 0; i < glyphCount; i++) {
        drawBigGlyph(startX + i * (glyphWidth + gap), y, glyphs[i], scale);
    }
}

void drawHeadphonesPanel() {
    const int centerX = WIDTH / 2;
    const int spriteTop = 2;
    const int textY = 54;

    if (headphonesSprite.loaded) {
        drawLoadedSpriteCentered(headphonesSprite, centerX, spriteTop, textY - 4);
    }

    drawRecommendedText(centerX, textY);
}

int randomRangeMs(int minMs, int maxMs) {
    return minMs + (std::rand() % (maxMs - minMs + 1));
}

// -----------------------------------------------------------------------
//  Bouncing dots
// -----------------------------------------------------------------------
void drawFilledCircle(int cx, int cy, int r) {
    for (int px = -r; px <= r; px++) {
        int half = (int)sqrtf((float)(r * r - px * px));
        for (int py = -half; py <= half; py++)
            drawPixel(cx + px, cy + py);
    }
}

// -----------------------------------------------------------------------
//  Graceful exit — clear screen on Ctrl-C or kill
// -----------------------------------------------------------------------
volatile sig_atomic_t running = 1;

void onSignal(int) {
    running = 0;
}

// -----------------------------------------------------------------------
//  Main
// -----------------------------------------------------------------------
int main() {
    std::srand(static_cast<unsigned>(std::time(nullptr)) ^ static_cast<unsigned>(getpid()));

    loadPngSprite("headphones.png", headphonesSprite);

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    signal(SIGINT,  onSignal);
    signal(SIGTERM, onSignal);

    gpioSetMode(PIN_DC,  PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    spi = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (spi < 0) {
        std::cerr << "SPI open failed\n";
        gpioTerminate();
        return 1;
    }

    initDisplay();
    clearBuffer();
    clearScreen();
    updateDisplay();

    // Animation state
    float phases[3] = { 0.0f, 2.094f, 4.189f };  // 120° apart
    const float SPEED   = 0.12f;
    const int   DOT_R   = 3;
    const int   DOT_GAP = 12;
    const int   BOUNCE  = 5;
    const int   BASE_Y  = 52;
    const int   CX      = 63;

    const int dotX[3] = { CX - DOT_GAP, CX, CX + DOT_GAP };
    bool showHeadphones = false;
    auto nextFlashToggle =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(randomRangeMs(3000, 5000));

    while (running) {
        auto now = std::chrono::steady_clock::now();
        if (now >= nextFlashToggle) {
            showHeadphones = !showHeadphones;
            int delayMs = showHeadphones ? randomRangeMs(2000, 3000) : randomRangeMs(3000, 5000);
            nextFlashToggle = now + std::chrono::milliseconds(delayMs);
        }

        clearBuffer();

        if (showHeadphones) {
            drawHeadphonesPanel();
        } else {
            drawPX1(CX, 10);

            for (int d = 0; d < 3; d++) {
                phases[d] += SPEED;
                int cy = BASE_Y - (int)(sinf(phases[d]) * BOUNCE);
                drawFilledCircle(dotX[d], cy, DOT_R);
            }
        }

        updateDisplay();
        usleep(33000);  // ~30 fps — light on CPU during your compile
    }

    // Clean exit
    clearBuffer();
    clearScreen();
    updateDisplay();
    spiClose(spi);
    gpioTerminate();
    return 0;
}

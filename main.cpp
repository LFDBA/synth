#include <pigpio.h>
#include <cstring>
#include <string>
#include <vector>
#include <iostream>

// OLED buffer size
const int WIDTH = 128;
const int HEIGHT = 64;
uint8_t buffer[WIDTH * (HEIGHT / 8)];
int global_spi_handle = -1;

// Pins
#define PIN_DC 25
#define PIN_RES 24

// --------------------------------------
// OLED Helpers
// --------------------------------------
void clearBuffer() {
    memset(buffer, 0x00, sizeof(buffer));
}

void drawPixel(int x, int y) {
    if(x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] |= (1 << (y % 8));
}

void drawRect(int x, int y, int w, int h, bool fill=false) {
    if(fill){
        for(int i=0;i<w;i++)
            for(int j=0;j<h;j++)
                drawPixel(x+i, y+j);
    }else{
        for(int i=0;i<w;i++){ drawPixel(x+i,y); drawPixel(x+i,y+h-1); }
        for(int j=0;j<h;j++){ drawPixel(x,y+j); drawPixel(x+w-1,y+j); }
    }
}

// Bresenham line
void drawLine(int x0,int y0,int x1,int y1){
    int dx=abs(x1-x0), sx=x0<x1?1:-1;
    int dy=-abs(y1-y0), sy=y0<y1?1:-1;
    int err=dx+dy;
    for(;;){
        drawPixel(x0,y0);
        if(x0==x1 && y0==y1) break;
        int e2=err*2;
        if(e2>=dy){ err+=dy; x0+=sx; }
        if(e2<=dx){ err+=dx; y0+=sy; }
    }
}

// Fake "text" rendering using 5x7 pixel font
void drawChar(int x, int y, char c) {
    // Minimal font example: only uppercase letters and space
    static const uint8_t font[][5] = {
        {0x00,0x00,0x00,0x00,0x00}, // space
        {0x7C,0x12,0x11,0x12,0x7C}, // A
        {0x7F,0x49,0x49,0x49,0x36}, // B
        {0x3E,0x41,0x41,0x41,0x22}, // C
        {0x7F,0x41,0x41,0x22,0x1C}, // D
        {0x7F,0x49,0x49,0x49,0x41}, // E
        {0x7F,0x09,0x09,0x09,0x01}, // F
        {0x3E,0x41,0x49,0x49,0x7A}, // G
        // ... add more letters as needed
    };

    int idx = 0;
    if(c == ' ') idx=0;
    else if(c >= 'A' && c <= 'G') idx = c-'A'+1; // only A-G for demo

    for(int i=0;i<5;i++){
        uint8_t col = font[idx][i];
        for(int j=0;j<7;j++){
            if(col & (1<<j)) drawPixel(x+i, y+j);
        }
    }
}

void drawText(int x,int y,const std::string &s){
    for(size_t i=0;i<s.size();i++){
        drawChar(x + i*6, y, s[i]);
    }
}

// Send buffer to OLED
void sendCommand(int spi, uint8_t cmd) {
    gpioWrite(PIN_DC,0);
    spiWrite(spi,(char*)&cmd,1);
}

void sendData(int spi, const uint8_t* data, size_t len){
    gpioWrite(PIN_DC,1);
    spiWrite(spi,(char*)data,len);
}

void updateDisplay(int spi){
    for(int page=0;page<HEIGHT/8;page++){
        sendCommand(spi,0xB0+page);
        sendCommand(spi,0x00);
        sendCommand(spi,0x10);
        sendData(spi, &buffer[page*WIDTH], WIDTH);
    }
}

// --------------------------------------
// Menu Drawing
// --------------------------------------
void drawMenu(const std::vector<std::string> &items, int selected){
    clearBuffer();
    int boxHeight = HEIGHT / items.size();

    for(size_t i=0;i<items.size();i++){
        int y = i*boxHeight;
        bool highlight = (i==selected);
        drawRect(2, y+2, WIDTH-4, boxHeight-4, highlight);
        drawText(6, y + boxHeight/2 - 3, items[i]);
    }

    updateDisplay(global_spi_handle);
}

// --------------------------------------
// MAIN
// --------------------------------------
int main(){
    if(gpioInitialise()<0){ std::cerr<<"pigpio init failed\n"; return 1; }

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    global_spi_handle = spiOpen(0, 8000000, 0);
    if(global_spi_handle<0){ std::cerr<<"SPI failed\n"; return 1; }

    // reset OLED
    gpioWrite(PIN_RES,0);
    gpioDelay(100000);
    gpioWrite(PIN_RES,1);
    gpioDelay(100000);

    // Demo menu
    std::vector<std::string> menuItems = {"TONE", "VOICE", "ADSR", "REVERB"};
    int selected = 0;

    while(true){
        std::cout << "huh";
        drawMenu(menuItems, selected);
        std::cout << "out";
        selected = (selected + 1) % menuItems.size();
        gpioDelay(500000); // half second delay for demo
    }

    return 0;
}

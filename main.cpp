#include <iostream>
#include <cmath>
#include <vector>
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
#include <ncurses.h>  // For proper non-blocking keyboard input
#include <pigpio.h>
#include <cstdint>
#include <cstring>
#include <signal.h>





// ======================================================
//                     Screen Setup
// ======================================================
#define SPI_CHANNEL 0          // CE0
#define SPI_SPEED 8000000      // 8 MHz
#define PIN_DC 25              // Data/Command pin
#define PIN_RES 24             // Reset pin

const int WIDTH = 128;
const int HEIGHT = 64;

uint8_t buffer[WIDTH * (HEIGHT / 8)];
int global_spi_handle = -1;   // Needed for safe exit

void clearBuffer();
void updateDisplay(int spi);
void gracefulExit(int signum);

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

void drawPixel(int x, int y) {
    if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) return;
    buffer[x + (y/8)*WIDTH] |= (1 << (y % 8));
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

// ======================================================
//                     Constants
// ======================================================
const float sampleRate = 48000.0f;
const int numVoices = 7;
float outputLevel = 0.1f;
bool normVoices = true; // Normalize by active voices
float pan = 0.0f;
int fd;
int editIndex = 0;
bool edit = false;

Reverb reverb(sampleRate);

// Menus
enum Mode {
    MODE_NONE,
    VOICE_TONE_MENU,
    TONE_MENU,
    WAVE_MENU,
    ADSR_MENU,
    REVERB_MENU
};
Mode menu = TONE_MENU;

// Input device variables
int p1, p2, p3, p4;
int lastP1=-1, lastP2, lastP3, lastP4;
float knobPosition = 0.9f;

// Waveform editor
const int WAVE_RES = 12;
float wavePoints[WAVE_RES];
static std::vector<float> customTable;
static bool waveNeedsRebuild = true;
const int TABLE_SIZE = 8192;
float curvature = 1.0f;

// ======================================================
//                     Voice Struct
// ======================================================
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;        // key held
    bool releasing = false;     // is in release phase
    float envTime = 0.0f;       // time since note-on or release start
    float oscVolume = 1.0f;
};


Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60, 64, 67, 72}; // MIDI notes
bool custom = false;

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
    if(x > 1.0f) return 1.0f - expf(-x);   // smooth limit
    if(x < -1.0f) return -1.0f + expf(x);  // smooth limit
    return x;
}

// ======================================================
//                     Custom Wave (Fixed-Point Editor)
// ======================================================
inline float lerp(float a, float b, float t) { return a + (b-a)*t; }
inline float curveInterp(float a, float b, float t, float curv) {
    if(curv!=1.0f) t=powf(t,curv);
    return lerp(a,b,t);
}

void rebuildWaveTable() {
    for(int i=0;i<TABLE_SIZE;i++){
        float t = float(i)/(TABLE_SIZE-1);
        float idxF = t*(WAVE_RES-1);
        int idx = int(idxF);
        float frac = idxF-idx;
        float v1 = wavePoints[idx];
        float v2 = wavePoints[std::min(idx+1,WAVE_RES-1)];
        customTable[i] = curveInterp(v1,v2,frac,curvature);
    }
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
    return fC0*pow(2.0f,float(noteNumber)/12.0f);
}

// ======================================================
//                  ADSR Envelope
// ======================================================
float ADSR(float attack,float decay,float sustain,float release,bool trig,float t,float lvl){
    float curvature=3.0f;
    if(trig){
        if(t<attack) return powf(t/attack,curvature)*lvl;
        else if(t<attack+decay) return (1.0f - powf((t-attack)/decay,1.0f/curvature)*(1.0f-sustain))*lvl;
        else return sustain*lvl;
    }else{
        if(t<release) return (1.0f - powf(t/release,1.0f/curvature))*(sustain*lvl);
        else return 0.0f;
    }
}


// ======================================
//         Output Waveform Draw
// ======================================

const int DRAW_WIDTH = WIDTH;       // OLED width
const int BUF_LEN = 10;            // Number of samples to keep for display
float sampleBuffer[BUF_LEN];        // circular buffer
int bufIndex = 0;                   // current write position

// Push new sample into circular buffer
void pushSample(float s) {
    sampleBuffer[bufIndex++] = s;
    if(bufIndex >= BUF_LEN) bufIndex = 0;
}

// Helper for mapping integers
inline int iMap(int val,int inMin,int inMax,int outMin,int outMax){
    return (val - inMin)*(outMax - outMin)/(inMax - inMin) + outMin;
}

// Draw waveform to OLED
void drawOutput() {
    clearBuffer();

    // Find max absolute value for normalization
    float maxVal = 0.0001f; // avoid division by zero
    for(int i=0;i<BUF_LEN;i++)
        if(fabs(sampleBuffer[i]) > maxVal) maxVal = fabs(sampleBuffer[i]);

    // Draw line across width
    for(int x=0;x<DRAW_WIDTH;x++){
        // Map x to buffer index
        int idx = (bufIndex + iMap(x,0,DRAW_WIDTH,0,BUF_LEN)) % BUF_LEN;
        // Normalize to -1..1
        float normSample = sampleBuffer[idx] / maxVal;
        // Map to pixel
        int y = HEIGHT/2 - int(normSample * (HEIGHT/2 - 1));

        // Draw line from previous point
        if(x>0){
            int prevIdx = (bufIndex + iMap(x-1,0,DRAW_WIDTH,0,BUF_LEN)) % BUF_LEN;
            float prevSample = sampleBuffer[prevIdx] / maxVal;
            int y0 = HEIGHT/2 - int(prevSample * (HEIGHT/2 - 1));
            drawLine(x-1, y0, x, y);
        }
    }
}





// ======================================================
//                  Audio Callback
// ======================================================
int audioCallback(void *outputBuffer, void* /*inputBuffer*/, unsigned int nBufferFrames,
                  double /*streamTime*/, RtAudioStreamStatus /*status*/, void* /*userData*/) 
{
    float *output = static_cast<float*>(outputBuffer);

    if(custom && waveNeedsRebuild) rebuildWaveTable();

    for(unsigned int i=0;i<nBufferFrames;i++){
        float mix=0.0f;
        int activeVoices=0;

        for(int v=0;v<numVoices;v++){
            Voice &voice = voices[v];
            float sample = 0.0f;

            if(voice.active || voice.releasing){
                activeVoices++;

                // Increment envelope time
                voice.envTime += 1.0f/sampleRate;

                // Compute ADSR
                float env = ADSR(attack, decay, sustain, release, voice.active, voice.envTime, voice.oscVolume);

                // Oscillator
                float oscSample;
                if(custom){
                    int idx=int(voice.phase*TABLE_SIZE);
                    if(idx>=TABLE_SIZE) idx=TABLE_SIZE-1;
                    oscSample = customTable[idx];
                }else{
                    float seg=knobPosition*4.0f;
                    int idx=int(seg);
                    float blend=seg-idx;
                    float w1,w2;
                    switch(idx){
                        case 0: w1=sineWave(voice.phase); w2=squareWave(voice.phase); break;
                        case 1: w1=squareWave(voice.phase); w2=sawWave(voice.phase); break;
                        case 2: w1=sawWave(voice.phase); w2=triangleWave(voice.phase); break;
                        case 3: w1=triangleWave(voice.phase); w2=sineWave(voice.phase); break;
                        default: w1=w2=0.0f;
                    }
                    oscSample = ((1.0f-blend)*w1 + blend*w2);
                }

                sample = oscSample * env;

                // Increment phase
                voice.phase += voice.frequency/sampleRate;
                if(voice.phase >= 1.0f) voice.phase -= 1.0f;

                // Handle end of release
                if(voice.releasing && voice.envTime >= release){
                    voice.releasing = false;
                    voice.envTime = 0.0f;
                    voice.phase = 0.0f;
                }
            }
 
            mix += sample;
        }

        // Normalize
        if(normVoices && activeVoices>0) mix; ///= (activeVoices*0.2f);

        mix = softClip(mix * outputLevel);
        mix = reverb.process(mix);

        pushSample(mix);
        

        output[2*i]     = mix*(1.0f-pan);
        output[2*i + 1] = mix*(pan+1.0f);
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

// ======================================================
//               Read Input Device
// ======================================================
bool initSerial(const char* port="/dev/ttyACM1") {
    fd=open(port,O_RDONLY|O_NOCTTY);
    if(fd<0){ std::cerr<<"Failed to open serial port\n"; return false; }

    termios tty{};
    if(tcgetattr(fd,&tty)!=0){ std::cerr<<"tcgetattr failed\n"; return false; }

    cfsetospeed(&tty,B115200);
    cfsetispeed(&tty,B115200);
    tty.c_cflag|=(CLOCAL|CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ISIG);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]=1;
    tty.c_cc[VTIME]=0;
    tcsetattr(fd,TCSANOW,&tty);

    return true;
}

void getInp() {
    static std::string line="";
    char buf[64];
    int n=read(fd,buf,sizeof(buf));
    if(n>0){
        for(int i=0;i<n;i++){
            char c=buf[i];
            if(c=='\n'){
                if(!line.empty()){
                    std::stringstream ss(line);
                    std::string label;
                    int value;
                    ss >> label >> value;
                    if(label=="p1") p1=(-value)+1023;
                    else if(label=="p2") p2=(-value)+1023;
                    else if(label=="p3") p3=(-value)+1023;
                    else if(label=="p4") p4=(-value)+1023;
                }
                line.clear();
            }else if(c!='\r') line+=c;
        }
    }
}

// ======================================================
//                     Wave Edit
// ======================================================
void editWave(){
    editIndex = static_cast<int>(norm(p1,0.0f,1023.0f,0.0f,WAVE_RES-1));
    if(abs(p2-lastP2)>1){
        wavePoints[editIndex] = norm(p2,0.0f,1023.0f,-2.0f,2.0f);
        waveNeedsRebuild=true;
    }
    if(abs(p3-lastP3)>1) curvature = norm(p3,0.0f,1023.0f,0.1f,5.0f);
    knobPosition = norm(p4,0.0f,1023.0f,0.0f,0.9f);
    if(abs(p4-lastP4)>2) custom=false;
    if(abs(p1-lastP1)>3 || abs(p2-lastP2)>3 || abs(p3-lastP3)>3) custom=true;
    updateWave();
}

// ======================================================
//                     ADSR Edit
// ======================================================
void editADSR(){
    if(abs(p1-lastP1)>1) attack = norm(p1,0.0f,1023.0f,0.0f,5.0f);
    if(abs(p2-lastP2)>1) decay = norm(p2,0.0f,1023.0f,0.0f,5.0f);
    if(abs(p3-lastP3)>1) sustain = norm(p3,0.0f,1023.0f,0.0f,1.0f);
    if(abs(p4-lastP4)>1) release = norm(p4,0.0f,1023.0f,0.0f,5.0f);
}

// ======================================================
//                   Reverb Edit
// ======================================================
void editReverb() {
    if(abs(p1-lastP1)>1){
        float dry = norm(p1,0.0f,1023.0f,0.0f,1.0f);
        float wet = 1.0f - dry;
        reverb.setDryWet(wet,dry);
    }
    if(abs(p2-lastP2)>1){
        float size = norm(p2,0.0f,1023.0f,0.1f,1.5f);
        reverb.setRoomSize(size);
    }
    if(abs(p3-lastP3)>1){
        float decay = norm(p3,0.0f,1023.0f,0.1f,1.0f);
        reverb.setDecay(decay);
    }
}

// ======================================================
//                     Tone Edit
// ======================================================
void editTone(){
    if(abs(p1-lastP1)>1) outputLevel = norm(p1,0.0f,1023.0f,0.0f,0.1f);
    if(abs(p2-lastP2)>1) pan = norm(p2,0.0f,1023.0f,-1.0f,1.0f);
}




void drawADSR() {
    clearBuffer();

    float sustainView = 0.2;
    float totalTime = attack + decay + sustainView + release;

    int lastY = -1;
    for (int x = 0; x < WIDTH; x++) {
        float t = (float)x / WIDTH * totalTime;

        float env;
        if (t < attack + decay + sustainView)
            env = ADSR(attack, decay, sustain, release, true, t, 1.0);
        else
            env = ADSR(attack, decay, sustain, release, false,
                       t - (attack + decay + sustainView),
                       1.0);

        int y = HEIGHT - 1 - int(env * (HEIGHT - 1));
        if (lastY >= 0) drawLine(x-1, lastY, x, y);
        lastY = y;
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
}







// ======================================================
//                        MAIN
// ======================================================
int main() {
    if(!initSerial()){
        try{
            initSerial("/dev/ttyACM0");
        }catch(...){
            std::cerr<<"Failed to open serial port\n";
            return 1;
        }
    };

    reverb.mode = ReverbType::SCHROEDER;
    reverb.setDryWet(1.0f,0.0f);
    reverb.setRoomSize(10.0f);
    reverb.setDecay(0.9f);

    initWavePoints();
    initKeyboard();


    if (gpioInitialise() < 0) {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    signal(SIGINT, gracefulExit);

    gpioSetMode(PIN_DC, PI_OUTPUT);
    gpioSetMode(PIN_RES, PI_OUTPUT);

    global_spi_handle = spiOpen(SPI_CHANNEL, SPI_SPEED, 0);
    if (global_spi_handle < 0) {
        std::cerr << "SPI failed\n";
        return 1;
    }

    initDisplay(global_spi_handle);



    // RtAudio setup
    RtAudio dac;
    RtAudio::StreamParameters oParams;
    oParams.deviceId = dac.getDefaultOutputDevice();
    oParams.nChannels = 2;
    unsigned int bufferFrames = 256;

    try {
        dac.openStream(&oParams,nullptr,RTAUDIO_FLOAT32,
                       sampleRate,&bufferFrames,&audioCallback);
        dac.startStream();
    } catch(RtAudioError &e){
        e.printMessage();
        return 1;
    }

    std::cout << "Polyphonic Synth Ready.\n";
    std::cout << "Press keys z,x,c,v to trigger voices, 1–3 for menus.\n";

    while(true){
        getInp(); // microcontroller input

        updateDisplay(global_spi_handle);

        if(lastP1==-1){ lastP1=p1; lastP2=p2; lastP3=p3; lastP4=p4; }

        // menu edits
        
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
        if(menu==REVERB_MENU && edit) editReverb();
    
        

        // Keyboard triggering
        int key = getKeyPress();
        if(key != -1){
            switch(key){
                case '0': edit = !edit; break;
                case '1': menu=WAVE_MENU; break;
                case '2': menu=ADSR_MENU; break;
                case '3': menu=REVERB_MENU; break;
                case '4': menu=TONE_MENU; break;
                case 'z': case 'x': case 'c': case 'v': {
                    int v = (key=='z')?0:(key=='x')?1:(key=='c')?2:3;
                    Voice &voice = voices[v];

                    if(voice.active){
                        // Start release
                        voice.active = false;
                        voice.releasing = true;
                        voice.envTime = 0.0f; // release phase timer
                    } else {
                        // Start note
                        voice.active = true;
                        voice.releasing = false;
                        voice.envTime = 0.0f;
                        voice.phase = 0.0f;
                        voice.frequency = noteToHz(noteMapping[v]);
                    }
                    break;
                }

            }
        }

        usleep(1000);
        lastP1=p1; lastP2=p2; lastP3=p3; lastP4=p4;
    }

    try{ dac.stopStream(); } catch(RtAudioError &e){}
    if(dac.isStreamOpen()) dac.closeStream();
    closeKeyboard();

    return 0;
}





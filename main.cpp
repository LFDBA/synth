#include <iostream>
#include <cmath>
#include <portaudio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <array>
#include <linux/input.h>
#include <algorithm>
#include <sstream>
#include <string>
#include "Reverb.h"

float norm(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// === Constants ===
const float sampleRate = 48000.0f;
const int numVoices = 7;
const char* device = "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00";
float outputLevel = 1f;
bool normVoices = true; // Normalize by active voices
float pan = 0.0f;

Reverb reverb(sampleRate);

enum Mode {
    MODE_NONE,
    VOICE_TONE_MENU,
    TONE_MENU,
    WAVE_MENU,
    ADSR_MENU,
    REVERB_MENU
};
Mode menu = TONE_MENU;

int p1, p2, p3, p4;
float curvature = 1.0f;  // Curvature for waveform interpolation

// --- Note to Hz ---
float noteToHz(int noteNumber) {
    float fC0 = 16.35f;
    return fC0 * pow(2.0f, float(noteNumber - 12) / 12.0f);
}

// --- Basic Oscillators ---
float sineWave(float phase)      { return sinf(2.0f * M_PI * phase); }
float squareWave(float phase)    { return (phase < 0.5f) ? 1.0f : -1.0f; }
float sawWave(float phase)       { return 2.0f * (phase - 0.5f); }
float triangleWave(float phase)  { return 4.0f * fabsf(phase - 0.5f) - 1.0f; }

// --- Morph knob mapping ---
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

// ======================================================
//                     Custom Wave (Fixed-Point Editor)
// ======================================================
const int WAVE_RES = 12;        // Number of editable points
float wavePoints[WAVE_RES];      // Editable waveform points
int editIndex = 0;               // Currently edited point

static std::vector<float> customTable;
static bool waveNeedsRebuild = true;
const int TABLE_SIZE = 8192;

// --- Fast linear interpolation ---
inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// --- Curvature interpolation ---
inline float curveInterp(float a, float b, float t, float curv) {
    if(curv != 1.0f) t = powf(t, curv);  // >1 = starts flat, <1 = starts sharp
    return lerp(a, b, t);
}

// --- Rebuild wavetable from fixed points with curvature ---
void rebuildWaveTable() {
    for(int i = 0; i < TABLE_SIZE; i++) {
        float t = float(i) / float(TABLE_SIZE - 1);
        float idxF = t * (WAVE_RES - 1);
        int idx = int(idxF);
        float frac = idxF - idx;
        float v1 = wavePoints[idx];
        float v2 = wavePoints[std::min(idx+1, WAVE_RES-1)];
        customTable[i] = curveInterp(v1, v2, frac, curvature); // Apply curvature
    }
    waveNeedsRebuild = false;
}

void updateWave() {
    waveNeedsRebuild = true;
}

// --- Initialize waveform points ---
void initWavePoints() {
    for(int i = 0; i < WAVE_RES; i++) {
        wavePoints[i] = norm(i, 0, WAVE_RES-1, -2.0f, 2.0f);
    }
    customTable.resize(TABLE_SIZE);
    waveNeedsRebuild = true;
}

// =================================================
//                  Voice Struct
// =================================================
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;
    float time = 0.0f;
    float oscVolume = 0.04f;
};

float ADSR(float attack, float decay, float sustain, float release, bool trig, float t, float lvl) {
    bool noteOn = trig;
    float curvature = 3.0f;

    if(noteOn) {
        if(t < attack) {
            return powf(t / attack, curvature) * lvl;
        } else if(t < attack + decay) {
            return (1.0f - (powf((t - attack) / decay, 1.0f/curvature) * (1.0f - sustain))) * lvl;
        } else {
            return sustain * lvl;
        }
    } else {
        if(t < release) {
            return (1.0f - powf(t / release, 1.0f/curvature)) * (sustain * lvl);
        } else {
            return 0.0f;
        }
    }
}

// =================================================
//                  Global Vars
// =================================================
float knobPosition = 0.9f;
Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60};
bool custom = false;
struct input_event ev;
int fd = -1;
float inpVal = 0;
float sweepPos = 0;
int inpMode = -1;
float attack = 0.5f;
float decay = 0.5f;
float sustain = 0.8f;
float release = 0.0f;

// =================================================
//                  Audio Callback
// =================================================
static int audioCallback(
    const void*, void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo*,
    PaStreamCallbackFlags,
    void*)
{
    float* output = (float*)outputBuffer;
    float t = (knobPosition);
    int activeVoices = 0;

    if (custom && waveNeedsRebuild)
        rebuildWaveTable();

    for(unsigned long i=0; i<framesPerBuffer; i++) {
        float mix = 0.0f;

        for(int v=0; v<numVoices; v++) {
            if(!voices[v].active) continue;
            activeVoices++;
            voices[v].time += 1.0f / sampleRate;
            voices[v].phase += voices[v].frequency / sampleRate;
            if(voices[v].phase >= 1.0f) voices[v].phase -= 1.0f;

            float adsr = ADSR(attack, decay, sustain, release, voices[v].active, voices[v].time, voices[v].oscVolume);
            float sample;

            if(custom) {
                int index = static_cast<int>(voices[v].phase * TABLE_SIZE);
                if(index >= TABLE_SIZE) index = TABLE_SIZE-1;
                sample = customTable[index] * adsr;

            } else {
                float segment = t * 4.0f;
                int idx = (int)floor(segment);
                float blend = segment - idx;

                float wave1, wave2;
                switch(idx) {
                    case 0: wave1 = sineWave(voices[v].phase);     wave2 = squareWave(voices[v].phase); break;
                    case 1: wave1 = squareWave(voices[v].phase);   wave2 = sawWave(voices[v].phase);    break;
                    case 2: wave1 = sawWave(voices[v].phase);      wave2 = triangleWave(voices[v].phase); break;
                    case 3: wave1 = triangleWave(voices[v].phase); wave2 = sineWave(voices[v].phase);   break;
                    default: wave1 = wave2 = 0.0f;
                }

                sample = ((1.0f - blend) * wave1 + blend * wave2) * adsr;
            }

            mix += sample;
        }
        //maybe add clamp later
        //mix = std::clamp(mix, -1.0f, 1.0f);

        mix = mix * outputLevel; // Normalize
        if(normVoices) mix = mix / activeVoices;
        mix = reverb.process(mix);

        output[2*i]     = mix * (1.0f-pan);
        output[2*i + 1] = mix * (pan+1.0f);
    }

    return 0;
}

// =================================================
//         Non-blocking keyboard input
// =================================================
void setNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~ICANON;
    ttystate.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

// =================================================
//               Read Input Device
// =================================================
std::string line = "";
int lastP1=-1, lastP2, lastP3, lastP4;

bool initSerial(const char* port = "/dev/ttyACM0") {
    fd = open(port, O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Failed to open serial port\n";
        return false;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error: tcgetattr failed\n";
        return false;
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

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &tty);
    return true;
}

void getInp() {
    char buf[64];
    int n = read(fd, buf, sizeof(buf));
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            char c = buf[i];

            if (c == '\n') {
                if (!line.empty()) {
                    std::stringstream ss(line);
                    std::string label;
                    int value;
                    ss >> label >> value;

                    if (label == "p1") p1 = (-value)+1023;
                    else if (label == "p2") p2 = (-value)+1023;
                    else if (label == "p3") p3 = (-value)+1023;
                    else if (label == "p4") p4 = (-value)+1023;
                }
                line.clear();
            } else if (c != '\r') {
                line += c;
            }
        }
    }
}

// ========================================
//                Wave Edit
// ========================================
void editWave(){
    // Map p1 to point index
    editIndex = static_cast<int>(norm(p1, 0.0f, 1023.0f, 0.0f, WAVE_RES-1));

    // Edit the waveform point
    if(abs(p2 - lastP2) > 1) {
        std::cout << "Editing pt: " << editIndex;
        wavePoints[editIndex] = norm(p2, 0.0f, 1023.0f, -2.0f, 2.0f);
        waveNeedsRebuild = true;
    }

    // Curvature
    if(abs(p3 - lastP3) > 1) curvature = norm(p3, 0.0f, 1023.0f, 0.1f, 5.0f); 

    // Morph knob
    knobPosition = norm(p4, 0.0f, 1023.0f, 0.0f, 0.9f);
    if(abs(p4 - lastP4) > 2) custom = false;
    if(abs(p1-lastP1) > 2 || abs(p2-lastP2) > 2 || abs(p3-lastP3) > 2) custom = true;
    updateWave();
}


// =================================================
//                     ADSR Edit
// =================================================

void editADSR(){
    // Attack
    if(abs(p1 - lastP1) > 1) attack = norm(p1, 0.0f, 1023.0f, 0.0f, 5.0f);
    // Decay
    if(abs(p2 - lastP2) > 1) decay = norm(p2, 0.0f, 1023.0f, 0.0f, 5.0f);
    // Sustain
    if(abs(p3 - lastP3) > 1) sustain = norm(p3, 0.0f, 1023.0f, 0.0f, 1.0f);
    // Release
    if(abs(p4 - lastP4) > 1) release = norm(p4, 0.0f, 1023.0f, 0.0f, 5.0f);
}

// =================================================
//                   Reverb Edit
// =================================================

void editReverb() {
    // Dry/Wet
    if(abs(p1 - lastP1) > 1) {
        float dry = norm(p1, 0.0f, 1023.0f, 0.0f, 1.0f);
        float wet = 1.0f - dry;
        reverb.setDryWet(wet, dry);
    }

    // Room Size — SAFE range
    if(abs(p2 - lastP2) > 1) {
        float size = norm(p2, 0.0f, 1023.0f, 0.1f, 1.5f);
        reverb.setRoomSize(size);
    }

    // Decay — SAFE range
    if(abs(p3 - lastP3) > 1) {
        float decay = norm(p3, 0.0f, 1023.0f, 0.1f, 1.0f);
        reverb.setDecay(decay);
    }

    if(abs(p4 - lastP4) > 1) {
        // future
    }
}

// =================================================
//                     Tone Edit
// =================================================

void editTone() {
    // Output Level
    if(abs(p1 - lastP1) > 1) {
        outputLevel = norm(p1, 0.0f, 1023.0f, 0.0f, 5.0f);
    }

    // Pan
    if(abs(p2 - lastP2) > 1) {
        pan = norm(p3, 0.0f, 1023.0f, -1.0f, 1.0f);
    }

    // Normalize Voices Toggle maybe
    // if(abs(p3 - lastP3) > 5) {
    //     normVoices = (p3 < 512);
    // }

    

    // future
}

// =================================================
//                        MAIN
// =================================================
int main() {
    if (!initSerial()) return 1;

    

    // Choose type
    reverb.mode = ReverbType::SCHROEDER;

    // Set parameters
    reverb.setDryWet(1.0f,0.0f);
    reverb.setRoomSize(10.0f);  // 0-1
    reverb.setDecay(0.9f);     // 0-1


    setNonBlockingInput();
    initWavePoints(); // Initialize waveform

    Pa_Initialize();
    PaStream* stream;
    Pa_OpenDefaultStream(&stream, 0, 2, paFloat32, sampleRate, 256, audioCallback, nullptr);
    Pa_StartStream(stream);

    std::cout << "Polyphonic Synth Ready.\n";
    std::cout << "Press keys 1–8 to morph wave. Press 0 for CUSTOM curve.\n";

    while(true) {
        char c;

        getInp();
        if(lastP1 == -1){
            lastP1 = p1; lastP2 = p2; lastP3 = p3; lastP4 = p4;
        }

        if(menu == WAVE_MENU) editWave(); 
        if(menu == ADSR_MENU) editADSR();
        if(menu == REVERB_MENU) editReverb();

        ssize_t n = read(STDIN_FILENO, &c, 1);
        if(n > 0) {
            if(c == '1') menu = WAVE_MENU;
            else if(c == '2') menu = ADSR_MENU;
            else if(c == '3') menu = REVERB_MENU;
            
            else if(c == 'z' || c == 'x' || c == 'c' || c == 'v') {
                int v = (c=='z')?0:(c=='x')?1:(c=='c')?2:3;
                if(voices[v].active) { voices[v].active=false; voices[v].time=0.0f; }
                else { voices[v].active=true; voices[v].time=0.0f; voices[v].phase=0.0f; voices[v].frequency=noteToHz(noteMapping[v]); }
            }
        }

        usleep(1000);
        lastP1 = p1; lastP2 = p2; lastP3 = p3; lastP4 = p4;
    }

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 0;
}






// ======================================
//            Example ADSR Draw
// ======================================

// void drawADSR(U8G2 &u8g2)
// {
//     float sustainVisTime = 0.2f; // purely visual

//     float totalTime = attack + decay + sustainVisTime + release;

//     for(int x = 0; x < 128; x++)
//     {
//         float t = (float)x / 128.0f * totalTime;

//         bool noteHeld = (t < attack + decay + sustainVisTime);

//         float env = ADSR(attack, decay, sustain, release,
//                          noteHeld,
//                          t,
//                          1.0f);

//         // scale ADSR output (0–1) to screen height (0–63)
//         int y = 63 - int(env * 63.0f);

//         u8g2.drawPixel(x, y);
//     }
// }


// ======================================
//          Example Voice Draw
// ======================================

// void drawVoice(u8g2_t &u8g2)
// {
//     u8g2.clearBuffer();

//     const int width  = 128;
//     const int height = 64;

//     int lastY = -1;

//     for (int x = 0; x < width; x++)
//     {
//         float phase = (float)x / (float)(width - 1);
//         float v = osc(phase);   // -1 to +1

//         int y = (int)((1.0f - v) * 0.5f * (height - 1));

//         if (x > 0)
//             u8g2.drawLine(x - 1, lastY, x, y);

//         lastY = y;
//     }

//     u8g2.sendBuffer();
// }

// ======================================
//          Example Output Draw
// ======================================

// void drawOutput(u8g2_t &u8g2, float sample) {
//     static int x = 0;

//     const int width  = 128;
//     const int height = 64;

//     // Map sample (-1..+1) to Y coordinate (0..63)
//     int y = (int)((1.0f - sample) * 0.5f * (height - 1));

//     // --- erase only this column ---
//     for (int i = 0; i < height; i++) {
//         u8g2.setDrawColor(0);      // draw "black"
//         u8g2.drawPixel(x, i);      // clear old pixels
//     }

//     // --- draw new waveform column ---
//     u8g2.setDrawColor(1);          // draw "white"

//     // draw a vertical line from the middle to the point
//     int mid = height / 2;
//     if (y > mid)
//         u8g2.drawLine(x, mid, x, y);
//     else
//         u8g2.drawLine(x, y, x, mid);

//     // advance x
//     x++;
//     if (x >= width) x = 0;

//     u8g2.sendBuffer();
// }

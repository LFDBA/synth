#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <string>
#include <functional>
#include <rtaudio/RtAudio.h>
#include "Reverb.h"  // your existing reverb

using namespace rt::audio;

// =======================
// Constants
// =======================
const double sampleRate = 48000.0;
const int numVoices = 7;
const int TABLE_SIZE = 8192;
float outputLevel = 1.0f;
float pan = 0.0f;
float curvature = 1.0f;

// =======================
// Waveform
// =======================
const int WAVE_RES = 12;
float wavePoints[WAVE_RES];
std::vector<float> customTable;
bool waveNeedsRebuild = true;
bool custom = false;

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }
inline float curveInterp(float a, float b, float t, float curv) {
    if(curv != 1.0f) t = powf(t, curv);
    return lerp(a, b, t);
}

void rebuildWaveTable() {
    for(int i = 0; i < TABLE_SIZE; i++) {
        float t = float(i) / (TABLE_SIZE - 1);
        float idxF = t * (WAVE_RES - 1);
        int idx = int(idxF);
        float frac = idxF - idx;
        float v1 = wavePoints[idx];
        float v2 = wavePoints[std::min(idx+1, WAVE_RES-1)];
        customTable[i] = curveInterp(v1, v2, frac, curvature);
    }
    waveNeedsRebuild = false;
}

void initWavePoints() {
    for(int i=0;i<WAVE_RES;i++) wavePoints[i] = -1.0f + 2.0f*i/(WAVE_RES-1);
    customTable.resize(TABLE_SIZE);
    waveNeedsRebuild = true;
}

// =======================
// Oscillators
// =======================
float sineWave(float phase)     { return sinf(2.0f*M_PI*phase); }
float squareWave(float phase)   { return (phase < 0.5f) ? 1.0f : -1.0f; }
float sawWave(float phase)      { return 2.0f*(phase - 0.5f); }
float triangleWave(float phase) { return 4.0f*fabsf(phase - 0.5f) - 1.0f; }

// =======================
// ADSR
// =======================
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;
    float time = 0.0f;
    float oscVolume = 1.0f;
};

float attack=0.1f, decay=0.1f, sustain=0.8f, release=0.0f;

float ADSR(float t, bool trig, float lvl) {
    if(trig) {
        if(t < attack) return powf(t/attack, 3.0f) * lvl;
        else if(t < attack+decay) return (1.0f - powf((t-attack)/decay, 1.0f/3.0f)*(1.0f-sustain)) * lvl;
        else return sustain*lvl;
    } else {
        if(t<release) return (1.0f - powf(t/release, 1.0f/3.0f)) * (sustain*lvl);
        return 0.0f;
    }
}

// =======================
// Global
// =======================
Voice voices[numVoices];
float knobPosition=0.0f;
Reverb reverb(sampleRate);

// =======================
// Audio callback
// =======================
auto audioCallback = [](void* outputBuffer, void* /*inputBuffer*/, unsigned int nFrames,
                        double /*streamTime*/, unsigned int /*status*/, void* /*userData*/) -> int {
    float* out = (float*)outputBuffer;

    if(custom && waveNeedsRebuild) rebuildWaveTable();

    for(unsigned int i=0;i<nFrames;i++) {
        float mix = 0.0f;
        int activeVoices=0;

        for(int v=0;v<numVoices;v++) {
            if(!voices[v].active) continue;
            activeVoices++;
            voices[v].time += 1.0/sampleRate;
            voices[v].phase += voices[v].frequency/sampleRate;
            if(voices[v].phase >= 1.0f) voices[v].phase -= 1.0f;

            float env = ADSR(voices[v].time, voices[v].active, voices[v].oscVolume);
            float sample = 0.0f;

            if(custom) {
                int idx = int(voices[v].phase*TABLE_SIZE);
                if(idx>=TABLE_SIZE) idx=TABLE_SIZE-1;
                sample = customTable[idx]*env;
            } else {
                float segment = knobPosition*4.0f;
                int idx = (int)floor(segment);
                float blend = segment-idx;
                float wave1=0, wave2=0;
                switch(idx) {
                    case 0: wave1=sineWave(voices[v].phase); wave2=squareWave(voices[v].phase); break;
                    case 1: wave1=squareWave(voices[v].phase); wave2=sawWave(voices[v].phase); break;
                    case 2: wave1=sawWave(voices[v].phase); wave2=triangleWave(voices[v].phase); break;
                    case 3: wave1=triangleWave(voices[v].phase); wave2=sineWave(voices[v].phase); break;
                }
                sample = ((1.0f-blend)*wave1 + blend*wave2)*env;
            }
            mix += sample;
        }

        mix = mix / (activeVoices ? activeVoices : 1);  // normalize
        mix = reverb.process(mix) * outputLevel;

        out[2*i] = mix*(1.0f-pan);
        out[2*i+1] = mix*(1.0f+pan);
    }

    return 0;
};

// =======================
// Main
// =======================
int main() {
    initWavePoints();
    reverb.mode = ReverbType::SCHROEDER;
    reverb.setDryWet(1.0f,0.0f);
    reverb.setRoomSize(0.5f);
    reverb.setDecay(0.8f);

    // RtAudio with error callback
    auto errorCallback = [](RtAudioErrorType type, const std::string &msg) {
        std::cerr << "RtAudio Error: " << msg << "\n";
    };

    RtAudio dac(RtAudio::Api::UNSPECIFIED, errorCallback);

    RtAudio::StreamParameters oParams;
    oParams.deviceId = dac.getDefaultOutputDevice();
    oParams.nChannels = 2;
    oParams.firstChannel = 0;

    RtAudio::StreamOptions options;
    options.flags = 0;

    dac.openStream(&oParams, nullptr, RTAUDIO_FLOAT32, sampleRate, nullptr, audioCallback, nullptr, &options);
    dac.startStream();

    std::cout << "Synth running...\nPress Ctrl+C to quit.\n";

    while(true) { sleep(1); }  // dummy main loop
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

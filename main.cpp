#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <linux/input.h>
#include <algorithm>

#include <rtaudio/RtAudio.h>   // ✅ Correct v6 include
using namespace rt::audio;
#include "Reverb.h"

// --- Constants ---
const float sampleRate = 48000.0f;
const int numVoices = 7;
float outputLevel = 4.0f;
float pan = 0.0f;
float curvature = 1.0f;

// --- Globals ---
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;
    float time = 0.0f;
    float oscVolume = 1.0f;
};
Voice voices[numVoices];

Reverb reverb(sampleRate);

float wavePoints[12];
std::vector<float> customTable;
bool custom = false;
bool waveNeedsRebuild = true;
const int TABLE_SIZE = 8192;

float knobPosition = 0.9f;
float attack=0.5f, decay=0.5f, sustain=0.8f, release=0.0f;
int p1,p2,p3,p4;
int lastP1=-1,lastP2,lastP3,lastP4;
int fd=-1;

// --- Normalize function ---
float norm(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

// --- Waveforms ---
float sineWave(float phase){ return sinf(2.0f*M_PI*phase); }
float squareWave(float phase){ return (phase<0.5f)?1.0f:-1.0f; }
float sawWave(float phase){ return 2.0f*(phase-0.5f); }
float triangleWave(float phase){ return 4.0f*fabsf(phase-0.5f)-1.0f; }

// --- ADSR ---
float ADSR(float attack,float decay,float sustain,float release,bool trig,float t,float lvl){
    if(trig){
        if(t<attack) return powf(t/attack,3.0f)*lvl;
        else if(t<attack+decay) return (1.0f-(powf((t-attack)/decay,1.0f/3.0f)*(1.0f-sustain)))*lvl;
        else return sustain*lvl;
    } else {
        if(t<release) return (1.0f-powf(t/release,1.0f/3.0f))*(sustain*lvl);
        else return 0.0f;
    }
}

// --- Rebuild custom wave ---
inline float lerp(float a,float b,float t){return a+(b-a)*t;}
inline float curveInterp(float a,float b,float t,float curv){ if(curv!=1.0f) t=powf(t,curv); return lerp(a,b,t); }

void rebuildWaveTable(){
    for(int i=0;i<TABLE_SIZE;i++){
        float t=float(i)/(TABLE_SIZE-1);
        float idxF=t*11; // WAVE_RES-1
        int idx=int(idxF);
        float frac=idxF-idx;
        float v1=wavePoints[idx];
        float v2=wavePoints[std::min(idx+1,11)];
        customTable[i]=curveInterp(v1,v2,frac,curvature);
    }
    waveNeedsRebuild=false;
}

void initWavePoints(){
    for(int i=0;i<12;i++) wavePoints[i]=norm(i,0,11,-2.0f,2.0f);
    customTable.resize(TABLE_SIZE);
    waveNeedsRebuild=true;
}

// --- Audio callback ---
int audioCallback(void* outputBuffer, void*, unsigned int nBufferFrames,
                  double, RtAudioStreamStatus, void*){
    float* out=(float*)outputBuffer;

    if(custom && waveNeedsRebuild) rebuildWaveTable();

    for(unsigned int i=0;i<nBufferFrames;i++){
        float mix=0.0f;
        for(int v=0;v<numVoices;v++){
            if(!voices[v].active) continue;
            voices[v].time+=1.0f/sampleRate;
            voices[v].phase+=voices[v].frequency/sampleRate;
            if(voices[v].phase>=1.0f) voices[v].phase-=1.0f;

            float adsr=ADSR(attack,decay,sustain,release,voices[v].active,voices[v].time,voices[v].oscVolume);
            float sample;
            if(custom){
                int idx=int(voices[v].phase*TABLE_SIZE);
                if(idx>=TABLE_SIZE) idx=TABLE_SIZE-1;
                sample=customTable[idx]*adsr;
            } else {
                float seg=knobPosition*4.0f;
                int idx=(int)floor(seg);
                float blend=seg-idx;
                float wave1,wave2;
                switch(idx){
                    case 0: wave1=sineWave(voices[v].phase); wave2=squareWave(voices[v].phase); break;
                    case 1: wave1=squareWave(voices[v].phase); wave2=sawWave(voices[v].phase); break;
                    case 2: wave1=sawWave(voices[v].phase); wave2=triangleWave(voices[v].phase); break;
                    case 3: wave1=triangleWave(voices[v].phase); wave2=sineWave(voices[v].phase); break;
                    default: wave1=wave2=0.0f;
                }
                sample=((1.0f-blend)*wave1+blend*wave2)*adsr;
            }
            mix+=sample;
        }
        mix*=outputLevel;
        mix=reverb.process(mix);
        out[2*i]=mix*(1.0f-pan);
        out[2*i+1]=mix*(pan+1.0f);
    }
    return 0;
}

// --- RtAudio error callback ---
void rtErrorCallback(RtAudioErrorType type, const std::string &message){
    std::cerr << "RtAudio Error (" << type << "): " << message << std::endl;
}

// --- Main ---
int main(){
    initWavePoints();

    RtAudio dac;
    dac.setErrorCallback(rtErrorCallback);

    RtAudio::StreamParameters oParams;
    oParams.deviceId=dac.getDefaultOutputDevice();
    oParams.nChannels=2;
    oParams.firstChannel=0;

    unsigned int bufferFrames=256;

    RtAudioErrorType err;
    err=dac.openStream(&oParams,nullptr,RTAUDIO_FLOAT32,48000, &bufferFrames, &audioCallback, nullptr);
    if(err!=0){ std::cerr << "Error opening stream: " << err << "\n"; return 1; }

    err=dac.startStream();
    if(err!=0){ std::cerr << "Error starting stream: " << err << "\n"; return 1; }

    std::cout << "Polyphonic Synth Ready\n";

    // --- Non-blocking input setup ---
    termios tt{};
    tcgetattr(STDIN_FILENO,&tt);
    tt.c_lflag &= ~(ICANON|ECHO);
    tcsetattr(STDIN_FILENO,TCSANOW,&tt);
    fcntl(STDIN_FILENO,F_SETFL,O_NONBLOCK);

    // --- Main loop ---
    while(true){
        char c;
        ssize_t n=read(STDIN_FILENO,&c,1);
        if(n>0){
            if(c=='z'||c=='x'||c=='c'||c=='v'){
                int v=(c=='z')?0:(c=='x')?1:(c=='c')?2:3;
                if(voices[v].active){ voices[v].active=false; voices[v].time=0.0f; }
                else { voices[v].active=true; voices[v].time=0.0f; voices[v].phase=0.0f; voices[v].frequency=261.63f; }
            }
        }
        usleep(1000);
    }

    err=dac.stopStream();
    if(err!=0) std::cerr << "Error stopping stream: " << err << "\n";

    dac.closeStream();
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

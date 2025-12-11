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
#include <rtaudio/RtAudio.h>
#include <ncurses.h>
#include "screen.h"
#include "Reverb.h"

// ======================================================
//                  Constants & Globals
// ======================================================
const float sampleRate = 48000.0f;
const int numVoices = 7;
float outputLevel = 0.1f;
bool normVoices = true;
float pan = 0.0f;
int fd;
int editIndex = 0;
bool edit = false;

Reverb reverb(sampleRate);

enum Mode { MODE_NONE, VOICE_TONE_MENU, TONE_MENU, WAVE_MENU, ADSR_MENU, REVERB_MENU };
Mode menu = TONE_MENU;

int p1,p2,p3,p4;
int lastP1=-1,lastP2,lastP3,lastP4;
float knobPosition = 0.9f;

const int WAVE_RES = 12;
float wavePoints[WAVE_RES];
std::vector<float> customTable;
bool waveNeedsRebuild = true;
const int TABLE_SIZE = 8192;
float curvature = 1.0f;

struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;
    bool releasing = false;
    float envTime = 0.0f;
    float oscVolume = 1.0f;
};

Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60, 64, 67, 72};
bool custom = false;

float attack = 0.5f;
float decay = 0.5f;
float sustain = 0.8f;
float release = 0.0f;

// ======================================================
//                     Utils
// ======================================================
float norm(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min;
}

float sineWave(float phase)      { return sinf(2.0f*M_PI*phase); }
float squareWave(float phase)    { return (phase<0.5f)?1.0f:-1.0f; }
float sawWave(float phase)       { return 2.0f*(phase-0.5f); }
float triangleWave(float phase)  { return 4.0f*fabsf(phase-0.5f)-1.0f; }

float softClip(float x) {
    if(x > 1.0f) return 1.0f - expf(-x);
    if(x < -1.0f) return -1.0f + expf(x);
    return x;
}

// ======================================================
//               Wave Table Editor
// ======================================================
inline float lerp(float a,float b,float t){return a+(b-a)*t;}
inline float curveInterp(float a,float b,float t,float curv){if(curv!=1.0f)t=powf(t,curv);return lerp(a,b,t);}

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
    for(int i=0;i<WAVE_RES;i++) wavePoints[i]=norm(i,0,WAVE_RES-1,-2.0f,2.0f);
    customTable.resize(TABLE_SIZE);
    waveNeedsRebuild=true;
}

// ======================================================
//                   Note to Hz
// ======================================================
float noteToHz(int noteNumber) {
    float fC0 = 16.35f;
    return fC0*pow(2.0f,float(noteNumber)/12.0f);
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
            float sample=0.0f;

            if(voice.active || voice.releasing){
                activeVoices++;
                voice.envTime += 1.0f/sampleRate;
                float env = ADSR(attack, decay, sustain, release, voice.active, voice.envTime, voice.oscVolume);

                float oscSample;
                if(custom){
                    int idx=int(voice.phase*TABLE_SIZE);
                    if(idx>=TABLE_SIZE) idx=TABLE_SIZE-1;
                    oscSample = customTable[idx];
                } else {
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
                    oscSample=((1.0f-blend)*w1 + blend*w2);
                }

                sample=oscSample*env;
                voice.phase+=voice.frequency/sampleRate;
                if(voice.phase>=1.0f) voice.phase-=1.0f;

                if(voice.releasing && voice.envTime>=release){
                    voice.releasing=false;
                    voice.envTime=0.0f;
                    voice.phase=0.0f;
                }
            }

            mix+=sample;
        }

        if(normVoices && activeVoices>0) mix/=activeVoices;
        mix=softClip(mix*outputLevel);
        mix=reverb.process(mix);

        output[2*i]=mix*(1.0f-pan);
        output[2*i+1]=mix*(pan+1.0f);
    }

    return 0;
}

// ======================================================
//                   Main
// ======================================================
int main() {
    initWavePoints();
    initKeyboard();

    RtAudio dac;
    RtAudio::StreamParameters oParams;
    oParams.deviceId=dac.getDefaultOutputDevice();
    oParams.nChannels=2;
    unsigned int bufferFrames=256;

    try {
        dac.openStream(&oParams,nullptr,RTAUDIO_FLOAT32,sampleRate,&bufferFrames,&audioCallback);
        dac.startStream();
    } catch(RtAudioError &e){ e.printMessage(); return 1; }

    std::cout << "Polyphonic Synth Ready\n";

    while(true){
        drawADSR(attack, decay, sustain, release);
        usleep(1000);
    }

    try{ dac.stopStream(); } catch(RtAudioError &e){}
    if(dac.isStreamOpen()) dac.closeStream();
    closeKeyboard();
}

#include <iostream>
#include <cmath>
#include <portaudio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <array>

// === Constants ===
const float sampleRate = 48000.0f;
const int numVoices = 7;

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
//      Custom Wave Wavetable System (Optimised)
// ======================================================

static std::vector<std::array<float,2>> controlPoints = {
    {0,0.0f}, {512, 2.0f}, {1024, 0.0f}, {1536, -2.0f}, {2048, 0.0f}
};

static std::vector<float> customTable;
static bool waveNeedsRebuild = true;
const int TABLE_SIZE = 2048;

// --- Fast linear interpolation ---
inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// --- Iterative De Casteljau ---
std::array<float,2> bezierPointIter(const std::vector<std::array<float,2>>& pts, float t) {
    std::vector<std::array<float,2>> temp = pts;

    for (size_t k = pts.size() - 1; k > 0; --k) {
        for (size_t i = 0; i < k; i++) {
            temp[i][0] = lerp(temp[i][0], temp[i+1][0], t);
            temp[i][1] = lerp(temp[i][1], temp[i+1][1], t);
        }
    }
    return temp[0];
}
// --- Linear interpolation of control points ---
inline float linearBezier(const std::vector<std::array<float,2>>& pts, float t) {
    int n = pts.size();
    float pos = t * (n - 1);
    int idx = (int)floor(pos);
    if(idx >= n-1) return pts.back()[1];
    float localT = pos - idx;
    return lerp(pts[idx][1], pts[idx+1][1], localT);
}

// --- Full Bézier point (De Casteljau) ---
inline float fullBezier(const std::vector<std::array<float,2>>& pts, float t) {
    return bezierPointIter(pts, t)[1];
}

// --- Rebuild wavetable with curvature morph ---
void rebuildWaveTable(float curvature = 1.0f) {
    customTable.resize(TABLE_SIZE);

    for(int i = 0; i < TABLE_SIZE; i++) {
        float t = float(i) / float(TABLE_SIZE - 1);

        float linearVal = linearBezier(controlPoints, t);
        float smoothVal = fullBezier(controlPoints, t);

        // Morph between linear and smooth using curvature
        customTable[i] = lerp(linearVal, smoothVal, curvature);
    }

    waveNeedsRebuild = false;
}



// --- Your requested function ---
void updateWave() {
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

    float pInput = 0.0f;
    float pOutput = 0.0f;
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
//                  Global Variables
// =================================================

int knobPosition = 1;
Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60};
float voiceSample;
bool custom = false;

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
    float t = getMorphValue(knobPosition);

    if (custom && waveNeedsRebuild)
        rebuildWaveTable();

    for(unsigned long i=0; i<framesPerBuffer; i++) {
        float mix = 0.0f;

        for(int v=0; v<numVoices; v++) {

            voices[v].time += 1.0f / sampleRate;
            voices[v].phase += voices[v].frequency / sampleRate;
            if(voices[v].phase >= 1.0f) voices[v].phase -= 1.0f;

            float adsr = ADSR(0.5f, 0.5f, 0.8f, 1.2f, voices[v].active, voices[v].time, voices[v].oscVolume);

            float sample;

            if(custom) {
                int index = (int)(voices[v].phase * TABLE_SIZE) & (TABLE_SIZE - 1);
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

        output[2*i]     = mix;
        output[2*i + 1] = mix;
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
//                        MAIN
// =================================================

int main() {
    setNonBlockingInput();

    Pa_Initialize();

    PaStream* stream;
    Pa_OpenDefaultStream(&stream, 0, 2, paFloat32, sampleRate, 256, audioCallback, nullptr);
    Pa_StartStream(stream);

    std::cout << "Polyphonic Synth Ready.\n";
    std::cout << "Press keys 1–8 to morph wave. Press 0 for CUSTOM curve.\n";

    while(true) {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);

        if(n > 0) {

            if(c >= '1' && c <= '8') {
                knobPosition = c - '0';
                custom = false;
                std::cout << "Morph knob = " << knobPosition << "\n";
            }

            if(c == '0') {
                custom = true;
                updateWave();   // you can manually force rebuild if needed
                std::cout << "Custom wave ON\n";
            }

            if(c >= 'q' && c <= 'u') {
                int v = c - 'q';

                if(voices[v].active) {
                    voices[v].active = false;
                    voices[v].time = 0.0f;
                    std::cout << "Voice " << v << " OFF\n";
                } else {
                    voices[v].active = true;
                    voices[v].time = 0.0f;
                    voices[v].phase = 0.0f;
                    voices[v].frequency = noteToHz(noteMapping[v]);
                    std::cout << "Voice " << v << " ON\n";
                }
            }
        }

        usleep(1000);
    }

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 0;
}

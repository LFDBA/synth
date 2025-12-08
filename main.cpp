#include <iostream>
#include <cmath>
#include <portaudio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <vector>
#include <array>
#include <unistd.h>
#include <linux/input.h>
#include <algorithm>
#include <sstream>
#include <string>


float norm(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// === Constants ===
const float sampleRate = 48000.0f;
const int numVoices = 7;
const char* device = "/dev/input/by-id/usb-Griffin_Technology__Inc._Griffin_PowerMate-event-if00";

enum Mode {
    MODE_NONE,
    WAVE_MENU,
    ADSR_MENU
};
Mode menu = WAVE_MENU;

int p1;
int p2;
int p3;
int p4;

float curvature = 1.0f;

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
//                     Custom Wave
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
void rebuildWaveTable() {
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
//                  Global Vars
// =================================================

int knobPosition = 1;
Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60};
float voiceSample;
bool custom = false;
struct input_event ev;
int fd = -1;
bool hold = true;
float inpVal = 0;
float sweepPos = 0;
int inpMode = -1;
float attack = 0.5f;
float decay = 0.5f;
float sustain = 0.8f;
float release = 1.2f;

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

            float adsr = ADSR(attack, decay, sustain, release, voices[v].active, voices[v].time, voices[v].oscVolume);

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
//               Read Input Device
// =================================================
std::string line = "";      // Buffer for incoming serial lines
int lastValue = 0;          // Last read potentiometer value

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

int lastP1 = -1;
int lastP2;
int lastP3;
int lastP4;
// Reads from serial, returns the latest value
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
                    std::cout << "[" << label << "] " << value << std::endl;

                    
                    if (label == "p1") p1 = (-value)+1023;
                    else if (label == "p2") p2 = (-value)+1023;
                    else if (label == "p3") p3 = (-value)+1023;
                    else if (label == "p4") p4 = (-value)+1023;
                }
                line.clear();
            }
            else if (c != '\r') {
                line += c;
            }
        }
    }
    
    
}

// ========================================
//                Wave Edit
// ========================================

void editWave(){
    sweepPos = norm(p1, 0.0f, 1023.0f, 0.0f, 1.0f);
    int sweepInt = static_cast<int>(sweepPos);
    auto it = std::find_if(controlPoints.begin(), controlPoints.end(),
                        [sweepInt](const std::array<float,2>& row) {
                            return row[0] == sweepPos;
                        });

    
    if(abs(p2-lastP2)>1){
        if (it != controlPoints.end()) {
            std::cout << "Found row where first element = " << sweepPos << "\n";
            std::cout << "Full row: [" << (*it)[0] << ", " << (*it)[1] << "]\n";
            (*it)[1] = norm(p2, 0.0f, 1023.0f, -2.0f, 2.0f);
        } else {
            controlPoints.push_back({sweepPos, norm(p2, 0.0f, 1023.0f, -2.0f, 2.0f)});
            
        }
    }
    if(abs(p3-lastP3)>1) curvature = norm(p3, 0.0f, 1023.0f, -2.0f, 2.0f);
    

    updateWave();
    
}


// =================================================
//                        MAIN
// =================================================

int main() {
    if (!initSerial()) return 1;
    // =============================================
    //              Init Input Device
    // =============================================

    

    

    std::cout << "Reading " << device << " Events \n";


    
    



    setNonBlockingInput();

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
            lastP1 = p1;
            lastP2 = p2;
            lastP3 = p3;
            lastP4 = p4;
        }
        
        if(menu == WAVE_MENU) editWave(); 
        else if(inpMode == 2 && attack >= 0) {
            attack = inpVal/100;   
            if(attack < 0) attack = 0;
        }
        else if(inpMode == 3 && decay >= 0) {
            decay = inpVal/100;   
            if(decay < 0) decay = 0;
        }
        else if(inpMode == 4 && sustain >= 0) {
            sustain = inpVal/100;   
            if(sustain < 0) sustain = 0;
        }
        else if(inpMode == 5 && release >= 0) {
            release = inpVal/100;
            if(release < 0) release = 0;
        }

        ssize_t n = read(STDIN_FILENO, &c, 1);
        if(n > 0) {

            // --- Wave Morph Keys (1–8) ---
            if(c >= '1' && c <= '8') {
                knobPosition = c - '0';
                custom = false;
                std::cout << "Morph knob = " << knobPosition << "\n";
            }

            // --- Custom Wave (0) ---
            else if(c == '0') {
                custom = true;
                updateWave(); // force rebuild
                std::cout << "Custom wave ON\n";
            }

            // --- Edit Wave / Curvature Toggle (9) ---
            else if(c == '9') {
                inpMode = (inpMode == 1) ? 0 : 1;
                std::cout << "Edit mode = " << inpMode << "\n";
            }

            // --- ADSR Editing ---
            else if(c == 'a') inpMode = 2; // attack
            else if(c == 'd') inpMode = 3; // decay
            else if(c == 's') inpMode = 4; // sustain
            else if(c == 'r') inpMode = 5; // release

            // --- Voice Toggle Keys ---
            else if(c == 'z' || c == 'x' || c == 'c' || c == 'v') {
                int v = 0;
                if(c == 'z') v = 0;
                else if(c == 'x') v = 1;
                else if(c == 'c') v = 2;
                else if(c == 'v') v = 3;

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
        lastP1 = p1;
        lastP2 = p2;
        lastP3 = p3;
        lastP4 = p4;
    }
    
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 0;
}

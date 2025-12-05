#include <iostream>
#include <cmath>
#include <portaudio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

// --- Constants ---
const float sampleRate = 48000.0f;
const int numVoices = 7;

// --- Function: note to Hz ---
float noteToHz(int noteNumber) {
    float fC0 = 16.35f; // lowest C
    return fC0 * pow(2.0f, float(noteNumber - 12) / 12.0f);
}

// --- Waveform functions ---
float sineWave(float phase)      { return sin(2.0f * M_PI * phase); }
float squareWave(float phase)    { return (phase < 0.5f) ? 1.0f : -1.0f; }
float sawWave(float phase)       { return 2.0f * (phase - 0.5f); }
float triangleWave(float phase)  { return 4.0f * fabs(phase - 0.5f) - 1.0f; }

// --- Map keyboard to morph value ---
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

// --- Voice struct ---
struct Voice {
    float phase = 0.0f;
    float frequency = 261.63f;
    bool active = false;
    float time = 0.0f;
    float oscVolume = 0.04f;

    // Commented systems from your old code
    float pInput = 0.0f;
    float pOutput = 0.0f;
};

float ADSR(float attack, float decay, float sustain, float release, Voice& voice) {
    bool noteOn = voice.active;
    float time = voice.time;
    float baseLevel = voice.oscVolume;
    float curvature = 3.0f;

    // curving is essentially pow(time / param, curvature)   1.0f/curvature for reverse slope (steep-to-shallow)
    // linear would remove pow and just use (time / param), could also set curvature = 1.0f

    if(noteOn) {
        if(time < attack) {
            return pow(time / attack, curvature) * baseLevel;
        } else if(time < attack + decay) {
            return (1.0f - (pow((time - attack) / decay, 1.0f/curvature) * (1.0f - sustain))) * baseLevel;
        } else {
            return sustain*baseLevel;
        }
    } else {
        if(time < release) {
            return (1.0f - pow(time / release, 1.0f/curvature)) * (sustain * baseLevel);
        } else {
            return 0.0f;
        }
    }
}

// --- Global variables ---
int knobPosition = 1;
Voice voices[numVoices];
int noteMapping[numVoices] = {48, 52, 55, 60}; // 7 keys C,D,E,F,G,A,B-ish
float drive = 0.004f;
float sf = 1.2f;

// --- Audio callback ---
static int audioCallback(
    const void*, void* outputBuffer,
    unsigned long framesPerBuffer,
    const PaStreamCallbackTimeInfo*,
    PaStreamCallbackFlags,
    void*)
{
    float* output = (float*)outputBuffer;
    float t = getMorphValue(knobPosition);

    for(unsigned long i=0; i<framesPerBuffer; i++) {
        float sample = 0.0f;

        for(int v=0; v<numVoices; v++) {
            // if(!voices[v].active){
            //     continue;
            // } 
            voices[v].time += 1.0f / sampleRate;
            // Increment phase
            voices[v].phase += voices[v].frequency / sampleRate;
            if(voices[v].phase >= 1.0f) voices[v].phase -= 1.0f;

            // Morph waveform
            float segment = t * 4.0f;
            int index = (int)floor(segment);
            float blend = segment - index;

            float wave1, wave2;
            switch(index) {
                case 0: wave1 = sineWave(voices[v].phase);     wave2 = squareWave(voices[v].phase); break;
                case 1: wave1 = squareWave(voices[v].phase);   wave2 = sawWave(voices[v].phase);    break;
                case 2: wave1 = sawWave(voices[v].phase);      wave2 = triangleWave(voices[v].phase); break;
                case 3: wave1 = triangleWave(voices[v].phase); wave2 = sineWave(voices[v].phase);   break;
                default: wave1 = wave2 = 0.0f;
            }
            float adsr = ADSR(0.5f, 0.5f, 0.8f, 1.2f, voices[v]);
            float voiceSample = ((1.0f - blend) * wave1 + blend * wave2)*adsr;

            // ----------------- Drive ----------------- //
            // if(voiceSample > drive) voiceSample = drive;
            // if(voiceSample < -drive) voiceSample = -drive;
            // voiceSample *= (oscVolume / drive);

            // ----------------- Filtering ----------------- //
            // float sampleInput = voiceSample;
            // voiceSample = sf * (voices[v].pOutput + voiceSample - voices[v].pInput);
            // voices[v].pInput = sampleInput;
            // voices[v].pOutput = voiceSample;

            sample += voiceSample;
        }

        // Normalize by number of voices

        output[2*i]     = sample;
        output[2*i + 1] = sample;
    }

    return 0;
}

// --- Non-blocking keyboard setup ---
void setNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~ICANON;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~ICANON;
    ttystate.c_lflag &= ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

int main() {
    setNonBlockingInput();

    // Initialize PortAudio
    PaError error = Pa_Initialize();
    if(error != paNoError) { std::cout << "PortAudio error: " << Pa_GetErrorText(error) << std::endl; return 1; }

    PaStream* stream;
    error = Pa_OpenDefaultStream(&stream, 0, 2, paFloat32, sampleRate, 256, audioCallback, nullptr);
    if(error != paNoError) { std::cout << "PortAudio error: " << Pa_GetErrorText(error) << std::endl; return 1; }
    error = Pa_StartStream(stream);
    if(error != paNoError) { std::cout << "PortAudio error: " << Pa_GetErrorText(error) << std::endl; return 1; }

    std::cout << "Polyphonic morphing synth. Keys 1-7 trigger notes, 1-8 knob for waveform morph.\n";

    // --- Main loop ---
    while(true) {
        char c;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if(n > 0) {
            // Morph knob
            if(c >= '1' && c <= '8') {
                knobPosition = c - '0';
                std::cout << "Wave morph set to key " << knobPosition << std::endl;
            }

            // Note on/off: map keys 1-7 to voices
            if(c >= 'q' && c <= 'u') { // q=voice0, w=voice1, ... u=voice6
                int v = c - 'q';
                if(voices[v].active){
                    voices[v].active = false;
                    voices[v].time = 0.0f; // reset envelope time
                    std::cout << "Voice " << v << " OFF\n";
                    continue;
                }else{
                    voices[v].active = true;
                }
                voices[v].time = 0.0f; // reset envelope time
                voices[v].frequency = noteToHz(noteMapping[v]);
                voices[v].phase = 0.0f; // reset phase
                std::cout << "Voice " << v << " ON\n";
            }
            // Key release could be implemented here
        }

        usleep(1000);
    }

    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();

    return 0;
}

#pragma once
#include <vector>
#include <array>
#include <string>
#include <cmath>
#include "FFTConvolver.h"

// ---------- Simple Feedback Delay Reverb ----------
class SimpleReverb {
public:
    SimpleReverb(float delaySec, float decay, int sr);
    float process(float input);
    void setDryWet(float dry, float wet);
    void setRoomSize(float size);
    void setDecay(float decay);

private:
    std::vector<float> buffer;
    int sampleRate;
    int delaySamples;
    int readIndex;
    int writeIndex;
    float feedback;
    float dryMix, wetMix;

    // ✅ Add this
    int baseDelaySamples;  // original delay size, used for scaling
};


// ---------- Schroeder Reverb ----------
struct Comb {
    std::vector<float> buf;
    int idx = 0;
    float feedback;

    Comb(int size, float fb);
    float process(float x);
    void setFeedback(float fb);
};

struct Allpass {
    std::vector<float> buf;
    int idx = 0;
    float feedback;

    Allpass(int size, float fb);
    float process(float x);
};

class SchroederReverb {
public:
    SchroederReverb(int sr);
    float process(float input);
    void setDryWet(float dry, float wet);
    void setRoomSize(float size);
    void setDecay(float decay);

private:
    std::array<Comb,4> combs;
    std::array<Allpass,2> allpasses;
    float dryMix, wetMix;

    // ✅ Add this
    std::array<size_t,4> baseCombSizes;  // original comb buffer sizes
};


// ---------- Convolution Reverb ----------
class ConvolutionReverb {
public:
    bool loadIR(const std::string& path);
    float process(float input);
    void setDryWet(float dry, float wet);

private:
    fftconvolver::FFTConvolver convolver;
    float dryMix = 0.7f, wetMix = 0.3f;
};

// ---------- Unified Wrapper ----------
enum class ReverbType { SIMPLE, SCHROEDER, CONVOLUTION };

class Reverb {
public:
    Reverb(int sr);

    ReverbType mode;

    SimpleReverb simple;
    SchroederReverb schroeder;
    ConvolutionReverb convolution;

    float process(float input);
    void setDryWet(float dry, float wet);
    void setRoomSize(float size);
    void setDecay(float decay);

private:
    int sampleRate;
};

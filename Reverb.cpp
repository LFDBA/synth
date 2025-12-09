// Reverb.cpp
#include "Reverb.h"

#include <fstream>
#include <iostream>
#include <cstring>
#include <algorithm>

// ---------------- SimpleReverb ----------------
SimpleReverb::SimpleReverb(float delaySec, float decay, int sr)
    : feedback(decay), sampleRate(sr)
{
    delaySamples = int(delaySec * sr);
    buffer.assign(delaySamples, 0.0f);
}

float SimpleReverb::process(float input) {
    if (delaySamples <= 0) return input * dryMix; // safety
    float wet = buffer[readIndex];
    buffer[writeIndex] = input + wet * feedback;

    readIndex  = (readIndex + 1) % delaySamples;
    writeIndex = (writeIndex + 1) % delaySamples;

    return input * dryMix + wet * wetMix;
}

void SimpleReverb::setDryWet(float dry, float wet) {
    dryMix = dry; wetMix = wet;
}

void SimpleReverb::setRoomSize(float size) {
    int newDelay = int((0.05f + 0.15f*size) * sampleRate); // 50-200ms
    delaySamples = std::max(1, newDelay);
    buffer.assign(delaySamples, 0.0f);
    readIndex = writeIndex = 0;
}

void SimpleReverb::setDecay(float decay) {
    feedback = decay;
}

// ---------------- Comb ----------------
Comb::Comb(int size, float fb) : buf(size), feedback(fb) {}
float Comb::process(float x) {
    if (buf.empty()) return 0.0f;
    float y = buf[idx];
    buf[idx] = x + y * feedback;
    idx = (idx + 1) % buf.size();
    return y;
}
void Comb::setFeedback(float fb) { feedback = fb; }

// ---------------- Allpass ----------------
Allpass::Allpass(int size, float fb) : buf(size), feedback(fb) {}
float Allpass::process(float x) {
    if (buf.empty()) return x;
    float y = buf[idx];
    float out = -x + y;
    buf[idx] = x + y * feedback;
    idx = (idx + 1) % buf.size();
    return out;
}

// ---------------- SchroederReverb ----------------
SchroederReverb::SchroederReverb(int sr)
    : combs{
        Comb(int(sr*0.0297f), 0.805f),
        Comb(int(sr*0.0371f), 0.827f),
        Comb(int(sr*0.0411f), 0.783f),
        Comb(int(sr*0.0437f), 0.764f)
      },
      allpasses{
        Allpass(int(sr*0.005f), 0.7f),
        Allpass(int(sr*0.0017f),0.7f)
      }
{}

float SchroederReverb::process(float input) {
    float s = 0.0f;
    for (auto& c : combs) s += c.process(input);
    for (auto& a : allpasses) s = a.process(s);
    return input*dryMix + s*wetMix*0.25f;
}

void SchroederReverb::setDryWet(float dry, float wet) { dryMix=dry; wetMix=wet; }

void SchroederReverb::setRoomSize(float size) {
    // scale comb buffer sizes to simulate room size (recreate with new sizes)
    for (int i = 0; i < int(combs.size()); ++i) {
        size_t base = std::max<size_t>(1, combs[i].buf.size());
        size_t newDelay = std::max<size_t>(1, size_t(base * (0.7f + 0.3f*size)));
        combs[i].buf.assign(newDelay, 0.0f);
        combs[i].idx = 0;
    }
}

void SchroederReverb::setDecay(float decay) {
    for(auto& c : combs) c.setFeedback(decay);
}

// ---------------- WAV loader (simple, supports PCM16, PCM32, FLOAT32) ----------------
//
// This loader reads a WAV file and converts it to mono float samples in [-1,1].
// If the WAV is multi-channel, we average all channels to mono.
//
// Limitations: basic and defensive; it supports typical RIFF/WAVE files.
//

static bool readLittleEndianUint32(std::ifstream& f, uint32_t &out) {
    char buf[4];
    if (!f.read(buf, 4)) return false;
    out = (uint8_t)buf[0] | ((uint8_t)buf[1] << 8) | ((uint8_t)buf[2] << 16) | ((uint8_t)buf[3] << 24);
    return true;
}

static bool readLittleEndianUint16(std::ifstream& f, uint16_t &out) {
    char buf[2];
    if (!f.read(buf,2)) return false;
    out = (uint8_t)buf[0] | ((uint8_t)buf[1] << 8);
    return true;
}

static bool loadWavToFloatVector(const std::string& path, std::vector<float>& out) {
    std::ifstream file(path, std::ios::binary);
    if (!file) return false;

    // Read RIFF header
    char riff[4];
    file.read(riff,4);
    if (file.gcount() != 4 || std::memcmp(riff, "RIFF", 4) != 0) return false;

    uint32_t riffSize;
    if (!readLittleEndianUint32(file, riffSize)) return false;

    char wave[4];
    file.read(wave,4);
    if (file.gcount() != 4 || std::memcmp(wave, "WAVE", 4) != 0) return false;

    // Iterate chunks to find "fmt " and "data"
    uint16_t audioFormat = 0;
    uint16_t numChannels = 0;
    uint32_t sampleRate = 0;
    uint16_t bitsPerSample = 0;
    uint32_t dataChunkPos = 0;
    uint32_t dataChunkSize = 0;

    while (file.good()) {
        char chunkId[4];
        if (!file.read(chunkId, 4)) break;
        uint32_t chunkSize;
        if (!readLittleEndianUint32(file, chunkSize)) break;

        if (std::memcmp(chunkId, "fmt ", 4) == 0) {
            // parse fmt chunk (PCM or float)
            uint16_t audioFormatLE, numChannelsLE, bitsPerSampleLE;
            uint32_t sampleRateLE;
            uint32_t byteRate;
            uint16_t blockAlign;
            // read fields
            if (!readLittleEndianUint16(file, audioFormatLE)) return false;
            if (!readLittleEndianUint16(file, numChannelsLE)) return false;
            if (!readLittleEndianUint32(file, sampleRateLE)) return false;
            if (!readLittleEndianUint32(file, byteRate)) return false;
            if (!readLittleEndianUint16(file, blockAlign)) return false;
            if (!readLittleEndianUint16(file, bitsPerSampleLE)) return false;

            audioFormat = audioFormatLE;
            numChannels = numChannelsLE;
            sampleRate = sampleRateLE;
            bitsPerSample = bitsPerSampleLE;

            // skip any extra bytes in fmt chunk
            int remaining = int(chunkSize) - 16;
            if (remaining > 0) file.seekg(remaining, std::ios::cur);
        }
        else if (std::memcmp(chunkId, "data", 4) == 0) {
            dataChunkPos = static_cast<uint32_t>(file.tellg());
            dataChunkSize = chunkSize;
            // jump to next chunk (we'll read data later)
            file.seekg(chunkSize, std::ios::cur);
        } else {
            // skip other chunks
            file.seekg(chunkSize, std::ios::cur);
        }
    }

    if (dataChunkPos == 0 || bitsPerSample == 0 || numChannels == 0) return false;

    // read data chunk
    file.clear();
    file.seekg(dataChunkPos, std::ios::beg);

    size_t bytesPerSample = bitsPerSample / 8;
    size_t frameCount = dataChunkSize / (bytesPerSample * numChannels);

    out.clear();
    out.reserve(frameCount);

    // Helper lambda to read samples per frame and average channels
    for (size_t i = 0; i < frameCount; ++i) {
        double sampleSum = 0.0;
        for (uint16_t ch = 0; ch < numChannels; ++ch) {
            if (bitsPerSample == 16 && audioFormat == 1) {
                int16_t s;
                file.read(reinterpret_cast<char*>(&s), sizeof(int16_t));
                sampleSum += (double)s / 32768.0;
            }
            else if (bitsPerSample == 32 && audioFormat == 1) {
                // 32-bit PCM (rare)
                int32_t s;
                file.read(reinterpret_cast<char*>(&s), sizeof(int32_t));
                sampleSum += (double)s / 2147483648.0;
            }
            else if (bitsPerSample == 32 && audioFormat == 3) {
                // 32-bit float
                float f;
                file.read(reinterpret_cast<char*>(&f), sizeof(float));
                sampleSum += static_cast<double>(f);
            }
            else {
                // unsupported format
                return false;
            }
        }
        double avg = sampleSum / double(numChannels);
        // clamp slightly to [-1,1]
        if (avg > 1.0) avg = 1.0;
        if (avg < -1.0) avg = -1.0;
        out.push_back(static_cast<float>(avg));
    }

    return !out.empty();
}

// ---------------- ConvolutionReverb ----------------
bool ConvolutionReverb::loadIR(const std::string& path) {
    // Try to load WAV -> float vector
    std::vector<float> irData;
    if (!loadWavToFloatVector(path, irData)) {
        // failed to load WAV; try to load a raw float file (binary floats)
        std::ifstream file(path, std::ios::binary);
        if (!file) return false;
        file.seekg(0, std::ios::end);
        std::streampos size = file.tellg();
        file.seekg(0, std::ios::beg);
        if (size <= 0) return false;
        size_t count = static_cast<size_t>(size / sizeof(float));
        irData.resize(count);
        if (!file.read(reinterpret_cast<char*>(irData.data()), count * sizeof(float))) return false;
    }

    if (irData.empty()) return false;

    // Choose a block size. Smaller block -> less latency, more CPU.
    size_t blockSize = 1024;

    // reset the convolver and initialize with IR
    convolver.reset(); // ensure any previous state cleared (API often has reset)
    bool ok = convolver.init(blockSize, irData.data(), irData.size());
    return ok;
}

float ConvolutionReverb::process(float input) {
    float out = 0.0f;
    // FFTConvolver expects arrays; process N=1 sample
    convolver.process(&input, &out, 1);
    return input * dryMix + out * wetMix;
}

void ConvolutionReverb::setDryWet(float dry, float wet) { dryMix=dry; wetMix=wet; }

// ---------------- Unified Wrapper ----------------
Reverb::Reverb(int sr)
    : mode(ReverbType::SCHROEDER), simple(0.08f,0.7f,sr), schroeder(sr), sampleRate(sr)
{}

float Reverb::process(float input) {
    switch(mode) {
        case ReverbType::SIMPLE:      return simple.process(input);
        case ReverbType::SCHROEDER:   return schroeder.process(input);
        case ReverbType::CONVOLUTION: return convolution.process(input);
    }
    return input;
}

void Reverb::setDryWet(float dry,float wet) {
    simple.setDryWet(dry,wet);
    schroeder.setDryWet(dry,wet);
    convolution.setDryWet(dry,wet);
}

void Reverb::setRoomSize(float size) {
    simple.setRoomSize(size);
    schroeder.setRoomSize(size);
    // convolution room size baked in IR
}

void Reverb::setDecay(float decay) {
    simple.setDecay(decay);
    schroeder.setDecay(decay);
    // convolution decay baked in IR
}

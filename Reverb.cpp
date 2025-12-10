// Reverb.cpp
#include "Reverb.h"
#include <fstream>
#include <vector>
#include <algorithm>
#include <cstring>
#include <cmath>
#include <iostream>

// --------------------- SimpleReverb ---------------------
SimpleReverb::SimpleReverb(float delaySec, float decay, int sr)
    : feedback(decay), sampleRate(sr)
{
    baseDelaySamples = int(delaySec * sr);
    delaySamples = baseDelaySamples;
    buffer.assign(delaySamples, 0.0f);
    readIndex = writeIndex = 0;
    dryMix = 0.5f; wetMix = 0.5f;
}

float SimpleReverb::process(float input)
{
    if (delaySamples <= 0) return input * dryMix;

    float wet = buffer[readIndex];
    buffer[writeIndex] = input + wet * feedback;

    readIndex  = (readIndex + 1) % delaySamples;
    writeIndex = (writeIndex + 1) % delaySamples;

    float outSample = input * dryMix + wet * wetMix;

    // optional soft clipping
    return outSample / (1.0f + std::fabs(outSample));
}

void SimpleReverb::setDryWet(float dry, float wet)
{
    dryMix = dry; wetMix = wet;
}

void SimpleReverb::setRoomSize(float size)
{
    // size 0-1 maps to 0.5-1.5 of base delay
    int targetDelay = int(baseDelaySamples * (0.5f + size));
    targetDelay = std::max(1, targetDelay);

    if (targetDelay != delaySamples)
    {
        // resize buffer without losing all content
        std::vector<float> newBuffer(targetDelay, 0.0f);
        size_t copyLen = std::min(buffer.size(), newBuffer.size());
        for (size_t i = 0; i < copyLen; ++i)
            newBuffer[i] = buffer[(readIndex + i) % buffer.size()];
        buffer = std::move(newBuffer);
        readIndex = 0;
        writeIndex = copyLen % targetDelay;
        delaySamples = targetDelay;
    }

    // scale feedback slightly to prevent runaway
    feedback = 0.7f * size;
}

void SimpleReverb::setDecay(float decay)
{
    feedback = decay;
}

// --------------------- Comb ---------------------
Comb::Comb(int size, float fb) : buf(size), feedback(fb), idx(0) {}

float Comb::process(float x)
{
    if (buf.empty()) return 0.0f;
    float y = buf[idx];
    // simple damping
    float damp = 0.9f;
    buf[idx] = x + y * feedback * damp;
    idx = (idx + 1) % buf.size();
    return y;
}

void Comb::setFeedback(float fb) { feedback = fb; }

// --------------------- Allpass ---------------------
Allpass::Allpass(int size, float fb) : buf(size), feedback(fb), idx(0) {}

float Allpass::process(float x)
{
    if (buf.empty()) return x;
    float y = buf[idx];
    float out = -x + y;
    buf[idx] = x + y * feedback;
    idx = (idx + 1) % buf.size();
    return out;
}

// --------------------- SchroederReverb ---------------------
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
{
    for(int i=0;i<4;i++)
        baseCombSizes[i] = combs[i].buf.size();
    dryMix = 0.5f; wetMix = 0.5f;
}

float SchroederReverb::process(float input)
{
    float s = 0.0f;
    for (auto &c : combs) s += c.process(input);
    for (auto &a : allpasses) s = a.process(s);

    s /= combs.size(); // normalize sum

    float outSample = input * dryMix + s * wetMix;

    // optional soft clipping
    return outSample / (1.0f + std::fabs(outSample));
}

void SchroederReverb::setDryWet(float dry, float wet) { dryMix=dry; wetMix=wet; }

void SchroederReverb::setRoomSize(float size)
{
    for(int i=0;i<4;i++)
    {
        size_t newDelay = std::max<size_t>(1, size_t(baseCombSizes[i] * (0.5f + size)));

        if (newDelay != combs[i].buf.size())
        {
            std::vector<float> old = combs[i].buf;
            combs[i].buf.resize(newDelay, 0.0f);
            size_t copyLen = std::min(old.size(), combs[i].buf.size());
            for(size_t j=0;j<copyLen;j++)
                combs[i].buf[j] = old[(combs[i].idx + j) % old.size()];
            combs[i].idx = 0;
        }

        combs[i].setFeedback(0.7f * size);
    }
}

void SchroederReverb::setDecay(float decay)
{
    for(auto &c : combs) c.setFeedback(decay);
}

// --------------------- WAV loader ---------------------
static bool readLittleEndianUint32(std::ifstream &f, uint32_t &out)
{
    char buf[4];
    if(!f.read(buf,4)) return false;
    out = (uint8_t)buf[0] | ((uint8_t)buf[1]<<8) | ((uint8_t)buf[2]<<16) | ((uint8_t)buf[3]<<24);
    return true;
}

static bool readLittleEndianUint16(std::ifstream &f, uint16_t &out)
{
    char buf[2];
    if(!f.read(buf,2)) return false;
    out = (uint8_t)buf[0] | ((uint8_t)buf[1]<<8);
    return true;
}

static bool loadWavToFloatVector(const std::string &path, std::vector<float> &out)
{
    std::ifstream file(path, std::ios::binary);
    if(!file) return false;

    char riff[4]; file.read(riff,4);
    if(file.gcount()!=4 || std::memcmp(riff,"RIFF",4)!=0) return false;

    uint32_t riffSize; if(!readLittleEndianUint32(file,riffSize)) return false;

    char wave[4]; file.read(wave,4);
    if(file.gcount()!=4 || std::memcmp(wave,"WAVE",4)!=0) return false;

    uint16_t audioFormat=0, numChannels=0, bitsPerSample=0; uint32_t dataChunkPos=0, dataChunkSize=0;

    while(file.good())
    {
        char chunkId[4]; if(!file.read(chunkId,4)) break;
        uint32_t chunkSize; if(!readLittleEndianUint32(file,chunkSize)) break;

        if(std::memcmp(chunkId,"fmt ",4)==0)
        {
            uint16_t af, nc, blockAlign; uint32_t sr, byteRate; uint16_t bps;
            if(!readLittleEndianUint16(file,af)) return false;
            if(!readLittleEndianUint16(file,nc)) return false;
            if(!readLittleEndianUint32(file,sr)) return false;
            if(!readLittleEndianUint32(file,byteRate)) return false;
            if(!readLittleEndianUint16(file,blockAlign)) return false;
            if(!readLittleEndianUint16(file,bps)) return false;
            audioFormat=af; numChannels=nc; bitsPerSample=bps;
            int extra = int(chunkSize) - 16; if(extra>0) file.seekg(extra,std::ios::cur);
        }
        else if(std::memcmp(chunkId,"data",4)==0)
        {
            dataChunkPos = static_cast<uint32_t>(file.tellg());
            dataChunkSize = chunkSize;
            file.seekg(chunkSize,std::ios::cur);
        }
        else file.seekg(chunkSize,std::ios::cur);
    }

    if(dataChunkPos==0 || bitsPerSample==0 || numChannels==0) return false;

    file.clear(); file.seekg(dataChunkPos,std::ios::beg);
    size_t bytesPerSample = bitsPerSample/8;
    size_t frameCount = dataChunkSize / (bytesPerSample*numChannels);

    out.clear(); out.reserve(frameCount);
    for(size_t i=0;i<frameCount;i++)
    {
        double sum=0.0;
        for(uint16_t ch=0;ch<numChannels;ch++)
        {
            if(bitsPerSample==16 && audioFormat==1)
            {
                int16_t s; file.read(reinterpret_cast<char*>(&s),sizeof(int16_t));
                sum += double(s)/32768.0;
            }
            else if(bitsPerSample==32 && audioFormat==1)
            {
                int32_t s; file.read(reinterpret_cast<char*>(&s),sizeof(int32_t));
                sum += double(s)/2147483648.0;
            }
            else if(bitsPerSample==32 && audioFormat==3)
            {
                float f; file.read(reinterpret_cast<char*>(&f),sizeof(float));
                sum += double(f);
            }
            else return false;
        }
        double avg = sum/numChannels;
        if(avg>1.0) avg=1.0; if(avg<-1.0) avg=-1.0;
        out.push_back(static_cast<float>(avg));
    }
    return !out.empty();
}

// --------------------- ConvolutionReverb ---------------------
bool ConvolutionReverb::loadIR(const std::string &path)
{
    std::vector<float> irData;
    if(!loadWavToFloatVector(path, irData))
    {
        std::ifstream file(path,std::ios::binary);
        if(!file) return false;
        file.seekg(0,std::ios::end);
        std::streampos size = file.tellg();
        file.seekg(0,std::ios::beg);
        if(size<=0) return false;
        size_t count = static_cast<size_t>(size/sizeof(float));
        irData.resize(count);
        if(!file.read(reinterpret_cast<char*>(irData.data()),count*sizeof(float))) return false;
    }

    if(irData.empty()) return false;

    size_t blockSize = 1024;
    convolver.reset();
    return convolver.init(blockSize,irData.data(),irData.size());
}

float ConvolutionReverb::process(float input)
{
    float out=0.0f;
    convolver.process(&input,&out,1);
    float outSample = input*dryMix + out*wetMix;
    return outSample/(1.0f+std::fabs(outSample)); // soft clip
}

void ConvolutionReverb::setDryWet(float dry,float wet){ dryMix=dry; wetMix=wet; }

// --------------------- Unified Reverb ---------------------
Reverb::Reverb(int sr)
    : mode(ReverbType::SCHROEDER), simple(0.08f,0.7f,sr), schroeder(sr), sampleRate(sr)
{}

float Reverb::process(float input)
{
    switch(mode)
    {
        case ReverbType::SIMPLE: return simple.process(input);
        case ReverbType::SCHROEDER: return schroeder.process(input);
        case ReverbType::CONVOLUTION: return convolution.process(input);
    }
    return input;
}

void Reverb::setDryWet(float dry,float wet)
{
    simple.setDryWet(dry,wet);
    schroeder.setDryWet(dry,wet);
    convolution.setDryWet(dry,wet);
}

void Reverb::setRoomSize(float size)
{
    simple.setRoomSize(size);
    schroeder.setRoomSize(size);
}

void Reverb::setDecay(float decay)
{
    simple.setDecay(decay);
    schroeder.setDecay(decay);
}

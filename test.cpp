#include "portaudio.h"
#include <stdio.h> // For printf

int main(void) {
    PaError err;
    int numDevices;
    const PaDeviceInfo *deviceInfo;

    err = Pa_Initialize();
    if (err != paNoError) {
        fprintf(stderr, "PortAudio error: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
        fprintf(stderr, "PortAudio error: %s\n", Pa_GetErrorText(numDevices));
        Pa_Terminate();
        return 1;
    }

    printf("PortAudio Devices:\n");
    for (int i = 0; i < numDevices; i++) {
        deviceInfo = Pa_GetDeviceInfo(i);
        if (deviceInfo != NULL) {
            printf("  Device %d: %s\n", i, deviceInfo->name);
            printf("    Host API: %d\n", deviceInfo->hostApi); // You can resolve hostApi to its name if needed
            printf("    Max Input Channels: %d\n", deviceInfo->maxInputChannels);
            printf("    Max Output Channels: %d\n", deviceInfo->maxOutputChannels);
            printf("    Default Sample Rate: %.2f\n", deviceInfo->defaultSampleRate);
            // ... print other relevant info
        }
    }

    err = Pa_Terminate();
    if (err != paNoError) {
        fprintf(stderr, "PortAudio error: %s\n", Pa_GetErrorText(err));
        return 1;
    }

    return 0;
}
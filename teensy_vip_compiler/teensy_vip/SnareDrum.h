#pragma once
#include <Audio.h>

class SnareDrum {
public:
    // Constructor
    SnareDrum();

    // --- Public Methods ---
    void trigger();
    void setParams(float attackMs, float decayMs);
    
    // --- Public "getter" ---
    // Lets the main .ino file connect to the class's output
    AudioEffectEnvelope& getOutput() { return env; }

private:
    // --- Internal Audio Objects ---
    AudioSynthNoiseWhite     noise;
    AudioFilterStateVariable filter;
    AudioEffectEnvelope      env;

    // --- Internal Connections ---
    AudioConnection          c1; // (noise -> filter)
    AudioConnection          c2; // (filter -> env)
};
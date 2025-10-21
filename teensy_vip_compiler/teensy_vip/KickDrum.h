#pragma once
#include <Audio.h>

class KickDrum {
public:
    // Constructor
    KickDrum();

    // --- Public Methods ---
    void trigger();
    void update(); // Handles the pitch sweep
    void setParams(float attackMs, float decayMs);
    
    // --- Public "getter" ---
    // This lets the main .ino file connect to the class's output
    AudioEffectEnvelope& getOutput() { return env; }

private:
    // --- Internal Audio Objects ---
    AudioSynthWaveform       osc;
    AudioEffectEnvelope      env;
    AudioConnection          con; // (osc -> env)

    // --- Internal State ---
    bool        sweepActive;
    uint32_t    sweepStartUs;
    const uint32_t SWEEP_DUR_US = 50000;
    float       fStart;
    float       fEnd;
};
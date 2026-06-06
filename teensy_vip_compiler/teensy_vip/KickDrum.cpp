#include "KickDrum.h"

// Constructor: This is where you initialize the audio connections
// and set default values.
KickDrum::KickDrum() : 
    con(osc, 0, env, 0) // Initialize the connection: osc -> env
{
    osc.begin(WAVEFORM_SINE);
    osc.amplitude(1.0f);
    sweepActive = false;
}

// Set the envelope parameters
void KickDrum::setParams(float attackMs, float decayMs) {
     const float relMs = 8.0f; // Release is constant
     env.attack(attackMs);
     env.decay(decayMs);
     env.sustain(0.0f);
     env.release(relMs);
}

// Trigger the drum sound
void KickDrum::trigger() {
    fStart = 120.0f;
    fEnd   = 45.0f;
    osc.amplitude(1.0f);
    osc.frequency(fStart);
    sweepActive = true;
    sweepStartUs = micros();
    env.noteOn(); // This triggers the envelope
}

// Update the pitch sweep (call this every loop)
void KickDrum::update() {
    if (!sweepActive) return;
    
    uint32_t t = micros() - sweepStartUs;
    
    if (t >= SWEEP_DUR_US) {
        osc.frequency(fEnd);
        sweepActive = false;
    } else {
        float u = (float)t / (float)SWEEP_DUR_US;
        float f = fStart + (fEnd - fStart) * (u * u); // Quadratic sweep
        osc.frequency(f);
    }
}
#include "SnareDrum.h"

// Constructor: Initialize connections and set fixed parameters
SnareDrum::SnareDrum() : 
    c1(noise, 0, filter, 0),
    c2(filter, 1, env, 0)  // Use output 1 for bandpass
{
    // Set the "static" properties of the snare
    noise.amplitude(0.9f);
    filter.frequency(1800.0f);
    filter.resonance(1.5f);
}

// Set the envelope parameters (from pots)
void SnareDrum::setParams(float attackMs, float decayMs) {
     const float relMs = 8.0f; // Constant release time
     env.attack(attackMs);
     env.decay(decayMs);
     env.sustain(0.0f);
     env.release(relMs);
}

// Trigger the drum sound
void SnareDrum::trigger() {
    env.noteOn(); // Just trigger the envelope
}
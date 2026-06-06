#include "Oscillator.h"

// Constructor: Initialize connections and set defaults
Oscillator::Oscillator() : 
    conL(osc, 0, gainL, 0), // Connect osc to Left gain
    conR(osc, 0, gainR, 0)  // Connect osc to Right gain
{
    baseFreqHz = 440.0f;
    detuneCents = 0.0f;
    
    osc.begin(WAVEFORM_SINE);
    setAmp(0.75f); // Set overall amplitude on the oscillator
    setPan(0.0f);  // Set initial pan (center)
    _updateFreq(); // Set initial frequency
}

// --- Public Methods ---

void Oscillator::setFreq(float hz) {
    baseFreqHz = hz;
    _updateFreq();
}

void Oscillator::setDetune(float cents) {
    detuneCents = cents;
    _updateFreq();
}

void Oscillator::setAmp(float amp) {
    // Clamp amplitude 0.0 to 1.0
    if (amp < 0.0f) amp = 0.0f;
    if (amp > 1.0f) amp = 1.0f;
    osc.amplitude(amp);
}

void Oscillator::setPan(float pan) {
    // Clamp pan -1.0 to 1.0
    if (pan < -1.0f) pan = -1.0f;
    if (pan > 1.0f) pan = 1.0f;

    // "Constant Power" panning law
    // This prevents the perceived volume from dipping in the center.
    float panRad = pan * 0.785398f; // (pan * PI/4)
    float gainL = cosf(panRad);
    float gainR = sinf(panRad);
    
    // Apply gains
    this->gainL.gain(gainL * gainL);
    this->gainR.gain(gainR * gainR);
}

void Oscillator::setPhase(float degrees) {
    osc.phase(degrees);
}

void Oscillator::setWaveform(int waveformType) {
    osc.begin(waveformType);
}

// --- Private Helper ---

void Oscillator::_updateFreq() {
    // Cents to frequency multiplier conversion:
    // multiplier = 2 ^ (cents / 1200)
    float finalFreq = baseFreqHz * powf(2.0f, detuneCents / 1200.0f);
    osc.frequency(finalFreq);
}
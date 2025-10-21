#pragma once
#include <Audio.h>
#include <math.h> // For powf

class Oscillator {
public:
    // Constructor
    Oscillator();

    // --- Public Methods ---
    void setFreq(float hz);
    void setDetune(float cents); // Detune in cents (1/100th of a semitone)
    void setAmp(float amp);      // 0.0 to 1.0
    void setPan(float pan);      // -1.0 (Left) to 1.0 (Right)
    void setPhase(float degrees); // 0.0 to 360.0
    
    // Pass in e.g.: WAVEFORM_SINE, WAVEFORM_SQUARE, etc.
    void setWaveform(int waveformType);

    // --- Public "getters" for L/R outputs ---
    // You now have two outputs to connect from.
    AudioEffectMultiply& getOutputL() { return gainL; }
    AudioEffectMultiply& getOutputR() { return gainR; }

private:
    // --- Internal Audio Objects ---
    AudioSynthWaveform    osc;
    AudioEffectMultiply   gainL; // Acts as Left channel gain
    AudioEffectMultiply   gainR; // Acts as Right channel gain

    // --- Internal Connections ---
    AudioConnection       conL; // osc -> gainL
    AudioConnection       conR; // osc -> gainR

    // --- Internal State ---
    float baseFreqHz;
    float detuneCents;

    // --- Private Helper ---
    void _updateFreq(); // Applies base freq + detune
};
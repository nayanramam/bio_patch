/* Teensy 4.1 Drum Test (Kick + Snare) over USB Audio
 *
 * BTN_PLAY -> Kick
 * BTN_TYPE -> Snare
 * POT_A -> Attack (0..40 ms)
 * POT_D -> Decay (20..600 ms)
 *
 * Teensyduino: Tools → USB Type → "Audio" (or "Serial + MIDI + Audio")
 */

#include <Arduino.h>
#include <Audio.h>
#include <math.h>
#include "KickDrum.h"

// ---------------- Controls -------------------
const int BTN_PLAY = 24;   // Kick
const int BTN_TYPE = 25;   // Snare
const int POT_A = A0;      // Attack
const int POT_D = A1;      // Decay

// ---------------- Audio Graph ----------------
// kick: now encapsulated!
KickDrum kick;


// Kick: sine -> env
// AudioSynthWaveform   kickOsc;
// AudioEffectEnvelope  kickEnv;
// AudioConnection      kc1(kickOsc, 0, kickEnv, 0);

// Snare: white noise -> bandpass -> env
AudioSynthNoiseWhite snareNoise;
AudioFilterStateVariable snareFilt;
AudioEffectEnvelope  snareEnv;
AudioConnection      sc1(snareNoise, 0, snareFilt, 0);
AudioConnection      sc2(snareFilt, 1, snareEnv, 0); // bandpass

// Mix -> USB
AudioMixer4          mix;
AudioConnection      mx1(kickEnv, 0, mix, 0);
AudioConnection      mx2(snareEnv, 0, mix, 1);

AudioOutputUSB       usbOut;
AudioConnection      ux1(mix, 0, usbOut, 0);
AudioConnection      ux2(mix, 0, usbOut, 1);

// ---------------- Params ---------------------
struct Params {
  float attackMs;
  float decayMs;
};

// Params kickParams = { 5.0f, 150.0f };
Params snareParams = { 5.0f, 200.0f };

// Kick pitch sweep
bool kickSweepActive = false;
uint32_t kickSweepStartUs = 0;
const uint32_t KICK_SWEEP_DUR_US = 50000;
float kickFStart = 120.0f;
float kickFEnd   = 45.0f;

// Helpers
static inline float fmap(float x, float inMin, float inMax, float outMin, float outMax) {
  if (x < inMin) x = inMin;
  if (x > inMax) x = inMax;
  return outMin + (outMax - outMin) * ((x - inMin) / (inMax - inMin));
}
float mapAttack(uint16_t raw) { return fmap(raw, 0, 4095, 0.0f, 40.0f); }
float mapDecay (uint16_t raw) { return fmap(raw, 0, 4095, 20.0f, 600.0f); }

// ---------------- Envelope apply -------------
void applyEnvelopeParams(float attackMs, float decayMs) {
  const float relMs = 8.0f;
  // kickEnv.attack(attackMs);  kickEnv.decay(decayMs);  kickEnv.sustain(0.0f);  kickEnv.release(relMs);
  snareEnv.attack(attackMs); snareEnv.decay(decayMs); snareEnv.sustain(0.0f); snareEnv.release(relMs);
}

// ---------------- Trigger -------------------
// void triggerKick() {
//   applyEnvelopeParams(kickParams.attackMs, kickParams.decayMs);
//   kickFStart = 120.0f;
//   kickFEnd   = 45.0f;
//   kickOsc.begin(WAVEFORM_SINE);
//   kickOsc.amplitude(1.0f);
//   kickOsc.frequency(kickFStart);
//   kickSweepActive = true;
//   kickSweepStartUs = micros();
//   kickEnv.noteOn();
// }

void triggerSnare() {
  applyEnvelopeParams(snareParams.attackMs, snareParams.decayMs);
  snareNoise.amplitude(0.9f);
  snareFilt.frequency(1800.0f);
  snareFilt.resonance(1.5f);
  snareEnv.noteOn();
}

// ---------------- Kick sweep update ----------
// void updateKickSweep() {
//   if (!kickSweepActive) return;
//   uint32_t t = micros() - kickSweepStartUs;
//   if (t >= KICK_SWEEP_DUR_US) {
//     kickOsc.frequency(kickFEnd);
//     kickSweepActive = false;
//   } else {
//     float u = (float)t / (float)KICK_SWEEP_DUR_US;
//     float f = kickFStart + (kickFEnd - kickFStart) * (u * u);
//     kickOsc.frequency(f);
//   }
// }

// ---------------- Setup ----------------------
void setup() {
  pinMode(BTN_PLAY, INPUT_PULLUP);
  pinMode(BTN_TYPE, INPUT_PULLUP);
  analogReadResolution(12);

  Serial.begin(9600);
  AudioMemory(40);

  mix.gain(0, 0.9f);
  mix.gain(1, 0.8f);

  // kickOsc.begin(WAVEFORM_SINE);
  // kickOsc.amplitude(1.0f);
  snareNoise.amplitude(0.9f);

  Serial.println("Drum Test Ready: Press BTN_PLAY=Kick, BTN_TYPE=Snare");
}

// ---------------- Loop -----------------------
void loop() {
  // Update params from pots
  float currentAttack = mapAttack(analogRead(POT_A));
  float currentDecay  = mapDecay (analogRead(POT_D));
  snareParams.attackMs = kickParams.attackMs;
  snareParams.decayMs  = kickParams.decayMs;

  kick.setParams(currentAttack, currentDecay); // <-- SET PARAMS ON THE OBJECT

  // Button press detection (active LOW)
  static bool lastKick = HIGH, lastSnare = HIGH;
  bool kickNow = digitalRead(BTN_PLAY);
  bool snareNow = digitalRead(BTN_TYPE);

  if (lastKick == HIGH && kickNow == LOW) {
    kick.trigger();
    Serial.println("Kick!");
  }
  if (lastSnare == HIGH && snareNow == LOW) {
    triggerSnare();
    Serial.println("Snare!");
  }

  lastKick = kickNow;
  lastSnare = snareNow;

  // Update kick sweep
  kick.update();
}

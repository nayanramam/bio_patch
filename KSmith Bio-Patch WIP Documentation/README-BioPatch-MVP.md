# BioPatch MVP — README

This repo is the current working state of the **BioPatch MVP**. The idea is to read a biosignal (heart rate for now), detect beats in real time, and use that data in a musical context (visuals, audio, or both).

This is not a finished system. It works, but there’s plenty of space to build on it — adding new sensor types, refining the UI, and working on the code that handles patching and routing in real time.

---

## What’s here (at a glance)

* **RTOS build (primary): `BioPatch-MVP-Code-HR-RTOS.ino`**  
  Two-core build: sampling/beat detection on one core, UI/rendering on the other.

* **NoRTOS build (simpler): `KSMITH_Bio-Patch_MVP_No_RTOS.ino`**  
  Single-loop version that’s easier to read, good for onboarding and debugging.

* **Support/test sketches:**  
  * `Pulse_Display_Visualization_ESP32.ino` — quick way to prove the sensor + OLED path.  
  * `ESP32_No_RTOS_Troubleshoot_Echo.ino` — sanity check serial I/O (host <-> ESP32) and Daisy link.  
  * `ESP32_UART_Test.ino` — minimal UART send path from ESP32.  
  * `Daisy_UART_Test.ino` — Daisy-side UART receive and smoothing test.

* **Enclosure files:**  
  Sub-directory with 3D printing files. These were for the original form factor. The project is moving toward a modular approach, with Daisy Seed as a real-time sonification module. We will most likely need to resize and re-layout before printing.

---

## Hardware / Libraries

* **MCU:** Adafruit Feather ESP32 V2  
* **Display:** SSD1309 128×64 OLED over I²C (U8g2)  
* **Encoder/button:** Adafruit Seesaw I²C encoder (with optional push)  
* **Pulse sensor:** via PulseSensorPlayground  

**Arduino libraries (via Library Manager):**
* `U8g2` (olikraus)  
* `Adafruit Seesaw`  
* `PulseSensor Playground`  

---

## UI & Interaction

**On-device display (OLED):**
* Top HUD shows BPM when valid.  
* Center area shows waveform (about 6.4 s window; sampled at 200 Hz, decimated to ~20 Hz).  
* A small heart icon pulses after each detected beat.  
* Small dots at the lower-right show which channels are active.  

**Controls:**
* **BTN_CH1 / BTN_CH2:** toggle channels on/off (CH1 is heart rate input).  
* **Rotary encoder (turn):** cycle through active channels.  
* **Rotary encoder (hold + turn):** change the sensor type for the current channel (`HR / RESP / GSR / EMG / RAW`).  
* Only HR is wired now, but the UI is already set up to expand later.  

**Serial telemetry (50 Hz, every ~20 ms):**

Format: `BPM BEATFLAG RAW NORM AMP`

* **BPM:** integer estimate (0 if unknown)  
* **BEATFLAG:** `1` on detected beat, `0` otherwise  
* **RAW:** raw ADC value (12-bit)  
* **NORM:** normalized, DC-removed value  
* **AMP:** current autoscale span  

---

## The two main code paths

### 1. `BioPatch-MVP-Code-HR-RTOS.ino` (primary)

**What it’s doing:**
* Samples HR at ~200 Hz.  
* DC removal and autoscaling with slow decay (`DC_ALPHA`, `SCALE_DECAY`).  
* Beat detection via PulseSensor library (`HR_PIN=26`, default `HR_THRESH=1500`).  
* Rolling waveform buffer for display.  
* **Core 0:** sampling, filtering, bpm calculation, telemetry.  
* **Core 1:** inputs, UI rendering, channel selection.  
* Shared state handled with mutexes and snapshots.  

**Why use this one:**  
Stable for demos, already structured for multiple sensors and patching.  

**Where to tweak first:**  
* `HR_THRESH` (sensitivity)  
* IO task settings: `SAMPLE_HZ`, `WAVE_DECIMATE`, `DC_ALPHA`, `SCALE_DECAY`  
* `TX_INTERVAL_MS` for telemetry rate  

**UI loop behavior:**  
* Active channel list rebuilt whenever buttons toggle states.  
* Encoder turns switch the active channel.  
* Encoder hold + turn cycles the sensor type.  
* Changing channel resets autoscale to avoid carry-over artifacts.  

---

### 2. `KSMITH_Bio-Patch_MVP_No_RTOS.ino` (onboarding/debug)

**What it’s doing:**
* Same process (sample → filter → beat detect → display/telemetry) in a single loop.  
* Same UI controls, without RTOS complexity.  

**Why use this first:**
* Easier to read and modify.  
* If the RTOS version behaves oddly, test here to see if it’s hardware vs. scheduling.  

---

## Support sketches

* **`Pulse_Display_Visualization_ESP32.ino`**  
  Quick check that sensor and OLED work.  

* **`ESP32_No_RTOS_Troubleshoot_Echo.ino`**  
  Serial echo test. Confirms USB serial + Daisy link.  

* **`ESP32_UART_Test.ino`**  
  ESP32 → Daisy UART test.  

* **`Daisy_UART_Test.ino`**  
  Daisy-side UART receive + smoothing.  

---

## Enclosure (moving modular)

* Files in the enclosure sub-directory are for the old all-in-one design.  
* The potential direction now is a modular enclosure with Daisy Seed as a drop-in audio module.  
* Expect to resize and reformat for footprint before printing.  
* Plan mounting space for Daisy and I²C cables.  
* Leave room for future TRRS breakouts.  

---

## Where this should go next

* Add more sensors: RESP / GSR / EMG / RAW.  
* Routing/patching: Map telemetry values in Max/PD to MIDI CCs, triggers, etc.  
* UI refinements: Sensor labels, settings pane for threshold/scaling.  
* Stability: Add the ability to dynamically adjust `DC_ALPHA`, `SCALE_DECAY`, `HR_THRESH`.  
* Daisy integration: Keep UART format stable and explore structure for sending values for sonification to Daisy Seed.  

---

## Quick reference

* `HR_PIN=26` — heart rate analog in  
* `HR_THRESH=1500` — sensitivity threshold  
* `SAMPLE_HZ=200` — sampling rate  
* `WAVE_DECIMATE=10` — waveform downsampling  
* `TX_INTERVAL_MS=20` — telemetry rate (~50 Hz)  
* `BTN_CH1=27`, `BTN_CH2=33` — toggle channels  
* Encoder turn = select channel  
* Encoder hold + turn = change sensor type  

---

Start with the NoRTOS build, confirm the strcture, then move to the RTOS version, which is fully working in its curretn state. The code is written to be readable and modifiable — add sensors, try new filters, adjust the UI. Once the modular enclosure is more finalized it should make future hardware iterations easier.
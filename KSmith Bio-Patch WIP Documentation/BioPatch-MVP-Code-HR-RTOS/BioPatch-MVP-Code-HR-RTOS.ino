// ==== Libraries (Library Manager) ====
// U8g2 by olikraus
// Adafruit Seesaw by Adafruit
// PulseSensor Playground by Joel Murphy & WFE

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "Adafruit_seesaw.h"
#include <PulseSensorPlayground.h>

// ==== Pins / Addresses (Feather ESP32 V2 + Glass2) ====
#define I2C_SDA 22
#define I2C_SCL 20
#define OLED_ADDR_7BIT   0x3C
#define SEESAW_ADDR_7BIT 0x36
#define SEESAW_BTN_PIN   24     // encoder pushbutton (I2C via seesaw), active LOW

// Panel buttons (to GND, pullups) → toggle channels
#define BTN_CH1 27
#define BTN_CH2 33

// HR sensor
#define HR_PIN     26           // GPIO26 (ADC1_CH9)
#define HR_THRESH  1500         // tune as needed (ESP32 0..4095)

// ==== Display (SSD1309 128x64) ====
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ==== Rotary (I2C seesaw handled on UI core) ====
Adafruit_seesaw ss;
bool seesaw_btn_supported = false;
int32_t encLast = 0;

// ==== Channels / Sensors ====
static const int NUM_CHANNELS = 5;
enum SensorKind { SENSOR_HR, SENSOR_RESP, SENSOR_GSR, SENSOR_EMG, SENSOR_RAW, SENSOR_COUNT };
const char* SENSOR_NAME[SENSOR_COUNT] = {"HR", "RESP", "GSR", "EMG", "RAW"};

struct Channel {
  bool active = false;
  SensorKind kind = SENSOR_HR;
  int pin = -1;       // only CH1 uses HR_PIN today
};
Channel ch[NUM_CHANNELS];

// Active list + selection (managed on UI core)
int activeList[NUM_CHANNELS];
int activeCount = 0;
int currentActiveIndex = -1;

// Track which channel is currently shown (UI) to request an IO reset
int lastShownChannel = -1;

// ==== Layout ====
static const int MARGIN = 4;              // safe border
static const int WIDTH  = 128;
static const int HEIGHT = 64;
static const int DRAW_W = WIDTH  - 2*MARGIN;
static const int DRAW_H = HEIGHT - 2*MARGIN;
static const int WAVE_W = DRAW_W;         // one pixel per column

// ---- Shared data between tasks (guarded by mutex) ----
volatile int   g_bpm       = 0;
volatile int   g_raw       = 0;
volatile int   g_beatFlag  = 0;    // 1 if beat in last telemetry frame, else 0
volatile float g_norm      = 0.0f; // 0..1 normalized (DC-removed & scaled)
volatile float g_span      = 0.0f; // envelope scale

volatile uint32_t g_lastBeatMs = 0;

// Wave buffer produced by IO, consumed by UI (values are "band Y" 0..(bandH-1))
volatile uint8_t g_waveY[WAVE_W];
volatile int     g_waveWrite = 0;

// IO autoscale reset requested by UI
volatile bool g_ioResetRequested = false;

// Selected channel snapshot for IO to read (managed by UI)
volatile int  g_selIndex = -1;
volatile int  g_selPin   = -1;
volatile int  g_selKind  = SENSOR_HR;
volatile bool g_selActive= false;

// Mutexes
SemaphoreHandle_t g_dataMutex;   // sensor data + wave
SemaphoreHandle_t g_stateMutex;  // channel selection snapshot

// ==== Telemetry (IO task) ====
uint32_t lastTxMs = 0;
const uint32_t TX_INTERVAL_MS = 20; // ~50 Hz

// Latch ensuring BEATFLAG is visible at least once per beat
volatile uint8_t g_beatLatch = 0;

// ==== Task handles ====
TaskHandle_t taskIO  = nullptr; // Core 0
TaskHandle_t taskUI  = nullptr; // Core 1

// ------------------------------------------------------
// Helpers
// ------------------------------------------------------
bool readBtn(uint8_t pin){ return !digitalRead(pin); } // active-low
int wrap(int v,int n){ if(n<=0) return 0; while(v<0)v+=n; while(v>=n)v-=n; return v; }

void publishSelectionSnapshot() {
  if (xSemaphoreTake(g_stateMutex, (TickType_t)2) == pdTRUE) {
    int c = (activeCount==0 || currentActiveIndex<0) ? -1 : activeList[currentActiveIndex];
    g_selIndex  = c;
    g_selActive = (c>=0) ? ch[c].active : false;
    g_selKind   = (c>=0) ? ch[c].kind   : SENSOR_HR;
    g_selPin    = (c>=0) ? ch[c].pin    : -1;
    xSemaphoreGive(g_stateMutex);
  }
}

void rebuildActive() {
  activeCount = 0;
  for (int i=0;i<NUM_CHANNELS;i++) if (ch[i].active) activeList[activeCount++] = i;
  currentActiveIndex = (activeCount==0) ? -1 : wrap(currentActiveIndex<0?0:currentActiveIndex, activeCount);
  publishSelectionSnapshot();
}

// UI heart
void drawHeart(int cx, int cy, int r){
  u8g2.drawDisc(cx - r/2, cy - r/3, r/2);
  u8g2.drawDisc(cx + r/2, cy - r/3, r/2);
  u8g2.drawTriangle(cx - r, cy - r/6, cx + r, cy - r/6, cx, cy + r);
}

void drawEmpty() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(MARGIN, HEIGHT/2, "no sensors toggled : )");
  u8g2.sendBuffer();
}

void snapshotData(int &bpm, int &raw, float &norm, float &span, uint32_t &lbMs, uint8_t *waveLocal, int &widx) {
  if (xSemaphoreTake(g_dataMutex, (TickType_t)2) == pdTRUE) {
    bpm  = g_bpm;
    raw  = g_raw;
    norm = g_norm;
    span = g_span;
    lbMs = g_lastBeatMs;
    int loc = g_waveWrite;
    for (int i=0;i<WAVE_W;i++) waveLocal[i] = g_waveY[i];
    widx = loc;
    xSemaphoreGive(g_dataMutex);
  }
}

// ------------------------------------------------------
// IO Task (Core 0): sampling, BPM, DC removal, autoscale, waveform, serial
// ------------------------------------------------------
void taskIOFn(void*){
  // ===== Tunables for IO smoothing =====
  const int   SAMPLE_HZ     = 200;    // beat detection + capture rate
  const int   WAVE_DECIMATE = 10;     // push 1 of every 10 samples -> ~20 Hz wave
  const float DC_ALPHA      = 0.01f;  // DC removal (0..1), higher = faster
  const float SCALE_DECAY   = 0.005f; // envelope decay per sample
  const float SCALE_FLOOR   = 60.0f;  // minimum scale to avoid "flatlines"

  // ===== ADC + PulseSensor =====
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  PulseSensorPlayground pulse;   // use library’s own ISR sampling
  pulse.analogInput(HR_PIN);     // library needs a default analog pin
  pulse.setThreshold(HR_THRESH);
  pulse.begin();

  // ===== IO state (filtered) =====
  float dc = 0.0f;         // DC offset estimate
  float scale = 200.0f;    // envelope (auto-scale)
  int decim = 0;

  // Init wave / shared data
  if (xSemaphoreTake(g_dataMutex, (TickType_t)portMAX_DELAY) == pdTRUE) {
    for (int i=0;i<WAVE_W;i++) g_waveY[i] = (DRAW_H - 22) / 2;
    g_waveWrite = 0;
    g_bpm = 0; g_raw = 0; g_norm = 0.0f; g_span = scale; g_beatFlag = 0; g_beatLatch = 0;
    xSemaphoreGive(g_dataMutex);
  }

  const int bandH_est = HEIGHT - (MARGIN+14) - (MARGIN+8); // matches UI band
  const TickType_t tick = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
  TickType_t last = xTaskGetTickCount();

  for(;;){
    vTaskDelayUntil(&last, tick);

    // Pick up current selection (channel/pin/type)
    int selIdx, selPin, selKind; bool selActive;
    if (xSemaphoreTake(g_stateMutex, (TickType_t)2) == pdTRUE) {
      selIdx   = g_selIndex;
      selPin   = g_selPin;
      selKind  = g_selKind;
      selActive= g_selActive;
      xSemaphoreGive(g_stateMutex);
    } else {
      selIdx = -1; selPin = -1; selKind = SENSOR_HR; selActive=false;
    }

    // Reset requested by UI (switch channel/type)
    if (g_ioResetRequested) {
      if (xSemaphoreTake(g_dataMutex, (TickType_t)2) == pdTRUE) {
        for (int i=0;i<WAVE_W;i++) g_waveY[i] = (DRAW_H - 22) / 2;
        g_waveWrite = 0;
        g_norm = 0.0f; g_span = 0.0f; g_bpm = 0; g_raw = 0; g_beatFlag = 0; g_beatLatch = 0;
        xSemaphoreGive(g_dataMutex);
      }
      dc = 0.0f; scale = 200.0f; decim = 0;
      g_ioResetRequested = false;
    }

    int raw = 0, bpm = 0;
    int beatThisSample = 0;
    float norm = 0.0f;

    if (selIdx >= 0 && selActive && selKind == SENSOR_HR && selPin >= 0) {
      raw = analogRead(selPin);

      // Beat detection via PulseSensor ISR
      bpm = pulse.getBeatsPerMinute();
      if (pulse.sawStartOfBeat()) {
        beatThisSample = 1;
        g_lastBeatMs = millis();        // OK to write without lock (single writer)
        g_beatLatch = 1;                // latch until telemetered once
      }

      // DC removal (simple one-pole high-pass)
      dc += DC_ALPHA * (raw - dc);
      float ac = raw - dc;

      // Envelope / auto-scale with slow decay
      float mag = fabsf(ac);
      scale = fmaxf(scale * (1.0f - SCALE_DECAY), mag);
      if (scale < SCALE_FLOOR) scale = SCALE_FLOOR;

      // Normalize to 0..1 (map -scale..+scale → 0..1)
      float n = (ac / (scale + 1e-6f)) * 0.5f + 0.5f;
      if (n < 0) n = 0; if (n > 1) n = 1;
      norm = n;

      // Decimate for display waveform (~20 Hz)
      if (++decim >= WAVE_DECIMATE) {
        decim = 0;
        uint8_t yBand = (uint8_t)((1.0f - n) * (bandH_est - 1));
        if (xSemaphoreTake(g_dataMutex, (TickType_t)1) == pdTRUE) {
          g_waveY[g_waveWrite] = yBand;
          g_waveWrite = (g_waveWrite + 1) % WAVE_W;
          xSemaphoreGive(g_dataMutex);
        }
      }
    }

    // Publish sensor values
    if (xSemaphoreTake(g_dataMutex, (TickType_t)1) == pdTRUE) {
      g_raw = raw;
      g_bpm = bpm;
      g_norm = norm;
      g_span = scale;
      // Beat flag: show latched 1 at least once, then clear
      if (g_beatLatch) { g_beatFlag = 1; g_beatLatch = 0; }
      else { g_beatFlag = 0; }
      xSemaphoreGive(g_dataMutex);
    }

    // Telemetry (space-separated): BPM BEATFLAG RAW NORM AMP
    uint32_t now = millis();
    if (now - lastTxMs >= TX_INTERVAL_MS) {
      lastTxMs = now;
      int _bpm, _raw, _bf; float _norm, _span;
      if (xSemaphoreTake(g_dataMutex, (TickType_t)1) == pdTRUE) {
        _bpm = g_bpm; _raw = g_raw; _bf = g_beatFlag; _norm = g_norm; _span = g_span;
        xSemaphoreGive(g_dataMutex);
      } else {
        _bpm = 0; _raw = 0; _bf = 0; _norm = 0.0f; _span = 0.0f;
      }
      Serial.print(_bpm); Serial.print(" ");
      Serial.print(_bf);  Serial.print(" ");
      Serial.print(_raw); Serial.print(" ");
      Serial.print(_norm, 3); Serial.print(" ");
      Serial.println(_span, 1);
    }
  }
}

// ------------------------------------------------------
// UI Task (Core 1): I2C (OLED+seesaw), buttons, encoder, draw
// ------------------------------------------------------
void drawHUD(int chIdx, int bpm) {
  u8g2.setFont(u8g2_font_5x8_tr);
  char left[24]; snprintf(left, sizeof(left), "CH%d  %s", chIdx+1, SENSOR_NAME[ch[chIdx].kind]);
  u8g2.drawStr(MARGIN, MARGIN+8, left);

  if (ch[chIdx].kind == SENSOR_HR && ch[chIdx].pin >= 0) {
    char right[16]; snprintf(right, sizeof(right), "%3dbpm", bpm);
    int rw = u8g2.getStrWidth(right);
    u8g2.drawStr(WIDTH - MARGIN - rw, MARGIN+8, right);
  }
}

void drawHRView(uint8_t *waveLocal, int widx, uint32_t lastBeatMs) {
  const int top    = MARGIN + 14;
  const int bottom = HEIGHT - MARGIN - 8;
  const int bandH  = bottom - top;

  // waveform across inner width (20 Hz samples → ~6.4s window)
  int idx = widx;
  int prevY = top + waveLocal[idx];
  for (int x = 0; x < DRAW_W; ++x) {
    int y = top + waveLocal[idx];
    u8g2.drawLine(MARGIN + (x?x-1:0), prevY, MARGIN + x, y);
    prevY = y;
    idx = (idx + 1) % WAVE_W;
  }

  // heart pulse
  uint32_t dt = millis() - lastBeatMs;
  float pulseAmt = (dt < 160) ? (1.0f - (float)dt/160.0f) : 0.0f;
  int R = 7 + (int)(pulseAmt * 5);
  drawHeart(WIDTH/2, top + bandH/2, R);
}

void drawChannelCard(int chIdx){
  // snapshot sensor data + wave
  int bpm, raw; float norm, span; uint32_t lbMs; uint8_t waveLocal[WAVE_W]; int widx;
  snapshotData(bpm, raw, norm, span, lbMs, waveLocal, widx);

  u8g2.clearBuffer();
  drawHUD(chIdx, bpm);

  if (ch[chIdx].kind == SENSOR_HR && ch[chIdx].pin >= 0) {
    drawHRView(waveLocal, widx, lbMs);
  } else {
    u8g2.setFont(u8g2_font_6x12_tf);
    int tw = u8g2.getStrWidth(SENSOR_NAME[ch[chIdx].kind]);
    u8g2.drawStr((WIDTH - tw)/2, HEIGHT/2, SENSOR_NAME[ch[chIdx].kind]);
  }

  // tiny active dots at bottom right
  int x = WIDTH - MARGIN - 2;
  for (int i=0;i<NUM_CHANNELS;i++){
    if (ch[i].active) { u8g2.drawBox(x-2, HEIGHT - MARGIN - 3, 2, 2); x -= 4; }
  }

  u8g2.sendBuffer();
}

void handlePanelButtonsUI(){
  static bool p1=false, p2=false;
  bool b1 = readBtn(BTN_CH1);
  bool b2 = readBtn(BTN_CH2);

  bool changed = false;
  if (b1 && !p1) { ch[0].active = !ch[0].active; changed = true; }
  if (b2 && !p2) { ch[1].active = !ch[1].active; changed = true; }

  p1 = b1; p2 = b2;

  if (changed) {
    rebuildActive();
    g_ioResetRequested = true;
  }
}

void handleEncoderUI(){
  int32_t pos = ss.getEncoderPosition();
  bool held = seesaw_btn_supported ? (ss.digitalRead(SEESAW_BTN_PIN)==0) : false;
  int32_t d = pos - encLast;
  if (d != 0) {
    if (held) {
      int c = (activeCount==0 || currentActiveIndex<0) ? -1 : activeList[currentActiveIndex];
      if (c >= 0) {
        int step = (d>0)?1:-1;
        int k = (int)ch[c].kind;
        k = wrap(k + step, SENSOR_COUNT);
        ch[c].kind = (SensorKind)k;
        publishSelectionSnapshot();
        g_ioResetRequested = true;
      }
    } else {
      if (activeCount > 0) {
        int prev = currentActiveIndex;
        int step = (d>0)?1:-1;
        currentActiveIndex = wrap(currentActiveIndex + step, activeCount);
        if (currentActiveIndex != prev) {
          publishSelectionSnapshot();
          g_ioResetRequested = true;
        }
      }
    }
    encLast = pos;
  }
}

void taskUIFn(void*){
  // I2C + OLED
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  u8g2.begin();
  u8g2.setI2CAddress(OLED_ADDR_7BIT << 1);
  u8g2.setFlipMode(1);  // rotate 180° for upside-down mount

  // Buttons
  pinMode(BTN_CH1, INPUT_PULLUP);
  pinMode(BTN_CH2, INPUT_PULLUP);

  // Rotary
  if (ss.begin(SEESAW_ADDR_7BIT)) {
    ss.pinMode(SEESAW_BTN_PIN, INPUT_PULLUP);
    seesaw_btn_supported = true;
    encLast = ss.getEncoderPosition();
  }

  // Channels default (all OFF; only CH1 wired to HR)
  for (int i=0;i<NUM_CHANNELS;i++){ ch[i].active=false; ch[i].kind=SENSOR_HR; ch[i].pin=-1; }
  ch[0].pin = HR_PIN;
  rebuildActive();          // builds active list & publishes snapshot

  // Main UI loop
  for(;;){
    handlePanelButtonsUI();
    handleEncoderUI();

    int c = (activeCount==0 || currentActiveIndex<0) ? -1 : activeList[currentActiveIndex];
    if (c != lastShownChannel) {
      lastShownChannel = c;
      g_ioResetRequested = true; // ask IO to reset autoscale + wave
    }

    if (activeCount == 0) {
      drawEmpty();
    } else {
      drawChannelCard(c);
    }

    vTaskDelay(pdMS_TO_TICKS(16)); // ~60 FPS UI
  }
}

// ------------------------------------------------------
// Arduino setup/loop: spawn tasks and idle
// ------------------------------------------------------
void setup(){
  Serial.begin(115200);
  delay(50);

  g_dataMutex  = xSemaphoreCreateMutex();
  g_stateMutex = xSemaphoreCreateMutex();

  xTaskCreatePinnedToCore(taskIOFn, "IO",  6144, NULL, 2, &taskIO,  0); // Core 0
  xTaskCreatePinnedToCore(taskUIFn, "UI",  8192, NULL, 1, &taskUI,  1); // Core 1
}

void loop(){
  vTaskDelay(pdMS_TO_TICKS(1000));
}
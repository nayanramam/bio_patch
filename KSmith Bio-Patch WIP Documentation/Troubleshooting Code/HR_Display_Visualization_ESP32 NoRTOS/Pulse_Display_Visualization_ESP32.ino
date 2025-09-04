// ==== Libraries (Library Manager) ====
// U8g2 by olikraus
// Adafruit Seesaw by Adafruit
// PulseSensor Playground by Joel Murphy & WFE

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

// Panel buttons (to GND, pullups enabled) → toggle channels only
#define BTN_CH1 27
#define BTN_CH2 33

// HR sensor
#define HR_PIN     26           // GPIO26 (ADC1_CH9)
#define HR_THRESH  2000         // tune as needed (ESP32 0..4095)

// ==== Display (SSD1309 128x64) ====
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ==== Rotary ====
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

// Active list + selection
int activeList[NUM_CHANNELS];
int activeCount = 0;
int currentActiveIndex = -1;

// Track which channel is currently shown (to reset wave/scale on switch)
int lastShownChannel = -1;

// ==== HR processing ====
PulseSensorPlayground pulse;
volatile int g_bpm = 0;
volatile int g_raw = 0;
volatile uint32_t g_lastBeatMs = 0;

// ==== Waveform buffer / layout ====
static const int MARGIN = 4;              // safe border (skip flaky edge pixels)
static const int WIDTH  = 128;
static const int HEIGHT = 64;
static const int DRAW_W = WIDTH  - 2*MARGIN;
static const int DRAW_H = HEIGHT - 2*MARGIN;
static const int WAVE_W = DRAW_W;

uint8_t waveY[WAVE_W];
int waveWrite = 0;

// autoscale EMA (per-selected channel)
float emaMin = 4095.0f, emaMax = 0.0f;
const float EMA_A = 0.02f;

// ---- Serial control stream ----
uint32_t lastTxMs = 0;
const uint32_t TX_INTERVAL_MS = 20; // ~50 Hz
float g_norm = 0.0f;                // 0..1 normalized value
float g_span = 0.0f;                // amplitude proxy (emaMax - emaMin)
int g_beatFlag = 0;                 // 1 if beat this frame, else 0

// ==== Small helpers ====
bool readBtn(uint8_t pin){ return !digitalRead(pin); } // active-low
int wrap(int v,int n){ if(n<=0) return 0; while(v<0)v+=n; while(v>=n)v-=n; return v; }

void rebuildActive() {
  activeCount = 0;
  for (int i=0;i<NUM_CHANNELS;i++) if (ch[i].active) activeList[activeCount++] = i;
  currentActiveIndex = (activeCount==0) ? -1 : wrap(currentActiveIndex<0?0:currentActiveIndex, activeCount);
}

int currentChannel() {
  if (activeCount==0 || currentActiveIndex<0) return -1;
  return activeList[currentActiveIndex];
}

// minimalist heart
void drawHeart(int cx, int cy, int r){
  u8g2.drawDisc(cx - r/2, cy - r/3, r/2);
  u8g2.drawDisc(cx + r/2, cy - r/3, r/2);
  u8g2.drawTriangle(cx - r, cy - r/6, cx + r, cy - r/6, cx, cy + r);
}

// ==== UI: minimal screens ====
void drawEmpty() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(MARGIN, HEIGHT/2, "no sensors toggled : )");
  u8g2.sendBuffer();
}

void drawHUD(int chIdx) {
  u8g2.setFont(u8g2_font_5x8_tr);
  char left[24]; snprintf(left, sizeof(left), "CH%d  %s", chIdx+1, SENSOR_NAME[ch[chIdx].kind]);
  u8g2.drawStr(MARGIN, MARGIN+8, left);

  if (ch[chIdx].kind == SENSOR_HR && ch[chIdx].pin >= 0) {
    char right[16]; snprintf(right, sizeof(right), "%3dbpm", g_bpm);
    int rw = u8g2.getStrWidth(right);
    u8g2.drawStr(WIDTH - MARGIN - rw, MARGIN+8, right);
  }
}

void drawHRView() {
  const int top    = MARGIN + 14;
  const int bottom = HEIGHT - MARGIN - 8;
  const int bandH  = bottom - top;

  int idx = waveWrite;
  int prevY = top + waveY[idx];
  for (int x = 0; x < DRAW_W; ++x) {
    int y = top + waveY[idx];
    u8g2.drawLine(MARGIN + (x?x-1:0), prevY, MARGIN + x, y);
    prevY = y;
    idx = (idx + 1) % WAVE_W;
  }

  uint32_t dt = millis() - g_lastBeatMs;
  float pulseAmt = (dt < 160) ? (1.0f - (float)dt/160.0f) : 0.0f;
  int R = 7 + (int)(pulseAmt * 5);
  drawHeart(WIDTH/2, top + bandH/2, R);
}

void drawChannelCard(int chIdx){
  u8g2.clearBuffer();
  drawHUD(chIdx);

  if (ch[chIdx].kind == SENSOR_HR && ch[chIdx].pin >= 0) {
    drawHRView();
  } else {
    u8g2.setFont(u8g2_font_6x12_tf);
    int tw = u8g2.getStrWidth(SENSOR_NAME[ch[chIdx].kind]);
    u8g2.drawStr((WIDTH - tw)/2, HEIGHT/2, SENSOR_NAME[ch[chIdx].kind]);
  }

  int x = WIDTH - MARGIN - 2;
  for (int i=0;i<NUM_CHANNELS;i++){
    if (ch[i].active) { u8g2.drawBox(x-2, HEIGHT - MARGIN - 3, 2, 2); x -= 4; }
  }

  u8g2.sendBuffer();
}

// ==== Input handling ====
void handlePanelButtons(){
  static bool p1=false, p2=false;
  bool b1 = readBtn(BTN_CH1);
  bool b2 = readBtn(BTN_CH2);

  if (b1 && !p1) { ch[0].active = !ch[0].active; rebuildActive(); }
  if (b2 && !p2) { ch[1].active = !ch[1].active; rebuildActive(); }

  p1 = b1; p2 = b2;
}

void handleEncoder(){
  int32_t pos = ss.getEncoderPosition();
  bool held = seesaw_btn_supported ? (ss.digitalRead(SEESAW_BTN_PIN)==0) : false;
  int32_t d = pos - encLast;
  if (d != 0) {
    if (held) {
      int c = currentChannel();
      if (c >= 0) {
        int step = (d>0)?1:-1;
        int k = (int)ch[c].kind;
        k = wrap(k + step, SENSOR_COUNT);
        ch[c].kind = (SensorKind)k;
      }
    } else {
      if (activeCount > 0) {
        int step = (d>0)?1:-1;
        currentActiveIndex = wrap(currentActiveIndex + step, activeCount);
      }
    }
    encLast = pos;
  }
}

// ==== Setup / Loop ====
void setup(){
  Serial.begin(115200);
  delay(50);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  u8g2.begin();
  u8g2.setI2CAddress(OLED_ADDR_7BIT << 1);
  u8g2.setFlipMode(1);  // rotate 180° for upside-down mount

  pinMode(BTN_CH1, INPUT_PULLUP);
  pinMode(BTN_CH2, INPUT_PULLUP);

  if (ss.begin(SEESAW_ADDR_7BIT)) {
    ss.pinMode(SEESAW_BTN_PIN, INPUT_PULLUP);
    seesaw_btn_supported = true;
    encLast = ss.getEncoderPosition();
  }

  for (int i=0;i<NUM_CHANNELS;i++){ ch[i].active=false; ch[i].kind=SENSOR_HR; ch[i].pin=-1; }
  ch[0].pin = HR_PIN;
  rebuildActive();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pulse.analogInput(HR_PIN);
  pulse.setThreshold(HR_THRESH);
  pulse.begin();

  for (int i=0;i<WAVE_W;i++) waveY[i] = (DRAW_H - 22) / 2;
}

void loop(){
  handlePanelButtons();
  handleEncoder();

  int c = currentChannel();
  if (c != lastShownChannel) {
    lastShownChannel = c;
    emaMin = 4095.0f;
    emaMax = 0.0f;
    for (int i = 0; i < WAVE_W; ++i) waveY[i] = (DRAW_H - 22) / 2;
    waveWrite = 0;
  }

  g_beatFlag = 0; // reset each loop

  if (c >= 0 && ch[c].active && ch[c].kind == SENSOR_HR && ch[c].pin >= 0) {
    int raw = analogRead(ch[c].pin);
    int bpm = pulse.getBeatsPerMinute();
    if (pulse.sawStartOfBeat()) {
      g_lastBeatMs = millis();
      g_beatFlag = 1;
    }
    g_raw = raw; g_bpm = bpm;

    emaMin = (raw < emaMin) ? raw : (1.0f-EMA_A)*emaMin + EMA_A*raw;
    emaMax = (raw > emaMax) ? raw : (1.0f-EMA_A)*emaMax + EMA_A*raw;
    float span = emaMax - emaMin;
    if (span < 80.0f) { emaMin = raw - 40.0f; emaMax = raw + 40.0f; span = 80.0f; }
    g_span = span;

    float norm = (raw - emaMin) / span; if (norm<0) norm=0; if (norm>1) norm=1;
    g_norm = norm;

    const int top = MARGIN + 14;
    const int bottom = HEIGHT - MARGIN - 8;
    const int bandH = bottom - top;
    uint8_t y = (uint8_t)((1.0f - norm) * (bandH - 1));
    waveY[waveWrite] = y;
    waveWrite = (waveWrite + 1) % WAVE_W;
  }

  if (activeCount == 0) {
    drawEmpty();
  } else {
    drawChannelCard(c);
  }

  uint32_t now = millis();
  if (now - lastTxMs >= TX_INTERVAL_MS) {
    lastTxMs = now;
    // space-separated values: BPM BEATFLAG RAW NORM AMP
    Serial.print(g_bpm);
    Serial.print(" ");
    Serial.print(g_beatFlag);
    Serial.print(" ");
    Serial.print(g_raw);
    Serial.print(" ");
    Serial.print(g_norm, 3);
    Serial.print(" ");
    Serial.println(g_span, 1);
  }

  delay(16); // ~60 FPS UI
}
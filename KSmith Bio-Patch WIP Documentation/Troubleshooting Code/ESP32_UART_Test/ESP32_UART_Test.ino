// ===== Libraries (Library Manager) =====
// - "U8g2" by olikraus

#include <Wire.h>
#include <U8g2lib.h>

// ===== Hardware config (Adafruit ESP32 Feather V2 + Glass2 OLED) =====
#define I2C_SDA 22
#define I2C_SCL 20
#define OLED_ADDR_7BIT 0x3C     // Glass2 default
#define PULSE_PIN 26            // your wiring: GPIO26 (ADC1_CH9)

// ===== OLED (SSD1309 128x64 over I2C) =====
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// If your panel prefers another init sequence, swap to:
// U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ===== Graph settings =====
static const int   WIDTH  = 128;
static const int   HEIGHT = 64;
static const float LPF_A  = 0.35f;   // low-pass smoothing alpha (0..1); higher = smoother
static const float EMA_A  = 0.02f;   // auto-scale tracking speed for min/max
static const int   FPS    = 50;      // target refresh rate (Hz)

// ring buffer for waveform (one column per pixel)
uint16_t ring[WIDTH];
int writeIdx = 0;

// dynamic range tracking for auto-scale
float emaMin = 4095.0f;
float emaMax = 0.0f;

// timing
uint32_t lastMs = 0;
uint32_t frameIntervalMs = 1000 / FPS;

// simple low-pass filter state
float filt = 0.0f;

void drawGrid() {
  // faint grid: vertical lines every 16 px, horizontal lines every 16 px
  for (int x = 0; x < WIDTH; x += 16) u8g2.drawVLine(x, 0, HEIGHT);
  for (int y = 0; y < HEIGHT; y += 16) u8g2.drawHLine(0, y, WIDTH);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // ADC setup (ESP32)
  analogReadResolution(12);           // 0..4095
  analogSetAttenuation(ADC_11db);     // widen measurable range

  // OLED
  u8g2.begin();
  u8g2.setI2CAddress(OLED_ADDR_7BIT << 1);

  // init ring buffer
  for (int i = 0; i < WIDTH; ++i) ring[i] = HEIGHT / 2;
  filt = analogRead(PULSE_PIN);

  // small splash
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(0, 12, "Pulse Scope - GPIO26");
  u8g2.drawStr(0, 28, "Auto-scale, ~50 FPS");
  u8g2.sendBuffer();
  delay(600);
}

void loop() {
  // frame pacing
  uint32_t now = millis();
  if (now - lastMs < frameIntervalMs) return;
  lastMs = now;

  // read + smooth
  int raw = analogRead(PULSE_PIN);         // 0..4095
  filt = filt + LPF_A * (raw - filt);      // simple 1-pole LPF

  // update dynamic min/max (EMA)
  if (filt < emaMin) emaMin = filt;
  else               emaMin = (1.0f - EMA_A) * emaMin + EMA_A * filt;

  if (filt > emaMax) emaMax = filt;
  else               emaMax = (1.0f - EMA_A) * emaMax + EMA_A * filt;

  // avoid collapse if signal is very flat
  float span = emaMax - emaMin;
  if (span < 40.0f) { // minimal span to keep visible
    emaMin = filt - 20.0f;
    emaMax = filt + 20.0f;
    span   = 40.0f;
  }

  // scale to screen (invert Y so higher value plots higher)
  float norm = (filt - emaMin) / span;          // 0..1
  if (norm < 0) norm = 0; if (norm > 1) norm = 1;
  uint16_t y = (uint16_t)((1.0f - norm) * (HEIGHT - 1));  // 0..63

  // write into ring and advance
  ring[writeIdx] = y;
  writeIdx = (writeIdx + 1) % WIDTH;

  // draw
  u8g2.clearBuffer();
  drawGrid();

  // connect points across the whole width, oldest->newest left->right
  // ring[writeIdx] is the newest sample's "next" position; start at the oldest
  int idx = writeIdx;
  int prevY = ring[idx];
  for (int x = 0; x < WIDTH; ++x) {
    int curY = ring[idx];
    u8g2.drawLine(x ? x - 1 : 0, prevY, x, curY);
    prevY = curY;
    idx = (idx + 1) % WIDTH;
  }

  // small HUD with current raw and scaled range
  u8g2.setFont(u8g2_font_5x8_tr);
  char buf[40];
  snprintf(buf, sizeof(buf), "RAW:%4d  min:%4.0f max:%4.0f", raw, emaMin, emaMax);
  u8g2.drawStr(0, 62, buf);

  u8g2.sendBuffer();
}
HardwareSerial DaisySerial(USART1); // Daisy TX = pin 36, RX = pin 37

const int windowSize = 10;
int readings[windowSize];
int index_ = 0;
int total = 0;

void setup() {
  DaisySerial.begin(9600);
  for (int i = 0; i < windowSize; i++) readings[i] = 0;
}

void loop() {
  if (DaisySerial.available()) {
    String input = DaisySerial.readStringUntil('\n');
    input.trim();
    int raw = input.toInt();

    // Moving average smoothing
    total -= readings[index_];
    readings[index_] = raw;
    total += readings[index_];
    index_ = (index_ + 1) % windowSize;
    int average = total / windowSize;

    // Send smoothed value back
    DaisySerial.println(average);
  }
}
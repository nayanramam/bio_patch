#define DaisyRxTx Serial1

uint8_t valueToSend = 0;

void setup() {
  Serial.begin(115200);
  DaisyRxTx.begin(9600);
  Serial.println("Sending data to Daisy & listening for response...");
}

void loop() {
  // 1. Send changing data to the Daisy
  DaisyRxTx.write(valueToSend);
  Serial.print("To Daisy: ");
  Serial.println(valueToSend);

  valueToSend++;  // Increment value for next send

  // 2. Listen for Daisy's response
  while (DaisyRxTx.available()) {
    uint8_t received = DaisyRxTx.read();
    Serial.print("From Daisy: ");
    Serial.println(received);
  }

  delay(100);  // Slow down for readability
}
#include <Arduino.h>

// ==== Configuration ====
const int controlPin = 10;
const float vRef = 5.0;             // Supply ref (approx) if not using mV API
const int adcMax = 4095;            // 12-bit ADC math
const float sensitivity = 0.8;      // 0.8 V per 1A
const float midVoltage = vRef / 2;  // midpoint = 0A

const int analogPins[]  = {A0, A1, A2, A3, A4, A5};
const char* pinNames[]  = {"A0","A1","A2","A3","A4","A5"};

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(9600);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  
  // Force 12-bit resolution on UNO R4
  analogReadResolution(12);

  Serial.println("=== Current Monitor (UNO R4, 12-bit) ===");
  Serial.println("Send '1' → Output 8 ON");
  Serial.println("Send '0' → Output 8 OFF");
  Serial.println("=========================================");
}

static int readStableAnalog(int pin) {
  // Throw away first read after channel switch, allow mux to settle
  analogRead(pin);
  delayMicroseconds(10);
  return analogRead(pin);
}

void loop() {
  // Print once per second
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    for (int i = 0; i < 6; i++) {
      int raw = readStableAnalog(analogPins[i]);

      // If available on your core, prefer analogReadMilliVolts:
      // long mV = analogReadMilliVolts(analogPins[i]);
      // float voltage = mV / 1000.0f;

      float voltage = (raw * vRef) / adcMax;
      float current = (voltage - midVoltage) / sensitivity;

      Serial.print(pinNames[i]);
      Serial.print(": ");
      Serial.print(raw);
      Serial.print(" (");
      Serial.print(voltage, 3);
      Serial.print(" V)  I≈ ");
      Serial.print(current, 2);
      Serial.print(" A\t");
    }
    Serial.println();
  }

  // Serial control for pin 8
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      digitalWrite(controlPin, HIGH);
      Serial.print("Output ");
      Serial.print(controlPin);
      Serial.println(": ON");
    } else if (cmd == '0') {
      digitalWrite(controlPin, LOW);
      Serial.println("Output ");
      Serial.print(controlPin);
      Serial.println(": OFF");
    }
  }
}

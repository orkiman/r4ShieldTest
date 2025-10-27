#include <Arduino.h>

// --- Pinout Configuration (Please adjust to your hardware) ---
// this mapping is for the error connected boards version
const int GUN_PWM_PIN = 11;      // PWM pin to control the gun driver
const int CURRENT_SENSE_PIN = A5; // Analog pin for current measurement

// --- Conversion Constants (Calibrate these for your sensor) ---
// This assumes a sensor where output voltage is proportional to current.
// For Arduino R4, ADC is 12-bit (0-4095) and Vref is 5V.
// Example: If your sensor gives 1V per Amp (1000mV / 1000mA = 1mV/mA), then:
// ADC_TO_MA = (5000.0 / 4095.0) / (1.0 mV/mA) = 1.221
const float ADC_TO_MA = 1.221;

// --- Control Variables (with default values) ---
float dot_rate_hz = 50.0;
float start_current_ma = 800.0;
float hold_current_ma = 400.0;
float hold_threshold_ma = 30.0;
unsigned long dot_size_us = 2000;

// --- State Machine ---
enum GunState { 
  IDLE,
  RAMPING_TO_START,
  HOLDING_CURRENT,
};
GunState gun_state = IDLE;
bool is_running_dots = false; // For 'run' command

// --- Timers ---
unsigned long dot_start_time_us = 0;
unsigned long next_dot_time_ms = 0;

// --- Function Prototypes ---
void parseSerialCommand(String cmd);
void updateGun();
int readCurrent();
void printHelp();

void setup() {
  Serial.begin(115200);
  while (!Serial) { 
    ; // Wait for serial port to connect.
  }

  // must have for glue controller board!!!
  pinMode(8, OUTPUT); digitalWrite(8,LOW);
  pinMode(9, OUTPUT); digitalWrite(9,LOW);
  pinMode(10, OUTPUT); digitalWrite(10,LOW);
  pinMode(11, OUTPUT); digitalWrite(11,LOW);


  pinMode(GUN_PWM_PIN, OUTPUT);
  analogWrite(GUN_PWM_PIN, 0); // Ensure gun is off

  pinMode(CURRENT_SENSE_PIN, INPUT);
  analogReadResolution(12); // Set ADC to 12-bit for R4

  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    parseSerialCommand(command);
  }

  updateGun();
}

void printHelp() {
  Serial.println(F("\n--- Glue Gun Test Controller ---"));
  Serial.println(F("Commands:"));
  Serial.println(F("  dot        - Fire a single dot"));
  Serial.println(F("  run        - Fire dots at the specified rate"));
  Serial.println(F("  s          - Stop firing"));
  Serial.println(F("  rate <hz>  - Set dot rate (e.g., 'rate 50')"));
  Serial.println(F("  startA <ma>- Set start current (e.g., 'startA 800')"));
  Serial.println(F("  holdA <ma> - Set hold current (e.g., 'holdA 400')"));
  Serial.println(F("  holdTh <ma>- Set hold threshold (e.g., 'holdTh 30')"));
  Serial.println(F("  dotSize <us>- Set dot max time (e.g., 'dotSize 2000')"));
  Serial.println(F("  help       - Show this menu"));
  Serial.println(F("--------------------------------"));
}

void parseSerialCommand(String cmd) {
  if (cmd.length() == 0) return;

  if (cmd == "dot") {
    Serial.println(F("CMD: Firing single dot."));
    is_running_dots = false;
    if (gun_state == IDLE) {
      gun_state = RAMPING_TO_START;
      dot_start_time_us = micros();
    }
  } else if (cmd == "run") {
    Serial.println(F("CMD: Running dots."));
    is_running_dots = true;
    next_dot_time_ms = millis(); // Start immediately
  } else if (cmd == "s") {
    Serial.println(F("CMD: Stopping."));
    is_running_dots = false;
    gun_state = IDLE;
    analogWrite(GUN_PWM_PIN, 0);
  } else if (cmd.startsWith("rate ")) {
    dot_rate_hz = cmd.substring(5).toFloat();
    Serial.print(F("PARAM: Dot rate set to "));
    Serial.print(dot_rate_hz);
    Serial.println(F(" Hz"));
  } else if (cmd.startsWith("startA ")) {
    start_current_ma = cmd.substring(7).toFloat();
    Serial.print(F("PARAM: Start current set to "));
    Serial.print(start_current_ma);
    Serial.println(F(" mA"));
  } else if (cmd.startsWith("holdA ")) {
    hold_current_ma = cmd.substring(6).toFloat();
    Serial.print(F("PARAM: Hold current set to "));
    Serial.print(hold_current_ma);
    Serial.println(F(" mA"));
  } else if (cmd.startsWith("holdTh ")) {
    hold_threshold_ma = cmd.substring(7).toFloat();
    Serial.print(F("PARAM: Hold threshold set to "));
    Serial.print(hold_threshold_ma);
    Serial.println(F(" mA"));
  } else if (cmd.startsWith("dotSize ")) {
    dot_size_us = cmd.substring(8).toInt();
    Serial.print(F("PARAM: Dot size set to "));
    Serial.print(dot_size_us);
    Serial.println(F(" us"));
  } else if (cmd == "help") {
    printHelp();
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(cmd);
  }
}

void updateGun() {
  // --- Continuous run logic ---
  if (is_running_dots && gun_state == IDLE) {
    if (millis() >= next_dot_time_ms) {
      gun_state = RAMPING_TO_START;
      dot_start_time_us = micros();
      
      // Schedule next dot
      float period_ms = 1000.0 / dot_rate_hz;
      next_dot_time_ms = millis() + (unsigned long)period_ms;
    }
  }

  if (gun_state == IDLE) {
    analogWrite(GUN_PWM_PIN, 0);
    return;
  }

  // --- Universal dot timeout ---
  if (micros() - dot_start_time_us > dot_size_us) {
    gun_state = IDLE;
    analogWrite(GUN_PWM_PIN, 0);
    //Serial.println(F("Debug: Dot timed out."));
    return;
  }

  // --- State Machine Logic ---
  int current_ma = readCurrent();

  switch (gun_state) {
    case RAMPING_TO_START:
      // Apply full power to ramp up current quickly
      analogWrite(GUN_PWM_PIN, 255);
      if (current_ma >= start_current_ma) {
        gun_state = HOLDING_CURRENT;
        //Serial.println(F("Debug: Reached start current, switching to hold."));
      }
      break;

    case HOLDING_CURRENT:
      // Hysteresis control to maintain hold current
      if (current_ma > (hold_current_ma + hold_threshold_ma)) {
        // Current is too high, turn off PWM
        analogWrite(GUN_PWM_PIN, 0);
      } else if (current_ma < (hold_current_ma - hold_threshold_ma)) {
        // Current is too low, turn on PWM
        analogWrite(GUN_PWM_PIN, 255);
      }
      // If inside the deadband, do nothing to prevent oscillation.
      break;
    
    case IDLE:
      // Should already be handled, but as a safeguard
      analogWrite(GUN_PWM_PIN, 0);
      break;
  }
}

int readCurrent() {
  int adc_value = analogRead(CURRENT_SENSE_PIN);
  return (int)(adc_value * ADC_TO_MA);
}
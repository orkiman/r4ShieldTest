#ifndef GLUECONTROLLER_H
#define GLUECONTROLLER_H

#include <Arduino.h>
#include <ArduinoJson.h>

// ===== Protocol =====
const char STX = 0x02;
const char ETX = 0x03;

// ===== Pins =====
const int ENCODER_PIN_A = 2;
const int SENSOR_PIN    = 4;
const int GUN_PINS[4]   = {8, 9, 10, 11};
const int OUTPUT_CURRENT_PINS[4] = {A0, A1, A2, A3};
// Optional physical test inputs (active LOW)
const int TEST_INPUT_PINS[4] = {3, 5, 6, 7};
const int STATUS_LED    = 13;

// ===== ADC scale (10-bit on R3, 12-bit on R4) =====
#if defined(__AVR__)
  #define ADC_MAX 1023
#else
  #define ADC_MAX 4095
#endif

// Deadband used to block a new fire if the previous pulse current is still flowing.
// Historically 80 on a 10-bit ADC => scale proportionally for 12-bit.
#define INITIATION_DEADBAND_COUNTS ((int)((80.0 / 1023.0) * (ADC_MAX)))

// ===== Dot thresholds (Amps) =====
const double SMALL_DOT_THRESHOLD  = 0.7;
const double MEDIUM_DOT_THRESHOLD = 0.9;
const double LARGE_DOT_THRESHOLD  = 1.1;

// ===== Limits =====
#define MAX_ZONES_PER_GUN 32

// ===== Data =====
struct GlueRow {
  int from;   // pulses (with sensor offset baked-in)
  int to;     // pulses
  int space;  // pulses; 0 = continuous
};

struct GunConfig { bool enabled = true; };

struct ControllerConfig {
  String type = "dots";
  bool   enabled = false;
  double encoderPulsesPerMm = 1.0;
  int    sensorOffset = 10;            // mm
  int    sensorOffsetInPulses = 0;     // pulses
  double startCurrent = 1.0;           // A
  double startDuration = 500;          // ms
  double holdCurrent  = 0.5;           // A
  double minimumSpeed = 0.0;           // mm/s (0 = disabled)
  String dotSize = "medium";           // "small"|"medium"|"large"
};

struct ActiveZone {
  int from, to, next, space;
};

struct ZoneRing {
  ActiveZone buf[MAX_ZONES_PER_GUN];
  uint8_t head = 0, tail = 0, count = 0;
};

// ===== Globals =====
extern ControllerConfig config;
extern GunConfig guns[4];

extern volatile long encoderCount;
extern int currentPosition;

extern int pageLength;
extern bool isCalibrating;
extern unsigned long lastHeartbeat;

extern bool gunStates[4];
extern bool allFiringZonesInserted;
extern int  firingBasePosition;
extern int  currentThreshold;        // 0..ADC_MAX

extern ZoneRing firingZones[4];

// Latest ADC readings (native resolution)
#if defined(__AVR__)
  extern volatile uint16_t gunCurrentADC10[4]; // 0..1023
#else
  extern volatile uint16_t gunCurrentADC12[4]; // 0..4095
#endif

// ===== API =====
void setup();
void loop();

void processSerial();
void handleConfig(const JsonObject& json);
void handleTest(const JsonObject& json);
void initCalibration(const JsonObject& json);
void handleCalibrationSensorStateChange(bool sensorState);
void handleHeartbeat(const JsonObject& json); // no-op

void sendCalibrationResult(int pulses);

void updateGuns();
void calculateFiringZones();
void checkSensor();
void shutdownAllGuns();

// Enc ISR
void encoderISR();

// R4 ADC sampler
void startADCSampler(uint32_t totalSampleRateHz);

#endif // GLUECONTROLLER_H

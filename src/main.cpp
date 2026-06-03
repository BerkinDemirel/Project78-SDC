// SDC Kart Controller — ESP32
//
// Controls steering (closed-loop P with pot feedback) and
// brake (open-loop timed) via two BTS7960B motor drivers.
// Throttle is a DAC voltage on GPIO25.
//
// Wire it up like this:
//   Steer right PWM  → GPIO19
//   Steer left  PWM  → GPIO18
//   Steer pot        → GPIO34  (ADC1 only — ADC2 conflicts with WiFi)
//   Brake right PWM  → GPIO22
//   Brake left  PWM  → GPIO21
//   Throttle DAC     → GPIO25  (0V = off, ~1.94V = running)
//
// EN pins on the BTS7960B boards should be tied HIGH in hardware.
//
// Requires ArduinoJson (Library Manager) and arduino-esp32 framework.

#include <Arduino.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"

// ── Pins ──────────────────────────────────────────────────────────────────────

#define STEER_RPWM    19
#define STEER_LPWM    18
#define STEER_POT     34

#define BRAKE_RPWM    22
#define BRAKE_LPWM    21

#define THROTTLE_DAC  25   // DAC1 — 0 when braking/stopped, 150 (~1.94V) when running

// ── LEDC (PWM) ────────────────────────────────────────────────────────────────
// ESP32 uses the LEDC peripheral instead of analogWrite().
// Each motor driver input needs its own channel. 20 kHz keeps it inaudible.

#define LEDC_STEER_R  0
#define LEDC_STEER_L  1
#define LEDC_BRAKE_R  2
#define LEDC_BRAKE_L  3
#define LEDC_FREQ     20000
#define LEDC_RES      8     // 8-bit, so values are 0-255

static inline void pwmWrite(uint8_t ch, int val) {
  ledcWrite(ch, (uint32_t)constrain(val, 0, 255));
}

// ── Tuning ────────────────────────────────────────────────────────────────────
// Pot values are in 10-bit units (0-1023). The ESP32 ADC is 12-bit,
// so every analogRead() is shifted right by 2 to match.

int STEER_POT_MIN = 0;
int STEER_POT_CTR = 150;
int STEER_POT_MAX = 300;

#define STEER_DEADBAND   8
#define STEER_PWM_MIN    60
#define STEER_PWM_MAX    200
#define STEER_KP         1.8f   // increase if sluggish, decrease if oscillating

#define BRAKE_PWM_MIN    80
#define BRAKE_PWM_MAX    220
#define BRAKE_APPLY_MS   600    // ms for a full apply stroke
#define BRAKE_RELEASE_MS 600    // ms for a full release stroke

#define TELEM_MS         100    // telemetry interval

// Calibration
#define CAL_STEP_POT     12     // pot units per probe step
#define CAL_STEP_WAIT_MS 600    // settle time per step
#define CAL_STALL_THRESH 20     // shortfall that means we've hit the end stop
#define CAL_SAFETY_POT   20     // back off this many units from the end stop

#define WDT_TIMEOUT_S    3

// ── State ─────────────────────────────────────────────────────────────────────

struct SteerState {
  int           targetPos  = 150;
  int           currentPos = 150;
  bool          enabled    = true;
  unsigned long lastCmdMs  = 0;
} steer;

struct BrakeState {
  float current  = 0.0f;
  float target   = 1.0f;   // boot safe: fully braked
  bool  estopped = false;
  bool  started  = false;   // DAC stays at 0 until {"cmd":"start"} is received
} brk;

// ── Throttle ──────────────────────────────────────────────────────────────────
// Call this any time brake or started state changes.
// Motor gets ~1.94V only when explicitly started, not braking, and not e-stopped.

void updateThrottle() {
  bool running = brk.started && !brk.estopped && (brk.current <= 0.02f);
  dacWrite(THROTTLE_DAC, running ? 150 : 0);
}

// ── Steering ──────────────────────────────────────────────────────────────────

// Map a normalised angle (-1.0 = full left, +1.0 = full right) to a pot value.
// Two-segment because the mechanical centre is rarely the midpoint of min/max.
int angleToPot(float f) {
  f = constrain(f, -1.0f, 1.0f);
  if (f >= 0.0f)
    return (int)(STEER_POT_CTR + f * (STEER_POT_MAX - STEER_POT_CTR));
  else
    return (int)(STEER_POT_CTR + f * (STEER_POT_CTR - STEER_POT_MIN));
}

void updateSteering() {
  if (!steer.enabled) return;
  steer.currentPos = analogRead(STEER_POT) >> 2;
  int error = steer.targetPos - steer.currentPos;
  if (abs(error) <= STEER_DEADBAND) {
    pwmWrite(LEDC_STEER_R, 0);
    pwmWrite(LEDC_STEER_L, 0);
    return;
  }
  int pwm = constrain((int)(abs(error) * STEER_KP), STEER_PWM_MIN, STEER_PWM_MAX);
  if (error > 0) { pwmWrite(LEDC_STEER_L, 0); pwmWrite(LEDC_STEER_R, pwm); }
  else           { pwmWrite(LEDC_STEER_R, 0); pwmWrite(LEDC_STEER_L, pwm); }
}

// ── Brake ─────────────────────────────────────────────────────────────────────

void stopBrake() {
  pwmWrite(LEDC_BRAKE_R, 0);
  pwmWrite(LEDC_BRAKE_L, 0);
}

// Drive the actuator for a fixed time then stop.
// Pets the watchdog throughout so we don't reset mid-stroke.
void runBrake(bool extend, int pwm, unsigned long ms) {
  if (ms == 0) return;
  if (extend) { pwmWrite(LEDC_BRAKE_L, 0); pwmWrite(LEDC_BRAKE_R, pwm); }
  else        { pwmWrite(LEDC_BRAKE_R, 0); pwmWrite(LEDC_BRAKE_L, pwm); }
  unsigned long start = millis();
  while (millis() - start < ms) {
    esp_task_wdt_reset();
    while (Serial.available()) Serial.read();
  }
  stopBrake();
}

// Move toward brk.target. Does nothing if we're already close enough.
void applyBrakeTarget() {
  if (brk.estopped) return;
  float delta = brk.target - brk.current;
  if (abs(delta) < 0.02f) return;
  int pwm = constrain(
    (int)map((long)(abs(delta) * 1000), 0, 1000, BRAKE_PWM_MIN, BRAKE_PWM_MAX),
    BRAKE_PWM_MIN, BRAKE_PWM_MAX);
  if (delta > 0) runBrake(true,  pwm, (unsigned long)( delta * BRAKE_APPLY_MS));
  else           runBrake(false, pwm, (unsigned long)(-delta * BRAKE_RELEASE_MS));
  brk.current = brk.target;
  updateThrottle();
}

// Full brake as fast as possible. Latches — requires {"cmd":"release"} to clear.
void fullBrake() {
  float d = 1.0f - brk.current;
  if (d >= 0.02f) runBrake(true, BRAKE_PWM_MAX, (unsigned long)(d * BRAKE_APPLY_MS));
  brk.current = brk.target = 1.0f;
  updateThrottle();
}

// Release brake completely.
void releaseBrake() {
  float d = brk.current;
  if (d >= 0.02f) runBrake(false, BRAKE_PWM_MAX, (unsigned long)(d * BRAKE_RELEASE_MS));
  brk.current = brk.target = 0.0f;
  updateThrottle();
}

// ── Calibration ───────────────────────────────────────────────────────────────
// Creeps left until the motor stalls against the end stop, records the limit,
// then does the same going right. Sets centre as the midpoint and parks there.
//
// Adjust if needed:
//   CAL_STALL_THRESH  — raise if it stops too early, lower if it grinds into the stop
//   CAL_STEP_WAIT_MS  — raise if the motor doesn't settle in time
//   CAL_SAFETY_POT    — clearance to keep from the end stop

void calibrateSteer() {
  int pos = analogRead(STEER_POT) >> 2;
  { JsonDocument d; d["cal"] = "start"; d["pos"] = pos; serializeJson(d, Serial); Serial.println(); }

  for (;;) {
    int probe = pos - 50;
    steer.targetPos = probe;
    unsigned long t = millis();
    while (millis() - t < CAL_STEP_WAIT_MS) {
      esp_task_wdt_reset();
      updateSteering();
      while (Serial.available()) Serial.read();
    }
    int actual    = analogRead(STEER_POT) >> 2;
    int shortfall = abs(probe - actual);
    { JsonDocument d; d["cal"] = "left"; d["probe"] = probe; d["pot"] = actual; d["short"] = shortfall; serializeJson(d, Serial); Serial.println(); }
    if (shortfall >= CAL_STALL_THRESH) break;
    pos = actual;
  }
  int rawLeft = analogRead(STEER_POT) >> 2;
  int newMin  = rawLeft + CAL_SAFETY_POT;
  { JsonDocument d; d["cal"] = "left_limit"; d["raw"] = rawLeft; d["min"] = newMin; serializeJson(d, Serial); Serial.println(); }

  steer.targetPos = newMin + 100;
  { unsigned long t = millis(); while (millis() - t < CAL_STEP_WAIT_MS * 4) { esp_task_wdt_reset(); updateSteering(); while (Serial.available()) Serial.read(); } }
  pos = analogRead(STEER_POT) >> 2;

  for (;;) {
    int probe = pos + 50;
    steer.targetPos = probe;
    unsigned long t = millis();
    while (millis() - t < CAL_STEP_WAIT_MS) {
      esp_task_wdt_reset();
      updateSteering();
      while (Serial.available()) Serial.read();
    }
    int actual    = analogRead(STEER_POT) >> 2;
    int shortfall = abs(probe - actual);
    { JsonDocument d; d["cal"] = "right"; d["probe"] = probe; d["pot"] = actual; d["short"] = shortfall; serializeJson(d, Serial); Serial.println(); }
    if (shortfall >= CAL_STALL_THRESH) break;
    pos = actual;
  }
  int rawRight = analogRead(STEER_POT) >> 2;
  int newMax   = rawRight - CAL_SAFETY_POT;
  { JsonDocument d; d["cal"] = "right_limit"; d["raw"] = rawRight; d["max"] = newMax; serializeJson(d, Serial); Serial.println(); }

  int newCtr    = (newMin + newMax) / 2;
  STEER_POT_MIN = newMin;
  STEER_POT_MAX = newMax;
  STEER_POT_CTR = newCtr;

  steer.targetPos = newCtr;
  { unsigned long t = millis(); while (millis() - t < CAL_STEP_WAIT_MS * 4) { esp_task_wdt_reset(); updateSteering(); while (Serial.available()) Serial.read(); } }

  { JsonDocument d; d["cal"] = "done"; d["min"] = newMin; d["ctr"] = newCtr; d["max"] = newMax; serializeJson(d, Serial); Serial.println(); }
}

// ── Serial protocol ───────────────────────────────────────────────────────────
//
// Send newline-terminated JSON. Commands:
//   {"cmd":"start"}                             enable throttle (required after boot or e-stop)
//   {"cmd":"steer","value":0.35}                steer — -1.0 full left, +1.0 full right
//   {"cmd":"brake","value":0.8}                 brake — 0.0 off, 1.0 fully on
//   {"cmd":"release"}                           release brake and clear e-stop
//   {"cmd":"estop"}                             full brake, latched; clears started flag
//   {"cmd":"brake_reset"}                       reset brake estimate to 0 without moving
//   {"cmd":"enable","axis":"steer","on":true}   enable or disable steering motor
//   {"cmd":"calibrate"}                         run steering calibration
//
// Telemetry out every 100 ms:
//   {"sp":606,"st":606,"bf":0.00,"bt":0.00,"es":false,"go":false}
//   sp=steer pot  st=steer target  bf=brake pos  bt=brake target  es=estopped  go=started

void processSerial() {
  static char buf[128];
  static int  idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        JsonDocument doc;
        if (!deserializeJson(doc, buf)) {
          const char* cmd = doc["cmd"];

          if (strcmp(cmd, "start") == 0) {
            brk.started = true;
            updateThrottle();
            Serial.println("{\"start\":true}");

          } else if (strcmp(cmd, "steer") == 0) {
            steer.targetPos = angleToPot((float)doc["value"]);
            steer.lastCmdMs = millis();

          } else if (strcmp(cmd, "brake") == 0) {
            if (!brk.estopped)
              brk.target = constrain((float)doc["value"], 0.0f, 1.0f);

          } else if (strcmp(cmd, "release") == 0) {
            brk.estopped = false;
            releaseBrake();
            Serial.println("{\"release\":true}");

          } else if (strcmp(cmd, "estop") == 0) {
            brk.estopped = true;
            brk.started  = false;
            fullBrake();
            Serial.println("{\"estop\":true}");

          } else if (strcmp(cmd, "brake_reset") == 0) {
            brk.current  = 0.0f;
            brk.target   = 0.0f;
            brk.estopped = false;
            updateThrottle();
            Serial.println("{\"brake_reset\":true}");

          } else if (strcmp(cmd, "enable") == 0) {
            if (strcmp((const char*)doc["axis"], "steer") == 0) {
              steer.enabled = (bool)doc["on"];
            }

          } else if (strcmp(cmd, "calibrate") == 0) {
            calibrateSteer();
            Serial.println("{\"calibrate\":true}");
          }
        }
      }
      idx = 0;
    } else if (idx < 127) {
      buf[idx++] = c;
    }
  }
}

void sendTelemetry() {
  JsonDocument doc;
  doc["sp"] = steer.currentPos;
  doc["st"] = steer.targetPos;
  doc["bf"] = serialized(String(brk.current, 2));
  doc["bt"] = serialized(String(brk.target,  2));
  doc["es"] = brk.estopped;
  doc["go"] = brk.started;
  serializeJson(doc, Serial);
  Serial.println();
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  Serial.begin(115200);

  ledcSetup(LEDC_STEER_R, LEDC_FREQ, LEDC_RES); ledcAttachPin(STEER_RPWM, LEDC_STEER_R);
  ledcSetup(LEDC_STEER_L, LEDC_FREQ, LEDC_RES); ledcAttachPin(STEER_LPWM, LEDC_STEER_L);
  ledcSetup(LEDC_BRAKE_R, LEDC_FREQ, LEDC_RES); ledcAttachPin(BRAKE_RPWM, LEDC_BRAKE_R);
  ledcSetup(LEDC_BRAKE_L, LEDC_FREQ, LEDC_RES); ledcAttachPin(BRAKE_LPWM, LEDC_BRAKE_L);

  pwmWrite(LEDC_STEER_R, 0);
  pwmWrite(LEDC_STEER_L, 0);
  stopBrake();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  dacWrite(THROTTLE_DAC, 0);

  steer.currentPos = analogRead(STEER_POT) >> 2;
  steer.targetPos  = steer.currentPos;

  calibrateSteer();

  esp_task_wdt_reset();
  Serial.println("{\"status\":\"ready\"}");
}

// ── Loop ──────────────────────────────────────────────────────────────────────

void loop() {
  esp_task_wdt_reset();
  processSerial();
  updateSteering();
  applyBrakeTarget();

  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= TELEM_MS) {
    sendTelemetry();
    lastTelem = millis();
  }
}
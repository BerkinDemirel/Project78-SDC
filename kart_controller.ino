// ─────────────────────────────────────────────
// SDC Kart Controller
// Single Arduino — Steering + Brake
//
// Steering : BTS7960B + potentiometer feedback (closed-loop P)
// Brake    : BTS7960B open-loop timed (no pot)
//
// Requires: ArduinoJson (Library Manager)
// ─────────────────────────────────────────────

#include <avr/wdt.h>
#include <ArduinoJson.h>

// ═════════════════════════════════════════════
// PIN CONFIG
// ═════════════════════════════════════════════

#define STEER_RPWM  5
#define STEER_LPWM  6
#define STEER_R_EN  7
#define STEER_L_EN  8
#define STEER_POT   A0

#define BRAKE_RPWM  9
#define BRAKE_LPWM  10
#define BRAKE_R_EN  11
#define BRAKE_L_EN  12

// ═════════════════════════════════════════════
// TUNING
// ═════════════════════════════════════════════

// Steering
#define STEER_POT_MIN     0
#define STEER_POT_MAX     1024
#define STEER_DEADBAND    8
#define STEER_PWM_MIN     60
#define STEER_PWM_MAX     200
#define STEER_KP          1.8f
#define STEER_TIMEOUT_MS  500

// Brake (open-loop, no pot)
#define BRAKE_PWM_MIN      80
#define BRAKE_PWM_MAX      220
#define BRAKE_APPLY_MS     600   // ms to travel fully released → fully applied
#define BRAKE_RELEASE_MS   600   // ms to travel fully applied → fully released
#define BRAKE_TIMEOUT_MS   500   // watchdog: no command → full brake

// Telemetry interval
#define TELEM_MS  100

// ═════════════════════════════════════════════
// STATE
// ═════════════════════════════════════════════

struct SteerState {
  int   targetPos   = 512;
  int   currentPos  = 512;
  bool  enabled     = true;
  unsigned long lastCmdMs = 0;
} steer;

struct BrakeState {
  float current       = 0.0f;   // open-loop estimate
  float target        = 1.0f;   // boot safe: full brake
  bool  estopped      = false;
  bool  watchdogArmed = false;
  unsigned long lastCmdMs = 0;
} brk;

// ═════════════════════════════════════════════
// STEERING
// ═════════════════════════════════════════════

int angleToPot(float f) {
  f = constrain(f, -1.0f, 1.0f);
  return (int)map((long)(f * 1000), -1000, 1000, STEER_POT_MIN, STEER_POT_MAX);
}

void updateSteering() {
  if (!steer.enabled) return;
  steer.currentPos = analogRead(STEER_POT);
  int error = steer.targetPos - steer.currentPos;
  if (abs(error) <= STEER_DEADBAND) {
    analogWrite(STEER_RPWM, 0);
    analogWrite(STEER_LPWM, 0);
    return;
  }
  int pwm = constrain((int)(abs(error) * STEER_KP), STEER_PWM_MIN, STEER_PWM_MAX);
  if (error > 0) { analogWrite(STEER_LPWM, 0); analogWrite(STEER_RPWM, pwm); }
  else           { analogWrite(STEER_RPWM, 0); analogWrite(STEER_LPWM, pwm); }
}

// ═════════════════════════════════════════════
// BRAKE
// ═════════════════════════════════════════════

void stopBrake() {
  analogWrite(BRAKE_RPWM, 0);
  analogWrite(BRAKE_LPWM, 0);
}

// Blocking actuator run — pets watchdog and drains serial throughout
void runBrake(bool extend, int pwm, unsigned long ms) {
  if (extend) { analogWrite(BRAKE_LPWM, 0); analogWrite(BRAKE_RPWM, pwm); }
  else        { analogWrite(BRAKE_RPWM, 0); analogWrite(BRAKE_LPWM, pwm); }
  unsigned long start = millis();
  while (millis() - start < ms) {
    wdt_reset();
    while (Serial.available()) Serial.read();
  }
  stopBrake();
}

void applyBrakeTarget() {
  if (brk.estopped) return;
  float delta = brk.target - brk.current;
  if (abs(delta) < 0.02f) return;
  int pwm = constrain(
    (int)map((long)(abs(brk.target) * 1000), 0, 1000, BRAKE_PWM_MIN, BRAKE_PWM_MAX),
    BRAKE_PWM_MIN, BRAKE_PWM_MAX);
  if (delta > 0) runBrake(true,  pwm, (unsigned long)(delta       * BRAKE_APPLY_MS));
  else           runBrake(false, pwm, (unsigned long)(abs(delta)  * BRAKE_RELEASE_MS));
  brk.current = brk.target;
}

void fullBrake() {
  float d = 1.0f - brk.current;
  if (d >= 0.02f) runBrake(true, BRAKE_PWM_MAX, (unsigned long)(d * BRAKE_APPLY_MS));
  brk.current = brk.target = 1.0f;
}

void releaseBrake() {
  float d = brk.current;
  if (d >= 0.02f) runBrake(false, BRAKE_PWM_MAX, (unsigned long)(d * BRAKE_RELEASE_MS));
  brk.current = brk.target = 0.0f;
}

// ═════════════════════════════════════════════
// SERIAL PROTOCOL
// ═════════════════════════════════════════════
//
// Commands IN (newline-terminated JSON):
//   {"cmd":"steer","value":0.35}        // -1.0 left … +1.0 right
//   {"cmd":"brake","value":0.8}         // 0.0 released … 1.0 full
//   {"cmd":"release"}                   // fully release brake
//   {"cmd":"estop"}                     // stop steering + full brake
//   {"cmd":"ping"}                      // keepalive for brake watchdog
//   {"cmd":"enable","axis":"steer","on":true}
//
// Telemetry OUT (every TELEM_MS):
//   {"sp":512,"st":600,"bf":0.0,"es":false}
//   sp=steer_pot  st=steer_target  bf=brake_fraction  es=estopped

void processSerial() {
  static char buf[128];
  static int  idx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      if (idx > 0) {
        StaticJsonDocument<128> doc;
        if (!deserializeJson(doc, buf)) {
          const char* cmd = doc["cmd"];

          if (strcmp(cmd, "steer") == 0) {
            steer.targetPos  = angleToPot((float)doc["value"]);
            steer.lastCmdMs  = millis();

          } else if (strcmp(cmd, "brake") == 0) {
            brk.target       = constrain((float)doc["value"], 0.0f, 1.0f);
            brk.lastCmdMs    = millis();
            brk.watchdogArmed = true;

          } else if (strcmp(cmd, "release") == 0) {
            brk.estopped  = false;
            brk.target    = 0.0f;
            brk.lastCmdMs = millis();
            brk.watchdogArmed = true;

          } else if (strcmp(cmd, "estop") == 0) {
            brk.estopped = true;
            fullBrake();
            Serial.println("{\"estop\":true}");

          } else if (strcmp(cmd, "ping") == 0) {
            brk.lastCmdMs    = millis();
            brk.watchdogArmed = true;

          } else if (strcmp(cmd, "enable") == 0) {
            if (strcmp((const char*)doc["axis"], "steer") == 0) {
              steer.enabled = (bool)doc["on"];
              digitalWrite(STEER_R_EN, steer.enabled ? HIGH : LOW);
              digitalWrite(STEER_L_EN, steer.enabled ? HIGH : LOW);
            }
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
  StaticJsonDocument<96> doc;
  doc["sp"] = steer.currentPos;
  doc["st"] = steer.targetPos;
  doc["bf"] = brk.current;
  doc["es"] = brk.estopped;
  serializeJson(doc, Serial);
  Serial.println();
}

// ═════════════════════════════════════════════
// SETUP
// ═════════════════════════════════════════════

void setup() {
  wdt_enable(WDTO_2S);
  Serial.begin(115200);

  pinMode(STEER_RPWM, OUTPUT); pinMode(STEER_LPWM, OUTPUT);
  pinMode(STEER_R_EN, OUTPUT); pinMode(STEER_L_EN, OUTPUT);
  digitalWrite(STEER_R_EN, HIGH); digitalWrite(STEER_L_EN, HIGH);
  analogWrite(STEER_RPWM, 0);  analogWrite(STEER_LPWM, 0);

  pinMode(BRAKE_RPWM, OUTPUT); pinMode(BRAKE_LPWM, OUTPUT);
  pinMode(BRAKE_R_EN, OUTPUT); pinMode(BRAKE_L_EN, OUTPUT);
  digitalWrite(BRAKE_R_EN, HIGH); digitalWrite(BRAKE_L_EN, HIGH);
  stopBrake();

  // Read initial steer position and hold it
  steer.currentPos = analogRead(STEER_POT);
  steer.targetPos  = steer.currentPos;

  // Boot safe: full brake — Jetson must send {"cmd":"release"} to move
  fullBrake();

  wdt_reset();
  Serial.println("{\"status\":\"ready\",\"brake\":\"applied\"}");
}

// ═════════════════════════════════════════════
// LOOP
// ═════════════════════════════════════════════

void loop() {
  wdt_reset();
  processSerial();

  // Brake watchdog: no command for too long → full brake
  if (brk.watchdogArmed &&
      !brk.estopped &&
      (millis() - brk.lastCmdMs) > BRAKE_TIMEOUT_MS) {
    brk.estopped = true;
    Serial.println("{\"watchdog\":true}");
    fullBrake();
  }

  updateSteering();
  applyBrakeTarget();

  static unsigned long lastTelem = 0;
  if (millis() - lastTelem >= TELEM_MS) {
    sendTelemetry();
    lastTelem = millis();
  }
}

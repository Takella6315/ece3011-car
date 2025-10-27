#include <Arduino.h>

// ===== User tunables =====
static constexpr float CDR_MAX_SPEED_MPS = 0.90f;
static constexpr float CDR_MIN_SPEED_MPS = 0.30f;
static constexpr float DEFAULT_FWD_SPEED  = 0.50f;
static constexpr float MIN_TURN_RADIUS_M  = 0.50f;
static constexpr float TRACK_WIDTH_M      = 0.130f; // wheel track (m)

// Convert m/s -> 8-bit PWM (0..255)
static inline uint8_t pwmFromSpeed(float v_mps) {
  if (v_mps < 0) v_mps = 0;
  if (v_mps > CDR_MAX_SPEED_MPS) v_mps = CDR_MAX_SPEED_MPS;
  return (uint8_t)roundf(255.0f * (v_mps / CDR_MAX_SPEED_MPS));
}

static constexpr uint8_t  DUTY_SLEW_PER_TICK = 6;
static constexpr uint32_t CONTROL_PERIOD_MS  = 10; // 100 Hz

// ====== PIN MAP (ESP32-C6 → TB6612FNG) ======
static constexpr uint8_t AIN1 = 2;  // IO2: Left DIR1
static constexpr uint8_t AIN2 = 3;  // IO3: Left DIR2
static constexpr uint8_t PWMA = 4;  // IO4: Left PWM
static constexpr uint8_t BIN1 = 5;  // IO5: Right DIR1
static constexpr uint8_t BIN2 = 6;  // IO6: Right DIR2
static constexpr uint8_t PWMB = 7;  // IO7: Right PWM
static constexpr uint8_t STBY = 8;  // IO8: Standby (HIGH=enable)

// ===== State =====
enum class Command { IDLE, FWD, LEFT, RIGHT, HALT };
static volatile Command cmd = Command::IDLE;
static bool running = false;

static uint8_t dutyL = 0, dutyR = 0; // 0..255 (slew limited)

// ===== Low-level: DIR + PWM per motor =====
static inline void setDirPWM(uint8_t in1, uint8_t in2, uint8_t pwmPin, int signedDuty) {
  // signedDuty: -255..255
  signedDuty = constrain(signedDuty, -255, 255);
  bool forward = (signedDuty >= 0);
  uint8_t mag = (uint8_t)abs(signedDuty);

  // TB6612FNG truth table: forward = IN1=H, IN2=L; reverse = IN1=L, IN2=H
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW  : HIGH);
  analogWrite(pwmPin, mag);
}

static inline void slewTo(uint8_t &cur, int target) {
  target = constrain(target, 0, 255);
  int d = target - (int)cur;
  if (d >  (int)DUTY_SLEW_PER_TICK) d =  DUTY_SLEW_PER_TICK;
  if (d < -(int)DUTY_SLEW_PER_TICK) d = -DUTY_SLEW_PER_TICK;
  cur = (uint8_t)((int)cur + d);
}

// Signed duties: forward=+, reverse=-
void setWheelDutySigned(int leftDuty, int rightDuty) {
  // Slew limit magnitudes (we keep sign after)
  uint8_t tgtL = (uint8_t)abs(constrain(leftDuty,  -255, 255));
  uint8_t tgtR = (uint8_t)abs(constrain(rightDuty, -255, 255));

  // slew current mags toward targets
  uint8_t tmpL = dutyL, tmpR = dutyR;
  slewTo(tmpL, tgtL);
  slewTo(tmpR, tgtR);
  dutyL = tmpL; dutyR = tmpR;

  // apply with direction
  setDirPWM(AIN1, AIN2, PWMA, (leftDuty  >= 0) ? dutyL : -((int)dutyL));
  setDirPWM(BIN1, BIN2, PWMB, (rightDuty >= 0) ? dutyR : -((int)dutyR));
}

void setWheelSpeeds(float vL, float vR) {
  int dL = (int)pwmFromSpeed(fabsf(vL));
  int dR = (int)pwmFromSpeed(fabsf(vR));
  if (vL < 0) dL = -dL;
  if (vR < 0) dR = -dR;
  setWheelDutySigned(dL, dR);
}

// ===== Kinematics =====
void kinematicTurn(float v_forward_mps, float radius_m, bool leftTurn) {
  radius_m = max(0.0f, radius_m);
  const float W = TRACK_WIDTH_M;

  if (radius_m == 0.0f) { // in-place spin
    float v = min(CDR_MAX_SPEED_MPS*0.5f, max(CDR_MIN_SPEED_MPS*0.5f, 0.3f));
    float vL =  leftTurn ? -v :  v;
    float vR =  leftTurn ?  v : -v;
    setWheelSpeeds(vL, vR);
    return;
  }

  float v = min(CDR_MAX_SPEED_MPS, max(CDR_MIN_SPEED_MPS, v_forward_mps));
  float twoR = 2.0f * radius_m;
  float vL = v * (twoR - W) / twoR;
  float vR = v * (twoR + W) / twoR;
  if (!leftTurn) { float t = vL; vL = vR; vR = t; }
  setWheelSpeeds(vL, vR);
}

// ===== Public API =====
void start() { running = true; digitalWrite(STBY, HIGH); } // enable driver
void stop()  { running = false; setWheelDutySigned(0,0); digitalWrite(STBY, LOW); } // brake/standby
void moveForward(float speed_mps = DEFAULT_FWD_SPEED) {
  speed_mps = min(CDR_MAX_SPEED_MPS, max(CDR_MIN_SPEED_MPS, speed_mps));
  digitalWrite(STBY, HIGH);
  setWheelSpeeds(speed_mps, speed_mps);
}
void turnLeft (float fwd = DEFAULT_FWD_SPEED, float R = MIN_TURN_RADIUS_M) {
  digitalWrite(STBY, HIGH);
  kinematicTurn(fwd, R, true);
}
void turnRight(float fwd = DEFAULT_FWD_SPEED, float R = MIN_TURN_RADIUS_M) {
  digitalWrite(STBY, HIGH);
  kinematicTurn(fwd, R, false);
}

// ===== Test harness =====
uint32_t lastStepMs = 0;
int demoStep = 0;
bool demoEnabled = true;

void handleSerial() {
  while (Serial.available()) {
    demoEnabled = false;
    char c = (char)Serial.read();
    switch (c) {
      case 'f': case 'F': start(); cmd = Command::FWD;   Serial.println("[CMD] Forward"); break;
      case 'l': case 'L': start(); cmd = Command::LEFT;  Serial.println("[CMD] Left");    break;
      case 'r': case 'R': start(); cmd = Command::RIGHT; Serial.println("[CMD] Right");   break;
      case 's': case 'S': cmd = Command::HALT; stop();   Serial.println("[CMD] Stop");    break;
      default: break;
    }
  }
}

void runDemo() {
  uint32_t now = millis();
  uint32_t dwell = (demoStep % 4 == 3) ? 1000 : 2000; // stop=1s, others=2s
  if (now - lastStepMs < dwell) return;
  lastStepMs = now;
  switch (demoStep % 4) {
    case 0: start(); moveForward(DEFAULT_FWD_SPEED);                   Serial.println("[DEMO] Forward"); break;
    case 1: start(); turnLeft (DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M);  Serial.println("[DEMO] Left");    break;
    case 2: start(); turnRight(DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M);  Serial.println("[DEMO] Right");   break;
    case 3: stop();                                                    Serial.println("[DEMO] Stop");    break;
  }
  demoStep++;
}

// ===== Setup/loop =====
void setup() {
  // Per-pin PWM config on ESP32-C6
  const uint8_t  RES  = 8;      // 8-bit (0..255)
  const uint32_t FREQ = 20000;  // 20 kHz
  analogWriteResolution(PWMA, RES);
  analogWriteResolution(PWMB, RES);
  analogWriteFrequency(PWMA, FREQ);
  analogWriteFrequency(PWMB, FREQ);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  stop(); // standby low, motors off

  Serial.begin(115200);
  delay(200);
  Serial.println("\nESP32-C6 Motion (TB6612FNG DIR+PWM) Ready. Keys: f=forward, l=left, r=right, s=stop");
}

void loop() {
  handleSerial();

  if (demoEnabled) {
    runDemo();
  } else {
    switch (cmd) {
      case Command::FWD:   moveForward(DEFAULT_FWD_SPEED); break;
      case Command::LEFT:  turnLeft(DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M); break;
      case Command::RIGHT: turnRight(DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M); break;
      case Command::HALT:  stop(); break;
      case Command::IDLE:  /* hold */ break;
    }
  }

  delay(CONTROL_PERIOD_MS);
}

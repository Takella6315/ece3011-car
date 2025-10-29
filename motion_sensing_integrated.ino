#include <Arduino.h>

/* ================== CONFIG ================== */
// Motion targets
static constexpr float CDR_MAX_SPEED_MPS = 0.90f;
static constexpr float CDR_MIN_SPEED_MPS = 0.30f;
static constexpr float DEFAULT_FWD_SPEED  = 0.50f;   // tune to meet your CDR spec
static constexpr float MIN_TURN_RADIUS_M  = 0.50f;   // set 0.0f to pivot in place
static constexpr float TRACK_WIDTH_M      = 0.130f;  // wheel track (m)

// IR sensors (ADC) — LEFT/CENTER/RIGHT  (per your pinout)
static constexpr int PIN_LEFT   = 4; // ADC1_CH4
static constexpr int PIN_CENTER = 2; // ADC1_CH2
static constexpr int PIN_RIGHT  = 3; // ADC1_CH3

// Thresholds (tune on your surface using Serial prints)
static int THRESH     = 900;   // >THRESH => BLACK (12-bit ADC)
static int DEAD_BAND  = 80;    // hysteresis to reduce chatter
static int AVG_N      = 8;     // averaging samples

// Motor driver: IN1..IN4 board (your DRV8833 breakout)
static constexpr uint8_t IN1 = 5;   // Left A  (GPIO5)
static constexpr uint8_t IN2 = 6;   // Left B  (GPIO6)
static constexpr uint8_t IN3 = 21;  // Right A (GPIO21)
static constexpr uint8_t IN4 = 22;  // Right B (GPIO22)

// Control loop
static constexpr uint8_t  DUTY_SLEW_PER_TICK = 6;   // smoothing
static constexpr uint32_t CONTROL_PERIOD_MS  = 10;  // 100 Hz

/* ============== TYPES & INTERNALS ============== */
enum class Command { IDLE, FWD, LEFT, RIGHT, HALT };

struct Sense {
  bool Lb, Cb, Rb;
  int vL, vC, vR;
};

static Command lastCmd = Command::FWD;
static uint8_t dutyL_A = 0, dutyL_B = 0, dutyR_A = 0, dutyR_B = 0; // 0..255

static inline uint8_t pwmFromSpeed(float v_mps) {
  if (v_mps < 0) v_mps = 0;
  if (v_mps > CDR_MAX_SPEED_MPS) v_mps = CDR_MAX_SPEED_MPS;
  return (uint8_t)roundf(255.0f * (v_mps / CDR_MAX_SPEED_MPS));
}

static inline void slewTo(uint8_t &cur, int target) {
  target = constrain(target, 0, 255);
  int d = target - (int)cur;
  if (d >  (int)DUTY_SLEW_PER_TICK) d =  DUTY_SLEW_PER_TICK;
  if (d < -(int)DUTY_SLEW_PER_TICK) d = -DUTY_SLEW_PER_TICK;
  cur = (uint8_t)((int)cur + d);
}

/* ============== LOW-LEVEL DRIVE (IN1..IN4) ============== */
static inline void writeDuty(uint8_t pin, uint8_t duty) { analogWrite(pin, duty); }

static void driveRaw(int lA, int lB, int rA, int rB) { // each 0..255
  slewTo(dutyL_A, lA); slewTo(dutyL_B, lB);
  slewTo(dutyR_A, rA); slewTo(dutyR_B, rB);
  writeDuty(IN1, dutyL_A);
  writeDuty(IN2, dutyL_B);
  writeDuty(IN3, dutyR_A);
  writeDuty(IN4, dutyR_B);
}

// Signed duties: forward = +, reverse = -
static void setWheelDutySigned(int leftDuty, int rightDuty) { // -255..255
  leftDuty  = constrain(leftDuty,  -255, 255);
  rightDuty = constrain(rightDuty, -255, 255);
  int lA = leftDuty  > 0 ? leftDuty  : 0;   // left forward on IN1
  int lB = leftDuty  < 0 ? -leftDuty : 0;   // left reverse on IN2
  int rA = rightDuty > 0 ? rightDuty : 0;   // right forward on IN3
  int rB = rightDuty < 0 ? -rightDuty: 0;   // right reverse on IN4
  driveRaw(lA, lB, rA, rB);
}

static void setWheelSpeeds(float vL, float vR) {
  int dL = (int)pwmFromSpeed(fabsf(vL));
  int dR = (int)pwmFromSpeed(fabsf(vR));
  if (vL < 0) dL = -dL;
  if (vR < 0) dR = -dR;
  setWheelDutySigned(dL, dR);
}

/* ============== MOTION API ============== */
static void moveForward(float speed_mps = DEFAULT_FWD_SPEED) {
  speed_mps = min(CDR_MAX_SPEED_MPS, max(CDR_MIN_SPEED_MPS, speed_mps));
  setWheelSpeeds(speed_mps, speed_mps);
}

static void kinematicTurn(float v_forward_mps, float radius_m, bool leftTurn) {
  radius_m = max(0.0f, radius_m);
  const float W = TRACK_WIDTH_M;

  if (radius_m == 0.0f) { // pivot in place
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

static inline void turnLeft (float fwd = DEFAULT_FWD_SPEED, float R = MIN_TURN_RADIUS_M)  { kinematicTurn(fwd, R, true);  }
static inline void turnRight(float fwd = DEFAULT_FWD_SPEED, float R = MIN_TURN_RADIUS_M)  { kinematicTurn(fwd, R, false); }

/* ============== SENSING ============== */
static int readAvg(int pin, int N=AVG_N) {
  long s = 0;
  for (int i=0;i<N;++i) s += analogRead(pin);
  return (int)(s / N);
}

static Sense readSensors() {
  Sense S;
  S.vL = readAvg(PIN_LEFT);
  S.vC = readAvg(PIN_CENTER);
  S.vR = readAvg(PIN_RIGHT);
  // hysteresis around THRESH
  S.Lb = (S.vL > THRESH + DEAD_BAND);
  S.Cb = (S.vC > THRESH + DEAD_BAND);
  S.Rb = (S.vR > THRESH + DEAD_BAND);
  return S;
}

static Command decide(const Sense& S) {
  if (S.Cb)              return Command::FWD;    // centered → straight
  if (S.Rb && !S.Lb)     return Command::RIGHT;  // line to right → steer right
  if (S.Lb && !S.Rb)     return Command::LEFT;   // line to left  → steer left
  if (S.Lb && S.Rb)      return Command::FWD;    // intersection → prefer straight
  return lastCmd;                                  // lost → keep last correction
}

/* ============== SETUP / LOOP ============== */
void setup() {
  // PWM setup on all four motor pins (ESP32-C6 per-pin API)
  const uint8_t  RES  = 8;      // 0..255
  const uint32_t FREQ = 20000;  // 20 kHz
  analogWriteResolution(IN1, RES);
  analogWriteResolution(IN2, RES);
  analogWriteResolution(IN3, RES);
  analogWriteResolution(IN4, RES);
  analogWriteFrequency(IN1, FREQ);
  analogWriteFrequency(IN2, FREQ);
  analogWriteFrequency(IN3, FREQ);
  analogWriteFrequency(IN4, FREQ);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  analogReadResolution(12); // 0..4095
  setWheelDutySigned(0,0);

  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32-C6 Sensor-only Motion (IN1..IN4 driver)");
  Serial.println("IR: GPIO4(L), GPIO2(C), GPIO3(R)");
  Serial.println("Motor INs: GPIO5(IN1), GPIO6(IN2), GPIO21(IN3), GPIO22(IN4)");
}

void loop() {
  Sense S = readSensors();
  Command next = decide(S);

  // Debug (optional)
  Serial.print("L="); Serial.print(S.vL); Serial.print(S.Lb ? "(B) " : "(w) ");
  Serial.print("C="); Serial.print(S.vC); Serial.print(S.Cb ? "(B) " : "(w) ");
  Serial.print("R="); Serial.print(S.vR); Serial.print(S.Rb ? "(B) " : "(w) ");
  Serial.print(" -> ");
  if      (next == Command::FWD)   Serial.println("FWD");
  else if (next == Command::LEFT)  Serial.println("LEFT");
  else if (next == Command::RIGHT) Serial.println("RIGHT");
  else                              Serial.println("HOLD");

  switch (next) {
    case Command::FWD:   moveForward(DEFAULT_FWD_SPEED); break;
    case Command::LEFT:  turnLeft (DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M); break;
    case Command::RIGHT: turnRight(DEFAULT_FWD_SPEED, MIN_TURN_RADIUS_M); break;
    default:             setWheelDutySigned(0,0); break;
  }

  lastCmd = next;
  delay(CONTROL_PERIOD_MS);
}

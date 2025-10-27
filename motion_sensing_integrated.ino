#include <Arduino.h>

/* ================== CONFIG ================== */
// Motion targets
static constexpr float CDR_MAX_SPEED_MPS = 0.90f;
static constexpr float CDR_MIN_SPEED_MPS = 0.30f;
static constexpr float DEFAULT_FWD_SPEED  = 0.50f;   // tune to meet CDR forward speed
static constexpr float MIN_TURN_RADIUS_M  = 0.50f;   // set 0.0f to pivot in place for sharper correction
static constexpr float TRACK_WIDTH_M      = 0.130f;  // wheel track (m)

// IR sensors (ADC) — LEFT/CENTER/RIGHT
static constexpr int PIN_LEFT   = 4; // ADC
static constexpr int PIN_CENTER = 2; // ADC
static constexpr int PIN_RIGHT  = 3; // ADC

// Thresholds (tune to your surface)
static int THRESH     = 900;   // >THRESH => BLACK
static int DEAD_BAND  = 80;    // hysteresis around THRESH
static int AVG_N      = 8;     // averaging samples

// DRV8833 (two-input mode) — DIR on two pins; PWM only on IN1 pins
static constexpr uint8_t AIN1 = 5;  // Left DIR1 (PWM pin)
static constexpr uint8_t AIN2 = 6;  // Left DIR2 (opposite level)
static constexpr uint8_t BIN1 = 7;  // Right DIR1 (PWM pin)
static constexpr uint8_t BIN2 = 8;  // Right DIR2 (opposite level)
// nSLEEP: tie to 3.3V on hardware (or wire to a free IO and drive HIGH in setup())

/* ============== TYPES & INTERNALS ============== */
enum class Command { IDLE, FWD, LEFT, RIGHT, HALT };

struct Sense {
  bool Lb, Cb, Rb;
  int vL, vC, vR;
};

static Command lastCmd = Command::FWD;
static uint8_t dutyL = 0, dutyR = 0; // 0..255
static constexpr uint8_t  DUTY_SLEW_PER_TICK = 6;   // smoothing
static constexpr uint32_t CONTROL_PERIOD_MS  = 10;  // 100 Hz control

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

/* ============== LOW-LEVEL DRIVE (DRV8833) ============== */
static inline void driveSide(uint8_t in1, uint8_t in2, int signedDuty) {
  signedDuty = constrain(signedDuty, -255, 255);
  bool fwd = (signedDuty >= 0);
  uint8_t mag = (uint8_t)abs(signedDuty);
  // Two-input mode: set direction via levels, PWM on in1
  digitalWrite(in1, fwd ? HIGH : LOW);
  digitalWrite(in2, fwd ? LOW  : HIGH);
  analogWrite(in1, mag);
}

static void setWheelDutySigned(int leftDuty, int rightDuty) {
  uint8_t tgtL = (uint8_t)abs(constrain(leftDuty,  -255, 255));
  uint8_t tgtR = (uint8_t)abs(constrain(rightDuty, -255, 255));
  uint8_t nextL = dutyL, nextR = dutyR;
  slewTo(nextL, tgtL); slewTo(nextR, tgtR);
  dutyL = nextL; dutyR = nextR;
  driveSide(AIN1, AIN2, (leftDuty  >= 0) ? dutyL : -((int)dutyL));
  driveSide(BIN1, BIN2, (rightDuty >= 0) ? dutyR : -((int)dutyR));
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

  if (radius_m == 0.0f) { // pivot
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
  if (S.Cb)              return Command::FWD;    // line centered → go straight
  if (S.Rb && !S.Lb)     return Command::RIGHT;  // line to right → steer right
  if (S.Lb && !S.Rb)     return Command::LEFT;   // line to left  → steer left
  if (S.Lb && S.Rb)      return Command::FWD;    // intersection → prefer straight
  return lastCmd;                                  // lost line → keep last correction
}

/* ============== SETUP / LOOP ============== */
void setup() {
  // PWM setup on the two PWM pins (AIN1, BIN1)
  const uint8_t  RES  = 8;      // 0..255
  const uint32_t FREQ = 20000;  // 20 kHz
  analogWriteResolution(AIN1, RES);
  analogWriteResolution(BIN1, RES);
  analogWriteFrequency(AIN1, FREQ);
  analogWriteFrequency(BIN1, FREQ);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  analogReadResolution(12); // 0..4095

  setWheelDutySigned(0,0);

  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32-C6 Sensor-only Motion (DRV8833 two-input)");
  Serial.println("IR: IO4(L), IO2(C), IO3(R)");
}

void loop() {
  // Sense → decide → act. Keeps turning until center sees line, then forward.
  Sense S = readSensors();
  Command next = decide(S);

  // Debug (comment out if noisy)
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

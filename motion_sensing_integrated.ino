#include <Arduino.h>

/* ================== DRV8833 PIN MAP (ESP32) ================== */
const int IN1 = 5;    // Left A
const int IN2 = 6;    // Left B
const int IN3 = 21;   // Right A
const int IN4 = 22;   // Right B

/* ================== SPEED / TRIM TUNING ================== */
const int SPEED_FWD  = 200;   // forward drive
const int SPEED_TURN = 200;   // in-place spin
float TRIM_LEFT  = 0.72f;     // left was faster → trim it down
float TRIM_RIGHT = 1.00f;
const int MIN_PWM = 60;       // minimum PWM to break static friction

/* ================== SENSORS (ESP32-C6 ADC1) ================== */
// LEFT->GPIO3, CENTER->GPIO2, RIGHT->GPIO4
const int PIN_LEFT   = 3;
const int PIN_CENTER = 2;
const int PIN_RIGHT  = 4;

const int THRESH = 900;
const int AVG_N  = 8;

static inline int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < AVG_N; ++i) s += analogRead(pin);
  return (int)(s / AVG_N);
}

/* ================== MOTOR HELPERS ================== */
static inline int applyTrimClamp(int duty, float trim) {
  int d = (int)roundf(duty * trim);
  if (d < 0) d = 0;
  if (d > 255) d = 255;
  if (d > 0 && d < MIN_PWM) d = MIN_PWM;
  return d;
}

static inline void leftForwardRaw (int duty){ analogWrite(IN1, duty); analogWrite(IN2, 0); }
static inline void leftReverseRaw (int duty){ analogWrite(IN1, 0);    analogWrite(IN2, duty); }
static inline void rightForwardRaw(int duty){ analogWrite(IN3, duty); analogWrite(IN4, 0); }
static inline void rightReverseRaw(int duty){ analogWrite(IN3, 0);    analogWrite(IN4, duty); }

static inline void leftForward (int duty){ leftForwardRaw (applyTrimClamp(duty, TRIM_LEFT));  }
static inline void leftReverse (int duty){ leftReverseRaw (applyTrimClamp(duty, TRIM_LEFT));  }
static inline void rightForward(int duty){ rightForwardRaw(applyTrimClamp(duty, TRIM_RIGHT)); }
static inline void rightReverse(int duty){ rightReverseRaw(applyTrimClamp(duty, TRIM_RIGHT)); }

void motorsCoast() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);
}

void motorsBrake() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
}

/* ================== MOTIONS ================== */
void forward(int duty = SPEED_FWD) { leftForward(duty); rightForward(duty); }

// NOTE: swapped to match your wiring so labels == motion
void turnLeft(int duty = SPEED_TURN) {
  // previously: leftReverse + rightForward
  // now: leftForward + rightReverse → vehicle yaws LEFT on your setup
  leftForward(duty);
  rightReverse(duty);
}

void turnRight(int duty = SPEED_TURN) {
  // previously: leftForward + rightReverse
  // now: leftReverse + rightForward → vehicle yaws RIGHT on your setup
  leftReverse(duty);
  rightForward(duty);
}

void stopMotion(bool hardBrake = true){ if (hardBrake) motorsBrake(); else motorsCoast(); }

/* ================== SETUP / LOOP ================== */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("Line-follow + DRV8833 motion (trim + hard brake)");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  motorsCoast();

  // Optional PWM freq per pin:
  // analogWriteFrequency(IN1, 20000); analogWriteFrequency(IN2, 20000);
  // analogWriteFrequency(IN3, 20000); analogWriteFrequency(IN4, 20000);

  analogReadResolution(12); // 0..4095
  pinMode(PIN_LEFT, INPUT);
  pinMode(PIN_CENTER, INPUT);
  pinMode(PIN_RIGHT, INPUT);

  Serial.println("IR: LEFT=GPIO3, CENTER=GPIO2, RIGHT=GPIO4; tune THRESH as needed.");
}

void loop() {
  int vL = readAvg(PIN_LEFT);
  int vC = readAvg(PIN_CENTER);
  int vR = readAvg(PIN_RIGHT);

  bool L_black = (vL > THRESH);
  bool C_black = (vC > THRESH);
  bool R_black = (vR > THRESH);

  const char* action = "STOP";

  if (R_black && !L_black) {
    turnRight(SPEED_TURN);
    action = "TURN RIGHT";
  } else if (L_black && !R_black) {
    turnLeft(SPEED_TURN);
    action = "TURN LEFT";
  } else if (C_black || (L_black && R_black)) {
    forward(SPEED_FWD);
    action = "FORWARD";
  } else if (!L_black && !C_black && !R_black) {
    forward(SPEED_FWD); // all white → straight
    action = "FORWARD";
  } else {
    stopMotion(true);
    action = "STOP";
  }

  Serial.print("L="); Serial.print(vL); Serial.print(L_black ? " (BLACK)  " : " (white)  ");
  Serial.print("C="); Serial.print(vC); Serial.print(C_black ? " (BLACK)  " : " (white)  ");
  Serial.print("R="); Serial.print(vR); Serial.print(R_black ? " (BLACK)  " : " (white)  ");
  Serial.print(" ==> "); Serial.println(action);

  delay(50);
}

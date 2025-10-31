#include <Arduino.h>

/* ========== PIN DEFINITIONS ========== */
// IR sensors
const int PIN_LEFT   = 4;   // ADC1_CH4
const int PIN_CENTER = 2;   // ADC1_CH2
const int PIN_RIGHT  = 3;   // ADC1_CH3

// Motor driver (DRV8833)
const int IN1 = 5;    // Left motor input A
const int IN2 = 6;    // Left motor input B
const int IN3 = 21;   // Right motor input A
const int IN4 = 22;   // Right motor input B

/* ========== CONSTANTS ========== */
const int THRESH = 900;   // same as your original
const int AVG_N  = 8;
const int BASE_SPEED = 180;   // forward speed
const int TURN_SPEED = 150;   // turning speed

/* ========== SENSOR READ ========== */
int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < AVG_N; ++i) s += analogRead(pin);
  return (int)(s / AVG_N);
}

/* ========== MOTOR CONTROL ========== */
void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void moveForward() {
  analogWrite(IN1, BASE_SPEED); analogWrite(IN2, 0);
  analogWrite(IN3, BASE_SPEED); analogWrite(IN4, 0);
}

void turnLeft() {
  analogWrite(IN1, 0);          analogWrite(IN2, TURN_SPEED);  // left reverse
  analogWrite(IN3, TURN_SPEED); analogWrite(IN4, 0);           // right forward
}

void turnRight() {
  analogWrite(IN1, TURN_SPEED); analogWrite(IN2, 0);           // left forward
  analogWrite(IN3, 0);          analogWrite(IN4, TURN_SPEED);  // right reverse
}

/* ========== SETUP ========== */
void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("ESP32-C6 3-Sensor Line Follower Ready");
  Serial.println("Sensors: L=GPIO4, C=GPIO2, R=GPIO3");
  Serial.println("Motors: IN1=5, IN2=6, IN3=21, IN4=22");
}

/* ========== MAIN LOOP ========== */
void loop() {
  // Read sensors
  int vL = readAvg(PIN_LEFT);
  int vC = readAvg(PIN_CENTER);
  int vR = readAvg(PIN_RIGHT);

  bool L_black = (vL > THRESH);
  bool C_black = (vC > THRESH);
  bool R_black = (vR > THRESH);

  // Print sensor readings
  Serial.print("L="); Serial.print(vL); Serial.print(L_black ? "(B) " : "(w) ");
  Serial.print("C="); Serial.print(vC); Serial.print(C_black ? "(B) " : "(w) ");
  Serial.print("R="); Serial.print(vR); Serial.print(R_black ? "(B) " : "(w) ");
  Serial.print(" → ");

  // Decide motion
  if (R_black && !L_black) {
    turnRight();
    Serial.println("RIGHT");
  } 
  else if (L_black && !R_black) {
    turnLeft();
    Serial.println("LEFT");
  } 
  else if (C_black) {
    moveForward();
    Serial.println("FORWARD");
  } 
  else {
    stopMotors();
    Serial.println("STOP");
  }

  delay(50);
}

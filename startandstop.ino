#define BUTTON_PIN 5      // Input from push button
#define LED_PIN 4
#define MOTOR_AIN1 18
#define MOTOR_AIN2 19
#define MOTOR_BIN1 21     // if you have second motor
#define MOTOR_BIN2 22

bool systemOn = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // shorter debounce usually enough

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_BIN1, OUTPUT);
  pinMode(MOTOR_BIN2, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  stopMotor();

  Serial.begin(9600);
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState != lastButtonState) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && lastButtonState == HIGH) { // toggle on press
      systemOn = !systemOn;
      digitalWrite(LED_PIN, systemOn ? HIGH : LOW);
      if (systemOn) startMotor(); else stopMotor();
      Serial.println(systemOn ? "System ON" : "System OFF");
    }
  }

  lastButtonState = buttonState;
}

void startMotor() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
  digitalWrite(MOTOR_BIN1, HIGH); // second motor
  digitalWrite(MOTOR_BIN2, LOW);
}

void stopMotor() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  digitalWrite(MOTOR_BIN1, LOW);
  digitalWrite(MOTOR_BIN2, LOW);
}


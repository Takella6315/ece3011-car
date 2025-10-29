// --- Start/Stop Mechanism for ESP32-C6 ---
// Works with RF relay or push button + DRV8833

#define BUTTON_PIN 5      // Input from relay COM or push button
#define LED_PIN 4         // LED indicator (onboard or external)
#define MOTOR_AIN1 18     // DRV8833 motor driver input 1
#define MOTOR_AIN2 19     // DRV8833 motor driver input 2

bool systemOn = false;
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

void setup() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  stopMotor();

  Serial.begin(9600);
  Serial.println("System ready. Press button or use remote to start/stop.");
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && lastButtonState == HIGH) {
      systemOn = !systemOn;
      if (systemOn) {
        startMotor();
        digitalWrite(LED_PIN, HIGH);
        Serial.println("System ON");
      } else {
        stopMotor();
        digitalWrite(LED_PIN, LOW);
        Serial.println("System OFF");
      }
    }
  }

  lastButtonState = buttonState;
}

void startMotor() {
  digitalWrite(MOTOR_AIN1, HIGH);
  digitalWrite(MOTOR_AIN2, LOW);
}

void stopMotor() {
  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
}

// 3-sensor IR reflectance read for ESP32-C6
// LEFT  -> GPIO4  (ADC1_CH4)
// CENTER-> GPIO2  (ADC1_CH2)
// RIGHT -> GPIO3  (ADC1_CH3)

const int PIN_LEFT   = 4;
const int PIN_CENTER = 2;
const int PIN_RIGHT  = 3;

const int THRESH = 900;  // >2300 => BLACK, <=2300 => WHITE
const int AVG_N  = 8;     // small smoothing

int readAvg(int pin) {
  long s = 0;
  for (int i = 0; i < AVG_N; ++i) s += analogRead(pin);
  return (int)(s / AVG_N);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12); // 0..4095
  Serial.println("Buzzcar 3-sensor read (GPIO4=LEFT, GPIO2=CENTER, GPIO3=RIGHT)");
}

void loop() {
  int vL = readAvg(PIN_LEFT);
  int vC = readAvg(PIN_CENTER);
  int vR = readAvg(PIN_RIGHT);

  bool L_black = (vL > THRESH);
  bool C_black = (vC > THRESH);
  bool R_black = (vR > THRESH);

  // Decide direction (simple rule)
  const char* steer;
  if (R_black && !L_black)      steer = "Turn RIGHT";
  else if (L_black && !R_black) steer = "Turn LEFT";
  else                          steer = "Go STRAIGHT";

  // Print everything on one line
  Serial.print("L="); Serial.print(vL);
  Serial.print(L_black ? " (BLACK)  " : " (white)  ");
  Serial.print("C="); Serial.print(vC);
  Serial.print(C_black ? " (BLACK)  " : " (white)  ");
  Serial.print("R="); Serial.print(vR);
  Serial.print(R_black ? " (BLACK)  " : " (white)  ");
  Serial.print(" ==> ");
  Serial.println(steer);

  delay(50);
}

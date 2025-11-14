// --- Pines del ESP32 ---
#define ENA 27    // PWM de velocidad
#define IN1 25    // Dirección 1
#define IN2 26    // Dirección 2

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);   // ENA como salida

  Serial.println("Listo: Motor EV3 con L298N (sin encoder)");
}

// pwm: de 0 a 255
void setMotor(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);

  analogWrite(ENA, pwm);  
}

void stopMotor(bool brake = false) {
  if (brake) {
    // Freno activo: ambos en alto
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
  } else {
    // Parada libre
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, 0);  
}

void loop() {
  Serial.println("Girando hacia adelante fuerte (255)...");
  setMotor(400, true);    
  delay(2000);

  Serial.println("Parando...");
  stopMotor(true);
  delay(1000);

  Serial.println("Girando hacia atrás fuerte (255)...");
  setMotor(120, false);  
  delay(2000);

  Serial.println("Parando...");
  stopMotor(true);
  delay(1000);
}

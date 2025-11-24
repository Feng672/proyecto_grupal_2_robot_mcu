#include <Wire.h>

// ---------- Pines del ESP32 ----------
// Motores (L298N)
#define ENA 27    // PWM motor 1
#define IN1 25    // Dirección motor 1
#define IN2 26

#define ENB 14    // PWM motor 2
#define IN3 18    // Dirección motor 2
#define IN4 19

// I2C MPU-6050
#define SDA_PIN 21
#define SCL_PIN 22

// Dirección I2C del MPU-6050
const int MPU_ADDR = 0x68;

// ---------- Calibración / filtro ----------

// Offsets calculados en reposo
long offAccX = 0, offAccY = 0, offAccZ = 0;
long offGyroX = 0, offGyroY = 0, offGyroZ = 0;

// Valores filtrados (float para suavizar)
float fAccX = 0, fAccY = 0, fAccZ = 0;
float fGyroX = 0, fGyroY = 0, fGyroZ = 0;

// Parámetros del filtro
const float ALPHA = 0.2;      // 0..1  (más pequeño = más suave)
const int   GYRO_DEADZONE = 300; // zona muerta giroscopio (~2–3 °/s)

// ---------- Funciones de motores ----------

// pwm: de 0 a 255
void motor1(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);

  analogWrite(ENA, pwm);
}

void motor2(int pwm, bool forward) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);

  analogWrite(ENB, pwm);
}

void stopMotor(bool brake = false) {
  if (brake) {
    // Freno activo: ambos en alto
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
  } else {
    // Parada libre
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ---------- Lectura cruda del MPU-6050 ----------

void leerMPUcrudo(int16_t &accX, int16_t &accY, int16_t &accZ,
                  int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);             // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  // Acelerómetro
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();

  // Temperatura (se descarta)
  Wire.read(); Wire.read();

  // Giroscopio
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// ---------- Calibración en reposo ----------
// Calcula los offsets promedio mientras el sensor está quieto.

void calibrarMPU(int muestras = 500) {
  long sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;

  Serial.println("Calibrando MPU-6050, NO mover el robot...");

  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < muestras; i++) {
    leerMPUcrudo(ax, ay, az, gx, gy, gz);

    sumAccX += ax;
    sumAccY += ay;
    sumAccZ += az;

    sumGyroX += gx;
    sumGyroY += gy;
    sumGyroZ += gz;

    delay(5);
  }

  offAccX  = sumAccX / muestras;
  offAccY  = sumAccY / muestras;
  offAccZ  = sumAccZ / muestras;
  offGyroX = sumGyroX / muestras;
  offGyroY = sumGyroY / muestras;
  offGyroZ = sumGyroZ / muestras;

  // Inicializamos los filtrados con los offsets (para evitar salto inicial)
  fAccX = 0;
  fAccY = 0;
  fAccZ = 0;
  fGyroX = 0;
  fGyroY = 0;
  fGyroZ = 0;

  Serial.println("Calibracion terminada.");
  Serial.print("Offsets ACC: ");
  Serial.print(offAccX); Serial.print(" ");
  Serial.print(offAccY); Serial.print(" ");
  Serial.println(offAccZ);

  Serial.print("Offsets GYRO: ");
  Serial.print(offGyroX); Serial.print(" ");
  Serial.print(offGyroY); Serial.print(" ");
  Serial.println(offGyroZ);
}

// ---------- Lectura con offset + filtro + zona muerta ----------

void leerMPUFiltrado() {
  int16_t ax, ay, az, gx, gy, gz;

  leerMPUcrudo(ax, ay, az, gx, gy, gz);

  // Aplicar offset (dejamos el sistema centrado en ~0 en la posicion de calibracion)
  float accX = ax - offAccX;
  float accY = ay - offAccY;
  float accZ = az - offAccZ;

  float gyroX = gx - offGyroX;
  float gyroY = gy - offGyroY;
  float gyroZ = gz - offGyroZ;

  // Filtro exponencial simple
  fAccX  = ALPHA * accX  + (1.0 - ALPHA) * fAccX;
  fAccY  = ALPHA * accY  + (1.0 - ALPHA) * fAccY;
  fAccZ  = ALPHA * accZ  + (1.0 - ALPHA) * fAccZ;

  fGyroX = ALPHA * gyroX + (1.0 - ALPHA) * fGyroX;
  fGyroY = ALPHA * gyroY + (1.0 - ALPHA) * fGyroY;
  fGyroZ = ALPHA * gyroZ + (1.0 - ALPHA) * fGyroZ;

  // Zona muerta para giroscopio (pequeñas variaciones → 0)
  if (fabs(fGyroX) < GYRO_DEADZONE) fGyroX = 0;
  if (fabs(fGyroY) < GYRO_DEADZONE) fGyroY = 0;
  if (fabs(fGyroZ) < GYRO_DEADZONE) fGyroZ = 0;

  // Mostrar valores filtrados
  Serial.print("ACCf X="); Serial.print((int)fAccX);
  Serial.print(" Y=");     Serial.print((int)fAccY);
  Serial.print(" Z=");     Serial.print((int)fAccZ);

  Serial.print("  |  GYROf X="); Serial.print((int)fGyroX);
  Serial.print(" Y=");          Serial.print((int)fGyroY);
  Serial.print(" Z=");          Serial.println((int)fGyroZ);
}

// ---------- setup y loop ----------

void setup() {
  Serial.begin(115200);
  delay(300);

  // Pines de motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  stopMotor(false);

  // Iniciar I2C para MPU-6050
  Wire.begin(SDA_PIN, SCL_PIN);

  // Despertar el MPU-6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0);      // 0 = salir de sleep
  Wire.endTransmission(true);

  // Calibración inicial en reposo
  calibrarMPU();   // NO mover el robot durante esto

  Serial.println("Listo: Motores + MPU-6050 con calibracion y filtro.");
}

void loop() {
  Serial.println("Adelante (motores) + lecturas filtradas...");
  // 2 segundos adelante, leyendo MPU cada 200 ms
  for (int i = 0; i < 10; i++) {
    motor1(150, true);
    motor2(150, true);
    leerMPUFiltrado();
    delay(200);
  }

  Serial.println("Parando...");
  stopMotor(true);
  delay(1000);

  Serial.println("Atras (motores) + lecturas filtradas...");
  // 2 segundos atrás, leyendo MPU cada 200 ms
  for (int i = 0; i < 10; i++) {
    motor1(150, false);
    motor2(150, false);
    leerMPUFiltrado();
    delay(200);
  }

  Serial.println("Parando...");
  stopMotor(true);
  delay(1500);
}

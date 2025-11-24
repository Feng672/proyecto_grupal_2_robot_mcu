#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <Wire.h>
#include <math.h>

// ==================== CONFIGURACIÓN BLE ====================
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SENSOR_DATA_UUID    "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

// ==================== CONFIGURACIÓN PINES ====================
// ---- Motores - Driver L298N ----
#define ENA 27    // PWM motor 1 (izquierdo)
#define IN1 25    // Dirección motor 1
#define IN2 26

#define ENB 14    // PWM motor 2 (derecho)
#define IN3 18    // Dirección motor 2
#define IN4 19

// ---- Sensor ultrasónico HC-SR04 ----
#define TRIG_PIN 4
#define ECHO_PIN 5

// ---- Sensores de línea IR (opcional) ----
#define IR_LEFT   33
#define IR_CENTER 32
#define IR_RIGHT  35  // solo entrada

// ---- LED indicador ----
#define LED_PIN 2

// ---- Buzzer ----
#define BUZZER_PIN 15

// ---- I2C MPU-6050 ----
#define SDA_PIN 21
#define SCL_PIN 22
const int MPU_ADDR = 0x68;

// ==================== VARIABLES GLOBALES ====================
BLEServer *pServer;
BLECharacteristic *pCommandCharacteristic;
BLECharacteristic *pSensorCharacteristic;
bool deviceConnected = false;
bool robotMoving = false;

// Estados del robot
enum RobotState {
  IDLE,
  MOVING_FORWARD,
  TURNING_LEFT,
  TURNING_RIGHT,
  DELIVERING_MEDICINE,
  RETURNING_TO_BASE
};
RobotState currentState = IDLE;

// ---- Calibración / filtro MPU-6050 ----
long offAccX = 0, offAccY = 0, offAccZ = 0;
long offGyroX = 0, offGyroY = 0, offGyroZ = 0;

float fAccX = 0, fAccY = 0, fAccZ = 0;
float fGyroX = 0, fGyroY = 0, fGyroZ = 0;

float pitchDeg = 0.0;
float rollDeg  = 0.0;

const float ALPHA = 0.2;          // filtro exponencial
const int   GYRO_DEADZONE = 300;  // zona muerta giroscopio

// Datos de sensores (incluye MPU)
struct SensorData {
  int battery;
  bool obstacleDetected;
  long distance;
  String state;
  bool medicineLoaded;
  int accX, accY, accZ;
  int gyroX, gyroY, gyroZ;
  int pitch;   // grados aproximados
  int roll;    // grados aproximados
};

SensorData sensorData = {
  85,
  false,
  0,
  "IDLE",
  true,
  0, 0, 0,
  0, 0, 0,
  0, 0
};

// ==================== PROTOTIPOS ====================
void stopMotor(bool brake = true);
void stopMotors();
void updateSensors();
long getDistance();
bool checkObstacle();
void sendSensorData();
void sendNotification(String message);
void beep(int times);
void processCommand(String command);
void moveForward();
void turnLeft();
void turnRight();
void deliverMedicine();
void returnToBase();
void emergencyStop();

// MPU-6050
void leerMPUcrudo(int16_t &accX, int16_t &accY, int16_t &accZ,
                  int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ);
void calibrarMPU(int muestras = 500);
void leerMPUFiltrado();

// ==================== CALLBACKS BLE ====================
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("📱 Dispositivo conectado via BLE");
      digitalWrite(LED_PIN, HIGH);
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("📱 Dispositivo desconectado");
      digitalWrite(LED_PIN, LOW);
      stopMotors();
      pServer->startAdvertising();
      Serial.println("📡 BLE Advertising reiniciado");
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String command = pCharacteristic->getValue().c_str();
      command.trim();

      Serial.print("📨 Comando recibido: ");
      Serial.println(command);

      processCommand(command);
    }
};

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n🤖 Robot de Medicinas Iniciando...");

  // ---- Pines de motores ----
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  stopMotors();

  // ---- Sensor ultrasónico ----
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // ---- LEDs y buzzer ----
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // ---- Sensores IR ----
  pinMode(IR_LEFT,   INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT,  INPUT);

  // ---- I2C y MPU-6050 ----
  Wire.begin(SDA_PIN, SCL_PIN);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);   // PWR_MGMT_1
  Wire.write(0);      // salir de sleep
  Wire.endTransmission(true);

  Serial.println("🧭 Calibrando MPU-6050, NO mover el robot...");
  calibrarMPU();
  Serial.println("✅ Calibracion MPU-6050 lista.");

  // ---- BLE ----
  BLEDevice::init("RobotMedicinas-01");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_DATA_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  // Permitir que el cliente active notificaciones
  pSensorCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("✅ BLE inicializado - Listo para conexiones");
  beep(1);
  Serial.println("✅ Robot listo - Esperando conexión BLE...");
  Serial.println("📡 Buscar: 'RobotMedicinas-01'");
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  // Actualizar sensores básicos
  updateSensors();
  // Actualizar y filtrar MPU-6050
  leerMPUFiltrado();

  // Enviar datos de sensores periódicamente si hay conexión
  if (deviceConnected) {
    static unsigned long lastSensorUpdate = 0;
    if (millis() - lastSensorUpdate > 1000) {
      sendSensorData();
      lastSensorUpdate = millis();
    }
  }
  
  // Verificar obstáculos durante movimiento
  if (robotMoving && checkObstacle()) {
    emergencyStop();
  }
  
  delay(50);
}

// ==================== MOTORES ====================
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

void stopMotor(bool brake) {
  if (brake) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  robotMoving = false;
  currentState = IDLE;
  sensorData.state = "IDLE";
}

void stopMotors() {
  stopMotor(true);
}

// ==================== MOVIMIENTO ====================
void moveForward() {
  
  if (checkObstacle()) {
    sendNotification("OBSTACLE_DETECTED");
    return;
  }
  
  Serial.println("🔄 Moviendo hacia adelante");
  motor1(200, true);
  motor2(200, true);
  robotMoving = true;
  currentState = MOVING_FORWARD;
  sensorData.state = "MOVING";
  sendNotification("MOVING_FORWARD");
  beep(1);
}

void turnLeft() {
  Serial.println("↩️ Girando a la izquierda");
  motor1(150, false);
  motor2(150, true);
  robotMoving = true;
  currentState = TURNING_LEFT;
  sensorData.state = "TURNING_LEFT";
  sendNotification("TURNING_LEFT");
  beep(1);
  delay(800);
  stopMotors();
}

void turnRight() {
  Serial.println("↪️ Girando a la derecha");
  motor1(150, true);
  motor2(150, false);
  robotMoving = true;
  currentState = TURNING_RIGHT;
  sensorData.state = "TURNING_RIGHT";
  sendNotification("TURNING_RIGHT");
  beep(1);
  delay(800);
  stopMotors();
}

void emergencyStop() {
  Serial.println("🚨 PARADA DE EMERGENCIA - Obstáculo detectado");
  stopMotors();
  sendNotification("EMERGENCY_STOP_OBSTACLE");
  beep(3);
}

// ==================== MEDICINA ====================
void deliverMedicine() {
  if (!sensorData.medicineLoaded) {
    sendNotification("NO_MEDICINE_LOADED");
    return;
  }

  Serial.println("💊 Iniciando entrega de medicina");
  currentState = DELIVERING_MEDICINE;
  sensorData.state = "DELIVERING";
  sendNotification("DELIVERING_MEDICINE");
  beep(2);

  // ---- Ruta de entrega ----
  // 1) Adelante 2s
  moveForward();
  delay(2000);
  stopMotors();

  // 2) Izquierda
  turnLeft();

  // 3) Adelante 2s
  moveForward();
  delay(2000);
  stopMotors();

  // 4) Derecha
  turnRight();

  // 5) Adelante 1s
  moveForward();
  delay(1000);
  stopMotors();

  // 6) Derecha
  turnRight();

  // 7) Adelante 3s
  moveForward();
  delay(3000);
  stopMotors();

  delay(2000); // entrega simbólica

  sensorData.medicineLoaded = false;
  currentState = IDLE;
  sensorData.state = "IDLE";
  sendNotification("MEDICINE_DELIVERED");
  beep(2);
}

void returnToBase() {
  Serial.println("🏠 Volviendo a la base");
  currentState = RETURNING_TO_BASE;
  sensorData.state = "RETURNING";
  sendNotification("RETURNING_TO_BASE");
  beep(1);

  // ---- Giro 180° para devolverse ----
  motor1(150, true);   // ambos motores hacia adelante para girar derecha
  motor2(150, false);
  delay(1600); // Tiempo para 180°
  stopMotors();

  // ---- Regresar por la ruta inversa ----

  // Ruta original: 3s adelante
  moveForward();
  delay(3000);
  stopMotors();

  // Derecha (inversa de la ruta = IZQUIERDA)
  turnLeft();

  // Luego 1s adelante
  moveForward();
  delay(1000);
  stopMotors();

  // Derecha original → inversa = IZQUIERDA
  turnLeft();

  // Adelante 2s
  moveForward();
  delay(2000);
  stopMotors();

  // Izquierda original → inversa = DERECHA
  turnRight();

  // Adelante 2s
  moveForward();
  delay(2000);
  stopMotors();

  // Llegó a base:
  sensorData.medicineLoaded = true;
  currentState = IDLE;
  sensorData.state = "IDLE";
  sendNotification("AT_BASE");
  beep(2);
}


// ==================== SENSORES ====================
void updateSensors() {
  sensorData.distance = getDistance();
  sensorData.obstacleDetected = (sensorData.distance < 20);
  static unsigned long lastBatteryUpdate = 0;
  if (millis() - lastBatteryUpdate > 10000) {
    sensorData.battery = max(5, sensorData.battery - 1);
    lastBatteryUpdate = millis();
  }
}

long getDistance() {
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.034) / 2;
  return distance;
}

bool checkObstacle() {
  return sensorData.obstacleDetected;
}

// ==================== MPU-6050 ====================
void leerMPUcrudo(int16_t &accX, int16_t &accY, int16_t &accZ,
                  int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // temperatura
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

void calibrarMPU(int muestras) {
  long sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  long sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
  int16_t ax, ay, az, gx, gy, gz;

  for (int i = 0; i < muestras; i++) {
    leerMPUcrudo(ax, ay, az, gx, gy, gz);
    sumAccX += ax; sumAccY += ay; sumAccZ += az;
    sumGyroX += gx; sumGyroY += gy; sumGyroZ += gz;
    delay(5);
  }

  offAccX = sumAccX / muestras;
  offAccY = sumAccY / muestras;
  offAccZ = sumAccZ / muestras;
  offGyroX = sumGyroX / muestras;
  offGyroY = sumGyroY / muestras;
  offGyroZ = sumGyroZ / muestras;

  fAccX = fAccY = fAccZ = 0;
  fGyroX = fGyroY = fGyroZ = 0;
  pitchDeg = 0.0;
  rollDeg  = 0.0;
}

void leerMPUFiltrado() {
  int16_t ax, ay, az, gx, gy, gz;
  leerMPUcrudo(ax, ay, az, gx, gy, gz);

  float accX = ax - offAccX;
  float accY = ay - offAccY;
  float accZ = az - offAccZ;
  float gyroX = gx - offGyroX;
  float gyroY = gy - offGyroY;
  float gyroZ = gz - offGyroZ;

  fAccX  = ALPHA * accX  + (1.0 - ALPHA) * fAccX;
  fAccY  = ALPHA * accY  + (1.0 - ALPHA) * fAccY;
  fAccZ  = ALPHA * accZ  + (1.0 - ALPHA) * fAccZ;
  fGyroX = ALPHA * gyroX + (1.0 - ALPHA) * fGyroX;
  fGyroY = ALPHA * gyroY + (1.0 - ALPHA) * fGyroY;
  fGyroZ = ALPHA * gyroZ + (1.0 - ALPHA) * fGyroZ;

  if (fabs(fGyroX) < GYRO_DEADZONE) fGyroX = 0;
  if (fabs(fGyroY) < GYRO_DEADZONE) fGyroY = 0;
  if (fabs(fGyroZ) < GYRO_DEADZONE) fGyroZ = 0;

  // Calcular ángulos aproximados de pitch y roll en grados
  // Usamos solo acelerómetro para algo sencillo
  float accNormX = fAccX;
  float accNormY = fAccY;
  float accNormZ = fAccZ;

  rollDeg  = atan2(accNormY, accNormZ) * 180.0 / PI;
  pitchDeg = atan2(-accNormX, sqrt(accNormY * accNormY + accNormZ * accNormZ)) * 180.0 / PI;

  sensorData.accX  = (int)fAccX;
  sensorData.accY  = (int)fAccY;
  sensorData.accZ  = (int)fAccZ;
  sensorData.gyroX = (int)fGyroX;
  sensorData.gyroY = (int)fGyroY;
  sensorData.gyroZ = (int)fGyroZ;
  sensorData.pitch = (int)pitchDeg;
  sensorData.roll  = (int)rollDeg;
}

// ==================== BLE ====================
void processCommand(String command) {
  if (command == "CONNECT") {
    sendSensorData();
  } else if (command == "MOVE_FORWARD") {
    moveForward();
  } else if (command == "TURN_LEFT") {
    turnLeft();
  } else if (command == "TURN_RIGHT") {
    turnRight();
  } else if (command == "STOP") {
    stopMotors();
  } else if (command == "DELIVER_MEDICINE") {
    deliverMedicine();
  } else if (command == "RETURN_TO_BASE") {
    returnToBase();
  } else if (command == "GET_SENSOR_DATA") {
    sendSensorData();
  } else {
    Serial.println("❌ Comando desconocido: " + command);
  }
}

void sendSensorData() {
  if (!deviceConnected) return;

  String jsonData = "{";
  jsonData += "\"battery\":" + String(sensorData.battery) + ",";
  jsonData += "\"obstacle\":" + String(sensorData.obstacleDetected ? "true" : "false") + ",";
  jsonData += "\"distance\":" + String(sensorData.distance) + ",";
  jsonData += "\"state\":\"" + sensorData.state + "\",";
  jsonData += "\"medicineLoaded\":" + String(sensorData.medicineLoaded ? "true" : "false") + ",";
  jsonData += "\"accX\":" + String(sensorData.accX) + ",";
  jsonData += "\"accY\":" + String(sensorData.accY) + ",";
  jsonData += "\"accZ\":" + String(sensorData.accZ) + ",";
  jsonData += "\"gyroX\":" + String(sensorData.gyroX) + ",";
  jsonData += "\"gyroY\":" + String(sensorData.gyroY) + ",";
  jsonData += "\"gyroZ\":" + String(sensorData.gyroZ) + ",";
  jsonData += "\"pitch\":" + String(sensorData.pitch) + ",";
  jsonData += "\"roll\":" + String(sensorData.roll);
  jsonData += "}";

  pSensorCharacteristic->setValue(jsonData.c_str());
  pSensorCharacteristic->notify();

  Serial.println("📊 Enviando datos: " + jsonData);
}

void sendNotification(String message) {
  if (!deviceConnected) return;
  pSensorCharacteristic->setValue(message.c_str());
  pSensorCharacteristic->notify();
  Serial.println("📢 Notificación: " + message);
}

// ==================== UTILIDADES ====================
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(100);
  }
}

void rechargeBattery() {
  sensorData.battery = 100;
  Serial.println("🔋 Batería recargada al 100%");
}

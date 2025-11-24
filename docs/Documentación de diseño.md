# Documentación de Diseño

## Aplicación de Usuario y Comunicación

### 1. Arquitectura de la Aplicación

#### 1.1. Estructura General
La aplicación web desarrollada para controlar el Robot Asistente Doméstico sigue una arquitectura cliente-servidor basada en tecnologías web estándar:

```
Aplicación Web (Cliente)
├── Frontend (HTML/CSS/JavaScript)
│   ├── Interfaz de Usuario
│   ├── Controlador Principal
│   └── Módulo de Comunicación BLE
└── Comunicación
    └── Protocolo Bluetooth Low Energy (BLE)
        └── Robot ESP32 (Servidor)
```

#### 1.2. Tecnologías Utilizadas
- **HTML5**: Estructura semántica de la aplicación  
- **CSS3**: Estilos responsivos y diseño moderno  
- **JavaScript ES6+**: Lógica de aplicación y comunicación  
- **Web Bluetooth API**: Comunicación inalámbrica con el robot  
- **Arquitectura MVC**: Separación clara de responsabilidades  

---

### 2. Interfaces de Usuario

#### 2.1. Diseño de la Interfaz Principal

##### Panel de Estado del Robot
- **Indicador de Conexión**: Estado de conexión BLE en tiempo real  
- **Datos de Sensores**: Batería, estado, ubicación, obstáculos  
- **Información de Medicina**: Estado del compartimento de medicinas  
- **Datos de Orientación MPU-6050**: Acelerómetro, giroscopio, pitch, roll  

##### Panel de Control
- **Controles de Movimiento**: Adelante, izquierda, derecha, detener  
- **Controles de Medicina**: Entrega, retorno a base  
- **Botones de Conexión**: Conectar / desconectar  

#### 2.2. Características de Usabilidad
- **Diseño Responsive**  
- **Feedback Visual Inmediato**  
- **Mensajes de Estado**  
- **Indicadores de Seguridad**  

---

### 3. Protocolos de Comunicación Robot-Aplicación

#### 3.1. Bluetooth Low Energy (BLE)

##### Configuración del Servicio BLE
```javascript
SERVICE_UUID: '4fafc201-1fb5-459e-8fcc-c5c9c331914b'
COMMAND_UUID: 'beb5483e-36e1-4688-b7f5-ea07361b26a8'
SENSOR_DATA_UUID: '1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e'
```

##### Características Implementadas
1. **Característica de Comandos (Write)**  
   - Envío de instrucciones al robot  
   - Comandos en texto plano  

2. **Característica de Datos de Sensores (Read/Notify)**  
   - Transmisión periódica del robot  
   - Formato JSON  
   - Notificaciones en tiempo real  

#### 3.2. Formato de Comandos
```javascript
"MOVE_FORWARD"
"TURN_LEFT"
"TURN_RIGHT"
"STOP"
"DELIVER_MEDICINE"
"RETURN_TO_BASE"
"GET_SENSOR_DATA"
```

#### 3.3. Formato de Datos de Sensores
```json
{
  "battery": 85,
  "obstacle": false,
  "distance": 45,
  "state": "MOVING",
  "medicineLoaded": true,
  "accX": 125,
  "accY": -45,
  "accZ": 980,
  "gyroX": 2,
  "gyroY": -1,
  "gyroZ": 0,
  "pitch": 5,
  "roll": -2
}
```

---

### 4. Funcionalidades Implementadas para Control y Monitoreo

#### 4.1. Control en Tiempo Real

##### Gestión de Conexión
- Detección automática del robot  
- Reconexión inteligente  
- Verificación de compatibilidad con Web Bluetooth  

##### Envío de Comandos
- Latencia <200 ms  
- Confirmación visual  
- Manejo de errores  

#### 4.2. Monitoreo en Tiempo Real

##### Actualización de Sensores
- Movimiento, ubicación, obstáculos  
- Batería  
- MPU-6050 (orientación y aceleraciones)  
- Estado de medicina  

##### Sistema de Notificaciones
```javascript
"MOVING_FORWARD"
"OBSTACLE_DETECTED"
"EMERGENCY_STOP_OBSTACLE"
"DELIVERING_MEDICINE"
"MEDICINE_DELIVERED"
"AT_BASE"
```

#### 4.3. Características de Seguridad
- Validación de conexión  
- Parada automática ante obstáculos  
- Alertas de batería baja  
- Manejo de errores robusto  
- Indicadores visuales y auditivos  

---

### 5. Análisis de Comunicación

#### 5.1. Performance y Eficiencia

##### Ancho de Banda
- Comandos: ~20–50 bytes  
- Datos de sensores: ~200–300 bytes  
- Notificaciones: ~10–50 bytes  

##### Latencia
- Conexión inicial: 2–5 s  
- Comandos: <200 ms  
- Sensores: cada 1 s (configurable)  

#### 5.2. Confiabilidad
- Reintentos automáticos  
- Confirmación de estado  
- Sincronización consistente  

#### 5.3. Consumo Energético
- Comunicación BLE optimizada  
- Actualizaciones selectivas  
- Modo standby  

---

### 6. Consideraciones de Implementación

#### 6.1. Compatibilidad Multiplataforma
- Navegadores: Chrome, Edge (Web Bluetooth)  
- Dispositivos: Desktop, tablets, smartphones  
- Independiente del sistema operativo  

#### 6.2. Escalabilidad
- Arquitectura modular  
- Protocolo extensible  
- Interfaz adaptable  

#### 6.3. Mantenibilidad
- Código documentado  
- Manejo centralizado de errores  
- Logging comprensivo  


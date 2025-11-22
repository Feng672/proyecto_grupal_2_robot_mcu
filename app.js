// app.js - Controlador principal para el robot ESP32
class RobotController {
    constructor() {
        this.connected = false;
        this.sensorData = null;
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.updateUI();
        this.checkBluetoothSupport();
        console.log('🎮 Controlador de robot inicializado (Modo ESP32 Real)');
    }

    // Verificar soporte de Bluetooth
    checkBluetoothSupport() {
        if (!window.conexionESP32.isBluetoothSupported()) {
            this.showMessage('❌ Bluetooth no está soportado en este navegador. Usa Chrome o Edge.', 'error');
            document.getElementById('connectBtn').disabled = true;
        }
    }

    setupEventListeners() {
        // Botón de conexión principal
        document.getElementById('connectBtn').addEventListener('click', () => {
            this.connect();
        });

        // Botón de desconexión
        document.getElementById('disconnectBtn').addEventListener('click', () => {
            this.disconnect();
        });

        // Botones de comandos
        document.querySelectorAll('[data-command]').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const command = e.target.getAttribute('data-command');
                this.sendCommand(command);
            });
        });

        // Configurar callbacks para datos del ESP32
        window.conexionESP32.onSensorData((data) => {
            this.handleRealSensorData(data);
        });

        window.conexionESP32.onNotification((notification) => {
            this.handleRealNotification(notification);
        });
    }

    // Conectar al ESP32 
    async connect() {
        try {
            this.showMessage('🔍 Buscando robot ESP32...', 'info');
            
            const result = await window.conexionESP32.connect();

            if (result.success) {
                this.connected = true;
                this.showMessage('✅ ' + result.message, 'success');
                this.updateUI();
                this.startRealSensorUpdates();
            }
        } catch (error) {
            this.showMessage('❌ ' + error.message, 'error');
            console.error('Error de conexión:', error);
        }
    }

    // Enviar comando al ESP32
    async sendCommand(command) {
        if (!this.connected) {
            this.showMessage('⚠️ Primero conecta al robot ESP32', 'warning');
            return;
        }

        try {
            this.showMessage(`🔄 Enviando comando: ${command}...`, 'info');
            
            const result = await window.conexionESP32.sendCommand(command);

            if (result && result.success) {
                this.showMessage('✅ Comando enviado al ESP32', 'success');
            }
            
        } catch (error) {
            this.showMessage('❌ Error: ' + error.message, 'error');
            console.error('Error enviando comando:', error);
            
            // Si hay error de conexión, actualizar estado
            if (error.message.includes('conectado') || error.message.includes('conexión')) {
                this.connected = false;
                this.updateUI();
            }
        }
    }

    // Manejar datos de sensores del ESP32
    handleRealSensorData(data) {
        this.sensorData = data;
        this.updateSensorDisplay();
        
        // Mostrar notificación de datos recibidos (solo ocasionalmente para no spammear)
        if (Math.random() < 0.1) { // 10% de probabilidad
            this.showMessage('📊 Datos de sensores actualizados', 'info');
        }
    }

    // Manejar notificaciones del ESP32
    handleRealNotification(notification) {
        console.log('📢 Notificación del ESP32:', notification);
        
        switch(notification) {
            case 'MOVING_FORWARD':
                this.showMessage('🔄 Robot moviéndose hacia adelante', 'info');
                break;
            case 'TURNING_LEFT':
                this.showMessage('↩️ Robot girando a la izquierda', 'info');
                break;
            case 'TURNING_RIGHT':
                this.showMessage('↪️ Robot girando a la derecha', 'info');
                break;
            case 'STOPPED':
                this.showMessage('⏹️ Robot detenido', 'info');
                break;
            case 'OBSTACLE_DETECTED':
                this.showMessage('🚧 ¡Obstáculo detectado! Robot detenido', 'warning');
                break;
            case 'EMERGENCY_STOP_OBSTACLE':
                this.showMessage('🚨 ¡PARADA DE EMERGENCIA! Obstáculo detectado', 'error');
                break;
            case 'DELIVERING_MEDICINE':
                this.showMessage('💊 Entregando medicina...', 'info');
                break;
            case 'MEDICINE_DELIVERED':
                this.showMessage('✅ Medicina entregada exitosamente', 'success');
                break;
            case 'RETURNING_TO_BASE':
                this.showMessage('🏠 Robot volviendo a la base...', 'info');
                break;
            case 'AT_BASE':
                this.showMessage('🏠 Robot ha llegado a la base', 'success');
                break;
            case 'DISCONNECTED':
                this.connected = false;
                this.showMessage('🔌 Robot ESP32 desconectado', 'warning');
                this.updateUI();
                break;
        }
    }

    // Iniciar actualización de sensores
    startRealSensorUpdates() {
        console.log('📡 Escuchando datos de sensores en tiempo real del ESP32...');
        this.showMessage('📡 Conectado - Esperando datos del ESP32...', 'success');
    }

    // Actualizar interfaz de usuario
    updateUI() {
        const statusElement = document.getElementById('connectionStatus');
        const connectBtn = document.getElementById('connectBtn');
        const disconnectBtn = document.getElementById('disconnectBtn');
        
        if (statusElement) {
            statusElement.textContent = this.connected ? '🟢 Conectado' : '🔴 Desconectado';
            statusElement.className = this.connected ? 'status connected' : 'status disconnected';
        }
        
        // Mostrar/ocultar botones según conexión
        if (connectBtn) {
            connectBtn.style.display = this.connected ? 'none' : 'block';
        }
        if (disconnectBtn) {
            disconnectBtn.style.display = this.connected ? 'block' : 'none';
        }

        // Habilitar/deshabilitar botones de control según conexión
        document.querySelectorAll('[data-command]').forEach(btn => {
            btn.disabled = !this.connected;
        });

        // Actualizar datos de sensores si están disponibles
        if (this.sensorData) {
            this.updateSensorDisplay();
        }
    }

    // Actualizar display de sensores
// Actualizar display de sensores
    updateSensorDisplay() {
        if (!this.sensorData) return;

        const sd = this.sensorData;

        // Mapeo de elementos a actualizar
        const elements = {
            'batteryLevel': (sd.battery !== undefined ? `${sd.battery}%` : '--%'),
            'robotState': sd.state || 'Desconectado',
            'location': this.getLocationText(),
            'obstacles': sd.obstacle ? '⚠️ Detectados' : '✅ Libres',
            // Usa el valor real de medicineLoaded
            'medicineStatus': sd.medicineLoaded ? '✅ Cargada' : 'Vacía',
            'distance': (sd.distance !== undefined ? `${sd.distance} cm` : '-- cm'),

            // --- Nuevo: datos del MPU-6050 ---
            'accXValue': (sd.accX !== undefined ? sd.accX : '--'),
            'accYValue': (sd.accY !== undefined ? sd.accY : '--'),
            'accZValue': (sd.accZ !== undefined ? sd.accZ : '--'),
            'gyroXValue': (sd.gyroX !== undefined ? sd.gyroX : '--'),
            'gyroYValue': (sd.gyroY !== undefined ? sd.gyroY : '--'),
            'gyroZValue': (sd.gyroZ !== undefined ? sd.gyroZ : '--'),
            'pitchValue': (sd.pitch !== undefined ? `${sd.pitch}°` : '--°'),
            'rollValue': (sd.roll !== undefined ? `${sd.roll}°` : '--°')
        };

        // Actualizar cada elemento en la UI
        for (const [id, value] of Object.entries(elements)) {
            const element = document.getElementById(id);
            if (element) {
                element.textContent = value;
                this.applySensorStyles(id, value, sd);
            }
        }
    }


    // Obtener texto de ubicación según el estado
    getLocationText() {
        if (!this.sensorData.state) return 'Base';
        
        switch(this.sensorData.state) {
            case 'MOVING': return 'En movimiento';
            case 'DELIVERING': return 'Entregando medicina';
            case 'RETURNING': return 'Volviendo a base';
            case 'IDLE': return 'En base';
            default: return this.sensorData.state;
        }
    }

    // Aplicar estilos condicionales a los sensores
    applySensorStyles(sensorId, value, sensorData) {
        const element = document.getElementById(sensorId);
        if (!element) return;

        // Resetear estilos
        element.style.color = '';
        element.style.fontWeight = '';

        switch(sensorId) {
            case 'batteryLevel':
                if (sensorData.battery < 20) {
                    element.style.color = '#ff4444';
                    element.style.fontWeight = 'bold';
                } else if (sensorData.battery < 50) {
                    element.style.color = '#ff9800';
                }
                break;
                
            case 'obstacles':
                if (sensorData.obstacle) {
                    element.style.color = '#ff4444';
                    element.style.fontWeight = 'bold';
                }
                break;
                
            case 'robotState':
                if (sensorData.state === 'MOVING' || sensorData.state === 'DELIVERING') {
                    element.style.color = '#007bff';
                    element.style.fontWeight = 'bold';
                }
                break;
        }
    }

    // Mostrar mensajes al usuario
    showMessage(message, type = 'info') {
        console.log(`💬 [${type}] ${message}`);
        
        const messageElement = document.getElementById('message');
        if (messageElement) {
            messageElement.textContent = message;
            messageElement.className = 'message-area show';
            
            // Aplicar estilo según el tipo
            switch(type) {
                case 'success':
                    messageElement.style.background = '#d4edda';
                    messageElement.style.borderColor = '#c3e6cb';
                    messageElement.style.color = '#155724';
                    break;
                case 'error':
                    messageElement.style.background = '#f8d7da';
                    messageElement.style.borderColor = '#f5c6cb';
                    messageElement.style.color = '#721c24';
                    break;
                case 'warning':
                    messageElement.style.background = '#fff3cd';
                    messageElement.style.borderColor = '#ffeaa7';
                    messageElement.style.color = '#856404';
                    break;
                default:
                    messageElement.style.background = '#e7f3ff';
                    messageElement.style.borderColor = '#b3d9ff';
                    messageElement.style.color = '#0066cc';
            }
            
            // Ocultar mensaje después de 4 segundos
            setTimeout(() => {
                messageElement.classList.remove('show');
            }, 4000);
        }
    }

    // Desconectar del ESP32
    async disconnect() {
        try {
            await window.conexionESP32.disconnect();
            this.connected = false;
            this.sensorData = null;
            this.updateUI();
            this.showMessage('🔌 Desconectado del robot ESP32', 'info');
        } catch (error) {
            this.showMessage('❌ Error al desconectar: ' + error.message, 'error');
        }
    }
}

// Inicializar la aplicación cuando se carga la página
document.addEventListener('DOMContentLoaded', () => {
    window.robotController = new RobotController();
    console.log('🚀 Aplicación de control de robot ESP32 iniciada');
});
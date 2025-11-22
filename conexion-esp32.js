// conexion-esp32.js - Conexión Bluetooth real con ESP32
class ConexionESP32 {
    constructor() {
        this.device = null;
        this.server = null;
        this.service = null;
        this.commandCharacteristic = null;
        this.sensorCharacteristic = null;
        this.connected = false;
        this.onSensorDataCallback = null;
        this.onNotificationCallback = null;
        
        // UUIDs (deben coincidir con el firmware del ESP32)
        this.SERVICE_UUID = '4fafc201-1fb5-459e-8fcc-c5c9c331914b';
        this.COMMAND_UUID = 'beb5483e-36e1-4688-b7f5-ea07361b26a8';
        this.SENSOR_DATA_UUID = '1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e';
    }

    // Verificar si el navegador soporta Bluetooth
    isBluetoothSupported() {
        return navigator.bluetooth && navigator.bluetooth.requestDevice;
    }

    // Conectar al ESP32
    async connect() {
        if (!this.isBluetoothSupported()) {
            throw new Error('Bluetooth no está soportado en este navegador. Usa Chrome o Edge.');
        }

        try {
            console.log('🔍 Buscando dispositivo BLE...');
            
            // Solicitar dispositivo Bluetooth
            this.device = await navigator.bluetooth.requestDevice({
                filters: [
                    { name: 'RobotMedicinas-01' },
                    { namePrefix: 'RobotMedicinas' }
                ],
                optionalServices: [this.SERVICE_UUID]
            });

            console.log('📱 Dispositivo seleccionado:', this.device.name);
            
            // Conectar al GATT server
            this.server = await this.device.gatt.connect();
            
            // Obtener el servicio
            this.service = await this.server.getPrimaryService(this.SERVICE_UUID);
            
            // Obtener características
            this.commandCharacteristic = await this.service.getCharacteristic(this.COMMAND_UUID);
            this.sensorCharacteristic = await this.service.getCharacteristic(this.SENSOR_DATA_UUID);
            
            // Escuchar notificaciones de datos de sensores
            await this.sensorCharacteristic.startNotifications();
            this.sensorCharacteristic.addEventListener('characteristicvaluechanged', 
                (event) => this.handleSensorData(event));
            
            this.connected = true;
            
            // Escuchar eventos de desconexión
            this.device.addEventListener('gattserverdisconnected', 
                () => this.handleDisconnection());
            
            console.log('✅ Conectado al ESP32 via Bluetooth');
            
            // Enviar comando de conexión inicial
            await this.sendCommand('CONNECT');
            
            return {
                success: true,
                message: 'Conectado al robot exitosamente',
                deviceName: this.device.name
            };
            
        } catch (error) {
            console.error('❌ Error de conexión Bluetooth:', error);
            this.connected = false;
            
            if (error.name === 'NotFoundError') {
                throw new Error('No se encontró el robot. Asegúrate que esté encendido y visible.');
            } else if (error.name === 'NetworkError') {
                throw new Error('No se pudo conectar al robot. Verifica que esté cerca.');
            } else {
                throw new Error('Error de conexión: ' + error.message);
            }
        }
    }

    // Enviar comando al ESP32
    async sendCommand(command) {
        if (!this.connected || !this.commandCharacteristic) {
            throw new Error('No conectado al robot');
        }

        try {
            const encoder = new TextEncoder();
            const data = encoder.encode(command);
            await this.commandCharacteristic.writeValue(data);
            
            console.log(`📤 Comando enviado: ${command}`);
            return { success: true, command: command };
            
        } catch (error) {
            console.error('❌ Error enviando comando:', error);
            throw new Error('Error enviando comando al robot: ' + error.message);
        }
    }

    // Manejar datos recibidos del ESP32
    handleSensorData(event) {
        const value = event.target.value;
        const decoder = new TextDecoder();
        const dataString = decoder.decode(value);
        
        console.log('📊 Datos recibidos del ESP32:', dataString);
        
        try {
            // Intentar parsear como JSON
            const sensorData = JSON.parse(dataString);
            if (this.onSensorDataCallback) {
                this.onSensorDataCallback(sensorData);
            }
        } catch (e) {
            // Si no es JSON, es una notificación simple
            if (this.onNotificationCallback) {
                this.onNotificationCallback(dataString);
            }
        }
    }

    // Manejar desconexión
    handleDisconnection() {
        console.log('📱 Dispositivo desconectado');
        this.connected = false;
        this.device = null;
        this.server = null;
        
        if (this.onNotificationCallback) {
            this.onNotificationCallback('DISCONNECTED');
        }
    }

    // Desconectar manualmente
    async disconnect() {
        if (this.device && this.device.gatt.connected) {
            this.device.gatt.disconnect();
        }
        this.connected = false;
        console.log('🔌 Desconectado del ESP32');
    }

    // Registrar callbacks para datos de sensores
    onSensorData(callback) {
        this.onSensorDataCallback = callback;
    }

    // Registrar callbacks para notificaciones
    onNotification(callback) {
        this.onNotificationCallback = callback;
    }

    // Verificar estado de conexión
    isConnected() {
        return this.connected && this.device && this.device.gatt.connected;
    }
}

// Crear instancia global
window.conexionESP32 = new ConexionESP32();
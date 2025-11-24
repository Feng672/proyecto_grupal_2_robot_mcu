// conexion-esp32.js - Conexión Bluetooth con ESP32
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

            // Buscar por nombre y declarar el servicio como opcional
            this.device = await navigator.bluetooth.requestDevice({
                filters: [
                    { namePrefix: 'RobotMedicinas' }   // o { name: 'RobotMedicinas-01' }
                ],
                optionalServices: [this.SERVICE_UUID]
            });

            console.log('📱 Dispositivo seleccionado:', this.device.name);

            // Conectar al GATT server
            this.server = await this.device.gatt.connect();
            console.log('✅ Conectado al GATT server');

            // Obtener el servicio
            this.service = await this.server.getPrimaryService(this.SERVICE_UUID);
            console.log('✅ Servicio obtenido:', this.SERVICE_UUID);

            // Característica para comandos
            this.commandCharacteristic = await this.service.getCharacteristic(this.COMMAND_UUID);
            console.log('✅ Characteristic COMMAND lista:', this.COMMAND_UUID);

            // Característica para sensores
            this.sensorCharacteristic = await this.service.getCharacteristic(this.SENSOR_DATA_UUID);
            console.log('✅ Characteristic SENSORS lista:', this.SENSOR_DATA_UUID);

            // Activar notificaciones
            
            await this.sensorCharacteristic.startNotifications();
            this.sensorCharacteristic.addEventListener(
                'characteristicvaluechanged',
                (event) => this.handleSensorData(event)
            );
            console.log('✅ Notificaciones activadas');
            
            this.connected = true;

            this.device.addEventListener('gattserverdisconnected',
                () => this.handleDisconnection()
            );

            console.log('🎉 Conectado al ESP32 (SIN enviar comando CONNECT aún)');

            // OJO: ya NO enviamos "CONNECT" aquí
            return {
                success: true,
                message: 'Conectado al robot exitosamente',
                deviceName: this.device.name
            };

        } catch (error) {
            console.error('❌ Error de conexión Bluetooth DETALLADO:', error);
            this.connected = false;
            throw new Error('Error de conexión: ' + error.message);
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
        const dataString = decoder.decode(value.buffer);
        
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
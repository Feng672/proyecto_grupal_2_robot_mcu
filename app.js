class RobotController {
    constructor() {
        this.connected = false;
        this.batteryLevel = 85;
        this.robotLocation = 'base';
        this.obstacleDetected = false;
        
        this.init();
    }

    init() {
        this.setupEventListeners();
        this.updateUI();
        console.log('🤖 Controlador de robot inicializado');
    }

    setupEventListeners() {
        // Botón de conexión
        document.getElementById('connectBtn').addEventListener('click', () => {
            this.connect();
        });

        // Botones de movimiento
        document.querySelectorAll('.btn-move, .btn-medicine').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const command = e.target.getAttribute('data-command');
                this.sendCommand(command);
            });
        });
    }

    async connect() {
        console.log('🔄 Intentando conectar al robot...');
        
        // Simular conexión (luego reemplazar con Bluetooth real)
        await this.simulateConnection();
        
        this.connected = true;
        this.updateUI();
        this.startSensorSimulation();
        
        console.log('✅ Conectado al robot');
    }

    async sendCommand(command) {
        if (!this.connected) {
            this.showMessage('⚠️ Primero conecta al robot');
            return;
        }

        console.log(`📤 Enviando comando: ${command}`);
        
        // Aquí irá el código Bluetooth real
        switch(command) {
            case 'MOVE_FORWARD':
                this.robotLocation = 'en_movimiento';
                break;
            case 'DELIVER_MEDICINE':
                await this.simulateMedicineDelivery();
                break;
            case 'RETURN_TO_BASE':
                this.robotLocation = 'base';
                break;
        }
        
        this.updateUI();
    }

    async simulateConnection() {
        // Simular tiempo de conexión
        return new Promise(resolve => {
            setTimeout(resolve, 2000);
        });
    }

    async simulateMedicineDelivery() {
        this.showMessage('💊 Entregando medicina...');
        await new Promise(resolve => setTimeout(resolve, 3000));
        this.showMessage('✅ Medicina entregada exitosamente');
        this.robotLocation = 'usuario';
    }

    startSensorSimulation() {
        // Simular datos del robot en tiempo real
        setInterval(() => {
            this.batteryLevel = Math.max(10, this.batteryLevel - 0.5);
            this.obstacleDetected = Math.random() > 0.8;
            this.updateUI();
        }, 3000);
    }

    updateUI() {
        // Actualizar estado de conexión
        const statusElement = document.getElementById('status');
        statusElement.textContent = this.connected ? '🟢 Conectado' : '🔴 Desconectado';
        statusElement.className = this.connected ? 'status connected' : 'status disconnected';

        // Actualizar datos de sensores
        document.getElementById('batteryLevel').textContent = `${Math.round(this.batteryLevel)}%`;
        document.getElementById('location').textContent = this.robotLocation;
        document.getElementById('obstacles').textContent = 
            this.obstacleDetected ? '⚠️ Detectados' : '✅ Libres';
    }

    showMessage(message) {
        console.log(`💬 ${message}`);
        // Podrías agregar un sistema de notificaciones aquí
        alert(message); // Temporal - mejorar después
    }
}

// Inicializar la aplicación cuando se carga la página
document.addEventListener('DOMContentLoaded', () => {
    window.robotController = new RobotController();
});
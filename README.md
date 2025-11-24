# Robot MCU

---

## Propósito del proyecto

La implementación de un robot basado en microcontrolador para solventar una aplicacion especifica como actividad de aprendizaje para que asi el grupo de trabajo demuestre un dominio de la teoría vista en clase. Ademas de reforzar habilidades como el trabajo en equipo y el diseño de hardware y software, este proyecto también tiene como objetivo una profundización de temas sociales como la identificación de áreas dentro del contexto nacional o internacional donde la implementación de este robot sea justificable. 

---  

## Instrucciones de compilación y ejecución

### Requisitos Previos

- Python 3.8 o superior
- Mkcert

```bash
mkcert localhost  # para poder utiizar la página desde el teléfono
npx http-server -S -C localhost.pem -K localhost-key.pem -p 5500 #para iniciar la página 
```

---
## Lista de materiales

* Microcontrolador ESP32
* Motores DC con encoders
* Drivers de motores L298N
* Sensores IR para línea (módulo MPU-6050) 
* 2 Sensores ultrasonicos HC-SRO4 
* Display OLED I2C
* Bateria LiPo 7.4V y sistema de carga
* Chasis impresión 3D
* PCB

---


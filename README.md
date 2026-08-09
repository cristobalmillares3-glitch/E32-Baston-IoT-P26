# P26 – Bastón de asistencia para discapacidad visual

**Curso:** Fundamentos de IoT — 2º Semestre 2026, Universidad Autónoma de Chile
**Equipo:** E32

## Descripción del Proyecto
Sistema IoT diseñado para personas con discapacidad visual que detecta obstáculos a la altura del pecho o la cabeza[cite: 1]. El sistema captura información del entorno en tiempo real mediante sensores ultrasónicos (HC-SR04 / VL53L0X) y la procesa en un ESP32 para emitir respuestas instantáneas mediante vibración y sonido antes del contacto. 

Además, incluye un módulo GPS (NEO-6M) que transmite la ubicación en vivo mediante MQTT hacia un panel seguro (Node-RED e InfluxDB), permitiendo a un cuidador monitorear la posición del usuario.

## Integrantes y Roles
* **Cristóbal Millares:** Project Manager & QA
* **Martin Salinas:** Firmware & Lógica Edge
* **Franco Diaz:** Hardware & Electrónica
* **Anderson Pineda:** Datos & Integración Cloud
* **Karen Pérez:** Diseño UX, Ergonomía & Privacidad

## Estructura del Repositorio
* `/docs`: Documentación oficial del proyecto y evaluaciones.
* `/src`: Código fuente del ESP32 y scripts de Python.
* `/hardware`: Esquemas electrónicos y listas de componentes.

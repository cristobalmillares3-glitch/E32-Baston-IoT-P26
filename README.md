# P26 — Bastón de asistencia para discapacidad visual

**Equipo E32** · Fundamentos de IoT · 2º Semestre 2026 · Ingeniería Civil Informática · Universidad Autónoma de Chile
**NRC 22022** · Tutores: Miguel Cochea y Yainet García

> ⚠️ Prototipo educativo. No es un dispositivo de asistencia certificado ni sustituye al bastón blanco.

## Tabla de contenidos

- [Descripción del problema](#descripción-del-problema)
- [Arquitectura por capas](#arquitectura-por-capas)
- [Hardware y conexiones](#hardware-y-conexiones)
- [Máquina de estados (FSM)](#máquina-de-estados-fsm)
- [Calibración del sensor](#calibración-del-sensor)
- [Plan de datos y MQTT](#plan-de-datos-y-mqtt)
- [Alimentación del nodo](#alimentación-del-nodo)
- [Privacidad y consentimiento](#privacidad-y-consentimiento)
- [Alcance e hitos](#alcance-e-hitos)
- [Riesgos](#riesgos)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Cómo simular / ejecutar](#cómo-simular--ejecutar)
- [Organización del equipo](#organización-del-equipo)

## Descripción del problema

Las personas con discapacidad visual se desplazan principalmente con bastón blanco, que solo detecta obstáculos al contacto. Objetos a la altura del pecho o la cabeza (ramas, letreros, puertas abiertas) pueden golpearlas antes de ser detectados.

Según la ENDIDE 2022, en Chile existen aproximadamente 153.560 adultos ciegos y más de 4,6 millones de adultos con algún grado de pérdida de visión.

**P26** es un bastón inteligente que mide la distancia a los obstáculos en tiempo real y avisa mediante vibración y sonido *antes* del contacto. La variante con GPS permite, con consentimiento, que un cuidador monitoree la ubicación del usuario.

## Arquitectura por capas

```
EDGE                INGESTA              HOT PATH                    COLD PATH
──────────────────  ──────────────────   ─────────────────────────   ──────────────────
HC-SR04 / VL53L0X                                                    Export de datos
   │ distancia (cm)                                                       │
   ▼                                                                      ▼
Módulo GPS NEO-6M → ESP32 → Broker MQTT → Panel del cuidador           Dataset crudo
   │ lat/long          │        (Topic)     (ubicación en vivo)             │
   ▼                   │                        │                          ▼
Motor de vibración     │                        ▼                      ETL en Python
Buzzer                 └── Node-RED ──→ Base de datos en vivo              │
                        (suscripción MQTT)   / InfluxDB                    ▼
                                                                    Pregunta analítica
                                                                            │
                                                                            ▼
                                                                        Dashboard
```

- **EDGE:** captura datos en el bastón y activa alertas locales (vibración/sonido) en tiempo real.
- **INGESTA:** transporta los mensajes de forma segura entre el ESP32 y la nube mediante el broker MQTT.
- **HOT PATH:** transmite la ubicación en vivo al panel del cuidador y guarda registros inmediatos en InfluxDB.
- **COLD PATH:** procesa datos históricos mediante ETL en Python para generar análisis y hallazgos.

**¿Qué decide el ESP32 por sí solo?** Mide continuamente la distancia con el sensor ultrasónico y activa el motor de vibración y el buzzer con intensidad proporcional a la cercanía del obstáculo, sin depender de la conexión a internet.

**¿Qué decide la plataforma?** Recibe los datos del ESP32 para visualizarlos en el panel de monitoreo, almacena el historial de detecciones y, en la variante con GPS, muestra la ubicación al cuidador.

## Hardware y conexiones

| Componente | Pin ESP32 | Notas |
|---|---|---|
| HC-SR04 — TRIG | GPIO 26 | Conexión directa |
| HC-SR04 — ECHO | GPIO 25 | **Obligatorio** divisor de tensión 10 kΩ / 20 kΩ (ECHO sale en 5 V, el ESP32 lee 3,3 V) |
| Buzzer pasivo | GPIO 32 | Se maneja con `tone()`/`noTone()`; señal directa al GPIO (no necesita transistor) |
| Motor de vibración | GPIO 33 | **Siempre** vía transistor NPN: base←resistencia←GPIO; colector←motor←riel 5 V; emisor a GND común; diodo flyback en el motor. Nunca directo al GPIO ni al pin de 3,3 V |
| GPS NEO-6M — TX | GPIO 17 (RX del ESP32, UART2) | |
| GPS NEO-6M — RX | GPIO 16 (TX del ESP32, UART2) | |

Librería requerida: [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus) (Mikal Hart), instalable desde el Gestor de Librerías del Arduino IDE.

## Máquina de estados (FSM)

Histéresis de dos umbrales por frontera, para que el ruido del sensor no haga oscilar el estado cerca del límite:

| Estado | Entrada | Salida | Buzzer | Motor |
|---|---|---|---|---|
| `REPOSO` | — | < 2,0 m → `OBSTACULO_LEJOS` | apagado | apagado |
| `OBSTACULO_LEJOS` | < 2,0 m | > 2,5 m → `REPOSO` · < 1,0 m → `OBSTACULO_CERCA` | pulso lento (500 ms, 1500 Hz) | pulso (500 ms) |
| `OBSTACULO_CERCA` | < 1,0 m | > 1,5 m → `OBSTACULO_LEJOS` | pulso rápido (150 ms, 2500 Hz) | continuo |

```
Libre  ──2 m<──▶  Obstáculo lejos  ──1 m< detección──▶  Obstáculo cerca
  ▲     ◀──>2,5 m──                ◀──────>1,5 m────────
  │
  └── Errores de medición (5 mediciones erróneas consecutivas)
```

Cada flecha representa un rango de medición; una racha de 5 mediciones erróneas se trata como caso de error, no como obstáculo real.

## Calibración del sensor

Calibración propia mediante regresión lineal con datos medidos con huincha en 6 distancias reales (10/20/50/60/80/100 cm, n=10 c/u, N=60 muestras):

```
medido = CAL_M · real + CAL_B      (R² = 0,9987)
real   = (medido − CAL_B) / CAL_M
```

- `CAL_M = 0.9774`
- `CAL_B = 0.7886` cm

El timeout de disparo (`TIMEOUT_US = 12000`) está ajustado a ~2,06 m, el alcance útil del bastón, en vez de los 5 m máximos del sensor, para no gastar latencia esperando un eco que no va a volver. Mediciones por debajo de `DIST_MIN_PLAUSIBLE_CM = 2.0` cm se descartan como ruido y se mantiene la última distancia válida.

## Plan de datos y MQTT

**Tópico:** `P07/e32/p26/nodo1`

| Campo | Unidad | Sensor | Frecuencia | Justificación |
|---|---|---|---|---|
| `dist_obj` | cm | HC-SR04 o VL53L0X | 5 s | Mantener un control constante de las distancias medidas |
| `ubicacion` | latitud y longitud | GPS NEO-6M | 20 s | Monitoreo apropiado de la ubicación de la persona |

**Pregunta analítica (≥3 semanas de datos propios):** ¿Cuál es la distribución de las distancias de detección durante el tiempo de uso, y qué proporción corresponde a detecciones críticas? Se espera encontrar patrones de detección/uso del sensor, que pueden verse afectados por desajustes del sensor.

## Alimentación del nodo

Nodo a **batería** (2 celdas 18650 en serie, 7,4 V nominal) — es un dispositivo portátil; un cable anularía su función.

| Parámetro | Valor |
|---|---|
| Período del ciclo T | 0,1 s |
| Riel principal | 3,3 V |
| Rendimiento del conversor η | 0,85 |
| Capacidad nominal batería | 2600 mAh |
| Corriente media del nodo | 9115 mA |
| Corriente de pico máxima | 300 mA (ESP32 DevKit) |
| Corriente exigida a la batería | ≈4782 mA |
| Autonomía estimada | ≈0,43 h |

- Los actuadores (motor de vibración y buzzer) están en un **riel propio**, separado del ESP32, porque su pico de corriente podría causar caída de voltaje y reinicio de la placa si compartieran la fuente.
- No usa deep sleep completo: un obstáculo puede aparecer en cualquier momento mientras la persona camina, y el aviso debe llegar antes del contacto. Es compatible con publicación permanente porque el ESP32 permanece siempre activo.

*Nota: la autonomía estimada (~26 min) refleja el período de muestreo T=0,1 s usado en la ficha de cálculo; es un parámetro a revisar/optimizar antes de la Feria.*

## Privacidad y consentimiento

1. El usuario tiene discapacidad visual y no puede operar una interfaz digital de consentimiento: la activación del GPS **no depende del usuario**, sino de un cuidador o familiar designado, quien la activa/desactiva remotamente desde el panel mediante el tópico `cmd`.
2. El acceso a la ubicación en vivo está restringido: solo el cuidador asignado puede ver el panel, mediante autenticación.
3. El historial de ubicación se conserva **5 horas**, período óptimo para reconstruir la última posición conocida en caso de emergencia y recolectar métricas internas de comportamiento del sistema.

## Alcance e hitos

| Hito | Qué se demuestra |
|---|---|
| **NP1 · Semana 7** | El nodo mide `dist_obj` cada 5 s, el motor de vibración responde a la distancia medida y el GPS entrega los datos requeridos |
| **NP2 · Semana 12** | Buzzer y motor entregan señales según distancia; el GPS obtiene ubicación; `dist_obj` y posición se envían por MQTT a Node-RED para visualización y almacenamiento en InfluxDB |
| **Feria · Semana 17** | Sensor calibrado para la distancia óptima de detección, junto a un chasis ergonómico para componentes y usuario |

**Fuera de alcance:** reinventar la estructura del bastón; aplicación de posición/dirección en tiempo real para terceros.

## Riesgos

| Riesgo | Probabilidad | Mitigación |
|---|---|---|
| Problemas de chasis que desconectan el sensor o generan detecciones incorrectas | Media | Verificaciones periódicas del cableado/funcionamiento |
| El sensor no detecta obstáculos por un periodo prolongado | Alta | Medición más allá de la distancia de alerta para verificar integridad del sensor |
| GPS entrega posición errónea o incoherente con el historial | Media | Validar la posición comparándola con el historial registrado |

## Estructura del repositorio

```
├── firmware/           # Sketches del ESP32 (FSM, sensores, actuadores, GPS)
├── simulacion/         # Proyectos Wokwi (GT1: cadena de adquisición y calibración)
├── docs/               # Fichas técnicas (alimentación del nodo, diagramas, ED)
└── README.md
```

## Cómo simular / ejecutar

1. Instalar Arduino IDE con soporte para placas ESP32.
2. Instalar la librería **TinyGPSPlus** (Mikal Hart) desde el Gestor de Librerías.
3. Cargar el sketch principal (FSM + buzzer + motor + GPS) en la ESP32, respetando el pinout de la sección [Hardware y conexiones](#hardware-y-conexiones).
4. Para pruebas sin hardware físico, usar el proyecto de simulación en Wokwi (carpeta `simulacion/`), que reproduce la cadena de adquisición (lectura ADC → conversión física → calibración de dos puntos → filtrado) con un potenciómetro como entrada simulada.
5. Abrir el Monitor Serie a 115200 baudios para ver la distancia, el estado de la FSM y los datos GPS en vivo.

## Organización del equipo

| Rol | Responsable | Rotación |
|---|---|---|
| Firmware & Lógica Edge | Martin Salinas | En NP2 intercambia con Datos/Cloud |
| Hardware & Electrónica | Franco Diaz | En NP2 intercambia con Diseño UX/Ergonomía |
| Datos & Integración Cloud | Anderson Pineda | En NP2 intercambia con Firmware & Lógica Edge |
| Diseño UX, Ergonomía & Privacidad | Karen Pérez | En NP2 intercambia con Hardware & Electrónica |
| Project Manager & QA | Cristóbal Millares | Rota en NP3 o se mantiene como integrador rotando en tareas secundarias semanales |

**Canales:** WhatsApp para avisos rápidos; GitHub para código; Google Drive/Notion para documentación. No se comparte código por WhatsApp.

**Trabajo síncrono:** martes y jueves de 15:30 a 16:50 en el laboratorio, para integración y pruebas físicas del bastón.

---

**Integrantes:** Cristóbal Millares, Anderson Pineda, Franco Diaz, Karen Pérez y Martin Salinas
**Repositorio:** https://github.com/cristobalmillares3-glitch/E32-Baston-IoT-P26

# P26 – Bastón de asistencia para discapacidad visual

**Curso:** Fundamentos de IoT — 2º Semestre 2026, Universidad Autónoma de Chile  
**Equipo:** E32  

## Descripción del Proyecto
Sistema IoT diseñado para personas con discapacidad visual que detecta obstáculos a la altura del pecho o la cabeza. El sistema captura información del entorno en tiempo real mediante sensores ultrasónicos (HC-SR04 / VL53L0X) y la procesa en un ESP32 para emitir respuestas instantáneas mediante vibración y sonido antes del contacto. 

Además, incluye un módulo GPS (NEO-6M) que transmite la ubicación en vivo mediante MQTT hacia un panel seguro (Node-RED e InfluxDB), permitiendo a un cuidador monitorear la posición del usuario.

## Calibración y Corrección de Sensores
De acuerdo con los requerimientos de la asignatura, se realizó el proceso de medición utilizando valores de referencia para calcular y aplicar la corrección lineal de lecturas en el firmware del ESP32.

### Simulación en Wokwi
https://wokwi.com/projects/472279508231864321
Para preparar el entorno de simulación en Wokwi, se ajustaron los siguientes parámetros en los componentes virtuales:
* **Ganancia (Gain) Wokwi:** 85
* **Offset Wokwi:** 1.30

### Valores de Referencia y Cálculo de Recta
Se utilizaron las siguientes distancias de prueba ($r$) entregadas para la calibración:
* **Punto de referencia 1 ($r_1$):** 9
* **Punto de referencia 2 ($r_2$):** 34

A partir de las mediciones en estos dos puntos, se despejaron la ganancia ($m$) y el offset ($b$) para ajustar la ecuación de la recta y aplicar la corrección directamente en el firmware:
* **Ganancia Calculada ($m$):** 0.951940
* **Offset Calculado ($b$):** -1.895092

### Verificación y Tolerancia
Para validar la corrección matemática implementada en el código, se verificó el sistema en un tercer punto de prueba distinto:
* **Punto de verificación ($r_3$):** 20.5

**Tolerancia del proyecto:** Considerando la naturaleza del proyecto (detección de obstáculos para evitar colisiones peatonales), se declara una tolerancia aceptable de **± 2 cm**. Esta tolerancia se considera adecuada, ya que pequeñas variaciones en este rango no afectan el tiempo de reacción necesario para alertar al usuario mediante la respuesta háptica (vibración) y sonora de forma segura.

### Filtro de Ruido y Criterio de Elección de N
Para mitigar el ruido inherente a la lectura de sensores espaciales y evitar falsos positivos, se implementó un **Filtro de Mediana** con una ventana de **N = 5**. 

**Criterio de elección:** Se descartó la media móvil porque promedia y arrastra los errores (como los "ecos" falsos del ultrasonido). En su lugar, el filtro de mediana descarta instantáneamente los picos de ruido. El tamaño de la ventana ($N=5$) se seleccionó como el equilibrio óptimo para el bastón: un $N$ menor no filtraría eficientemente las anomalías, mientras que un $N$ mayor (por ejemplo, 10 o 15) introduciría un retardo computacional prolongado. En un dispositivo de asistencia para la marcha, el tiempo real es crítico; un retardo excesivo provocaría que el usuario impacte el obstáculo antes de recibir la alerta háptica.

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

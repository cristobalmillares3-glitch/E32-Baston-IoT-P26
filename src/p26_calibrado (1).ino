// ============================================================
// Fundamentos de IoT 2026-2 · P26 · Baston de asistencia para
// discapacidad visual · E32 · LAB 7
// Sensores/actuadores: HC-SR04 (ultrasonico) + buzzer + motor de
// vibracion + GPS NEO-6M
//
// PASO 3 - FSM de deteccion por distancia + buzzer + motor de
// vibracion + lectura basica de GPS.
// Construido sobre el paso anterior: se mantiene el driver del
// sensor tal cual (mismos pines, mismo timeout ajustado a ~2 m de
// alcance util).
//   Item 4: FSM con enum + switch/case, sin delay(), con millis().
//   Item 5/6: actuacion de DOS actuadores.
//     - Buzzer PASIVO: se maneja con tone()/noTone() (necesita la
//       señal de la frecuencia, no basta con HIGH/LOW), directo al
//       GPIO (señal, no carga de potencia: no necesita transistor).
//     - Motor de vibracion: ya conectado. SIEMPRE via transistor NPN
//       (base <- resistencia <- GPIO; colector <- motor <- riel 5V;
//       emisor a GND comun; diodo flyback en el motor). NUNCA el
//       motor directo al GPIO ni al pin de 3,3V.
// GPS NEO-6M: se lee por UART2 (Serial2) y se parsea con la libreria
// TinyGPS++ (instalar "TinyGPSPlus" de Mikal Hart desde el Gestor de
// Librerias del IDE). Cada 5 s se imprime latitud/longitud/altitud/
// satelites ya calculados por el Monitor Serie.
//
// Version simple para probar: sin botones ni LEDs. Conectar sensor +
// buzzer + motor + GPS y deberia funcionar.
//
// Conexion:
//   HC-SR04 VCC -> 5V   GND -> GND
//   TRIG -> GPIO 26 (directo)
//   ECHO -> divisor 10 k / 20 k -> GPIO 25   (obligatorio: ECHO sale
//                                              en 5V, el ESP32 lee 3,3V)
//   Buzzer pasivo -> GPIO 32 (+ resistencia en serie si el modulo la
//                              pide), GND comun.
//   Motor de vibracion -> GPIO 33 -> base del transistor NPN (con su
//                          resistencia) -> colector al motor -> motor
//                          al riel de 5V -> emisor a GND comun; diodo
//                          flyback en paralelo con el motor.
//   GPS NEO-6M -> VCC a 3,3V o 5V (segun el modulo), GND a GND comun
//                 TX (del GPS) -> GPIO 17  (RX del ESP32, UART2)
//                 RX (del GPS) -> GPIO 16  (TX del ESP32, UART2)
//
// Nota: tone()/noTone() requieren el core ESP32 Arduino 2.x o
// superior; si el core es mas antiguo, revisar si esta disponible.
//
// PROTOTIPO EDUCATIVO. No es un dispositivo de asistencia ni
// sustituye al baston blanco.
// ============================================================

#include <TinyGPS++.h>

// ---------- 1. Pines ----------
const int PIN_TRIG   = 26;
const int PIN_ECHO   = 25;
const int PIN_BUZZER = 32;   // buzzer pasivo, se maneja con tone()
const int PIN_VIBRA  = 33;   // motor de vibracion, via transistor NPN
const int GPS_RX_PIN = 17;   // ESP32 RX  <- TX del GPS
const int GPS_TX_PIN = 16;   // ESP32 TX  -> RX del GPS

// ---------- 2. Sensor: mismos parametros del paso 1 ----------
// Alcance util del baston: unos 2 m. El timeout se ajusta a esa
// distancia (no a los 5 m maximos del sensor) para no gastar latencia
// esperando un eco que no va a volver.
const unsigned long TIMEOUT_US    = 12000;   // ~2,06 m de alcance
const float VELOCIDAD_CM_US       = 0.0343;
const unsigned long PERIODO_MS    = 60;      // pausa minima entre disparos
const float DIST_MIN_PLAUSIBLE_CM = 2.0;     // bajo esto se descarta (ruido)
const float SIN_OBSTACULO_CM      = 999.0;   // "sin eco" = nada en rango

// Calibracion propia (item D1): regresion lineal de datos medidos con
// huincha en 6 distancias reales (10/20/50/60/80/100 cm, n=10 c/u,
// N=60 muestras): medido = CAL_M*real + CAL_B, con R2=0,9987. Para
// corregir, se despeja: real = (medido - CAL_B) / CAL_M.
const float CAL_M = 0.9774;
const float CAL_B = 0.7886;   // cm

// ---------- 3. Umbrales de la FSM (histeresis) ----------
// Un umbral de entrada y uno de salida por frontera, para que el
// ruido del sensor no haga oscilar el estado cerca del limite.
const float D_LEJOS_ON_CM  = 200.0;   // < 2,0 m -> entra a OBSTACULO_LEJOS
const float D_LEJOS_OFF_CM = 250.0;   // > 2,5 m -> vuelve a REPOSO
const float D_CERCA_ON_CM  = 100.0;   // < 1,0 m -> entra a OBSTACULO_CERCA
const float D_CERCA_OFF_CM = 150.0;   // > 1,5 m -> vuelve a OBSTACULO_LEJOS

// ---------- 4. Ritmo de aviso: buzzer + motor por estado ----------
const uint32_t T_BUZZER_LEJOS_MS = 500;    // pitido lento: aviso suave
const uint16_t FREQ_BUZZER_LEJOS = 1500;   // tono grave
const uint32_t T_BUZZER_CERCA_MS = 150;    // pitido rapido: aviso urgente
const uint16_t FREQ_BUZZER_CERCA = 2500;   // tono agudo
const uint32_t T_VIBRA_LEJOS_MS  = 500;    // pulso del motor en LEJOS
const uint32_t T_SERIAL_MS       = 200;    // ver la distancia en vivo
const uint32_t T_GPS_MS          = 5000;   // volcar el GPS cada 5 s

// ---------- 5. Estado de la FSM ----------
enum Estado : uint8_t { REPOSO, OBSTACULO_LEJOS, OBSTACULO_CERCA };
Estado estado = REPOSO;
uint32_t t_entrada = 0;   // millis() al entrar al estado actual

// ---------- 6. Variables de medicion ----------
unsigned long t_sensor = 0;
float distancia_cm = SIN_OBSTACULO_CM;

// ---------- 7. GPS: objeto TinyGPS++ (valores ya calculados) ----------
TinyGPSPlus gps;
uint32_t t_gps = 0;

// ---------- 8. Diagnostico por Serial (no bloqueante) ----------
uint32_t t_serial = 0;

// ============================================================
// Unica funcion que escribe 'estado': asi el reloj del estado nuevo
// siempre parte en cero y queda registrada la transicion.
// ============================================================
void cambiar(Estado e) {
  estado = e;
  t_entrada = millis();
  const char* nombre[] = {"REPOSO", "OBSTACULO_LEJOS", "OBSTACULO_CERCA"};
  Serial.printf("[%lu ms] -> %s\n", millis(), nombre[e]);
}

// ============================================================
// Sensor: mismo driver del paso 1, gateado por millis() (sin delay(),
// salvo los delayMicroseconds() del protocolo electrico del HC-SR04).
// ============================================================
void leerDistancia() {
  if (millis() - t_sensor < PERIODO_MS) return;
  t_sensor = millis();

  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long t_us = pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);

  if (t_us == 0) {
    // Sin eco dentro de los ~2 m de alcance util: se interpreta como
    // "nada delante" (no como falla), porque el timeout esta acortado
    // a proposito para no gastar latencia (ver paso 1).
    distancia_cm = SIN_OBSTACULO_CM;
  } else {
    float d_bruta = (t_us * VELOCIDAD_CM_US) / 2.0;
    float d = (d_bruta - CAL_B) / CAL_M;   // corrige con la calibracion propia
    if (d >= DIST_MIN_PLAUSIBLE_CM) {
      distancia_cm = d;
    }
    // si d < DIST_MIN_PLAUSIBLE_CM se descarta como ruido y se
    // mantiene la ultima distancia valida (persistencia)
  }
}

// ============================================================
// Actuador: buzzer pasivo. Un buzzer pasivo no suena solo con
// HIGH/LOW: necesita que se le entregue la señal de la frecuencia
// que lo hace vibrar, por eso se usa tone()/noTone().
// ============================================================
void buzzerApagado() {
  noTone(PIN_BUZZER);
}

void buzzerPulso(uint32_t periodo_ms, uint16_t frecuencia_hz) {
  // Pulso no bloqueante: en vez de "prender, esperar, apagar" con
  // delay(), se pregunta en cada vuelta si ya toca alternar.
  static uint32_t t_ultimo = 0;
  static bool on = false;
  if (millis() - t_ultimo >= periodo_ms) {
    t_ultimo = millis();
    on = !on;
    if (on) tone(PIN_BUZZER, frecuencia_hz);
    else    buzzerApagado();
  }
}

// ============================================================
// Actuador: motor de vibracion. SIEMPRE via transistor NPN (base <-
// resistencia <- GPIO; colector <- motor <- riel 5V; emisor a GND
// comun; diodo flyback en el motor). NUNCA el motor directo al GPIO
// ni al pin de 3,3V: el GPIO no entrega la corriente que pide un
// motor. El GPIO solo maneja la base del transistor.
// ============================================================
void vibrador(bool encendido) {
  digitalWrite(PIN_VIBRA, encendido ? HIGH : LOW);
}

void vibradorPulso(uint32_t periodo_ms) {
  // Mismo patron no bloqueante que el buzzer.
  static uint32_t t_ultimo = 0;
  static bool on = false;
  if (millis() - t_ultimo >= periodo_ms) {
    t_ultimo = millis();
    on = !on;
    vibrador(on);
  }
}

// ============================================================
// GPS NEO-6M por UART2 (Serial2). leerGPS() drena la UART sin
// bloquear y le pasa cada byte a TinyGPS++, que arma las tramas NMEA
// y calcula lat/long/altitud/satelites por su cuenta.
// ============================================================
void leerGPS() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_VIBRA, OUTPUT);

  buzzerApagado();
  vibrador(false);

  Serial.println("P26 paso 3 - FSM + buzzer + motor + GPS");
  cambiar(REPOSO);
}

// ============================================================
// LOOP: leerDistancia() y leerGPS() cada vuelta (ambas se
// autolimitan/drenan sin bloquear), luego el switch/case recorre el
// grafo. Sin delay() en ningun punto del lazo ni de las funciones que
// llama.
// ============================================================
void loop() {
  leerDistancia();
  leerGPS();

  switch (estado) {

    case REPOSO:
      buzzerApagado();
      vibrador(false);
      if (distancia_cm < D_LEJOS_ON_CM) cambiar(OBSTACULO_LEJOS);
      break;

    case OBSTACULO_LEJOS:
      buzzerPulso(T_BUZZER_LEJOS_MS, FREQ_BUZZER_LEJOS);   // aviso suave
      vibradorPulso(T_VIBRA_LEJOS_MS);                     // pulso suave
      if (distancia_cm < D_CERCA_ON_CM)       cambiar(OBSTACULO_CERCA);
      else if (distancia_cm > D_LEJOS_OFF_CM) cambiar(REPOSO);
      break;

    case OBSTACULO_CERCA:
      buzzerPulso(T_BUZZER_CERCA_MS, FREQ_BUZZER_CERCA);   // aviso urgente
      vibrador(true);                                      // continuo
      if (distancia_cm > D_CERCA_OFF_CM) cambiar(OBSTACULO_LEJOS);
      break;
  }

  // Diagnostico en vivo, no bloqueante: sin LEDs, esto es lo que
  // permite ver que la distancia y el estado se mueven bien al probar.
  if (millis() - t_serial >= T_SERIAL_MS) {
    t_serial = millis();
    Serial.printf("d=%.1f cm  estado=%u\n", distancia_cm, (unsigned)estado);
  }

  // GPS: valores ya calculados por TinyGPS++, cada 5 s.
  if (millis() - t_gps >= T_GPS_MS) {
    t_gps = millis();
    if (gps.location.isValid()) {
      Serial.printf("GPS: lat=%.6f  lon=%.6f  alt=%.1fm  sat=%u\n",
                    gps.location.lat(), gps.location.lng(),
                    gps.altitude.meters(),
                    gps.satellites.isValid() ? gps.satellites.value() : 0);
    } else {
      Serial.printf("GPS: esperando fix (sat=%u, chars=%lu)\n",
                    gps.satellites.isValid() ? gps.satellites.value() : 0,
                    gps.charsProcessed());
    }
  }
}

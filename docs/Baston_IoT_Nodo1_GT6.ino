// ============================================================
// Fundamentos de IoT 2026-2 - P26 - Baston de asistencia para
// discapacidad visual - E32
// VERSION GT6 (Semana 10): FSM del baston + datos GT4 (MQTT) con GPS +
// COMANDO REMOTO por /cmd (Node-RED -> ESP32) como una entrada mas de la FSM.
//
// ---- Cambios GT6 (Anexo C de la guia) ----
//  1. onMqttMessage() SOLO registra el comando ("ON"/"OFF") en cmdRemoto.
//     No toca el hardware: la FSM decide (loop()).
//  2. Estado nuevo ACTUANDO_REMOTO (estado = 4 en el JSON). Un ON solo
//     produce la transicion desde REPOSO. Enciende el INDICADOR (LED).
//  3. Validacion de estado: en OBSTACULO_LEJOS, OBSTACULO_CERCA y
//     ERROR_SEGURO el comando se IGNORA (manda la logica local).
//  4. T_MAX_MS: si no llega OFF ni un ON de refresco, vuelve a REPOSO
//     (una caida de red no deja el indicador encendido).
//  5. Prioridad local: en ACTUANDO_REMOTO el baston sigue vigilando. Si
//     aparece un obstaculo (< D_LEJOS_ON_CM) o falla el sensor, la FSM
//     sale de ACTUANDO_REMOTO y avisa como siempre.
//  6. El comando actua sobre un INDICADOR (LED), NO sobre el buzzer ni el
//     motor de vibracion (actuadores principales del proyecto).
//  7. Hilos: el callback corre en el nucleo 0 (tarea de red) y la FSM en
//     el nucleo 1; el comando se pasa con una seccion critica.
//
// Base: version con GPS. Incluye la POSICION GPS en el JSON.
//
// Base: p26_calibrado (FSM + buzzer + motor de vibracion + boton)
// al que se le agrega el envio de datos de GT4 (ESP32 -> Mosquitto ->
// Telegraf -> InfluxDB -> Grafana) y las correcciones de la
// retroalimentacion tecnica NP1.
//
// Topicos (4 niveles exactos, item A2):
//   curso/E32/P26/nodo1          JSON solo numeros, entra a Telegraf
//   curso/E32/P26/nodo1/estado   "online" / "offline" (LWT)   solo MQTT
//   curso/E32/P26/nodo1/cmd      comandos entrantes (JSON)    {"cmd":"ON"} o {"cmd":"OFF"}
//
// Variables publicadas, una vez cada 5 s (lo que falla se omite y se avisa
// con su bandera):
//   distancia_cm  cm      distancia corregida (se omite si no hay eco o hay error)
//   sin_eco       0-1     1 = nada en rango (sin eco)
//   estado        0-4     FSM: 0 REPOSO, 1 OBSTACULO_LEJOS, 2 OBSTACULO_CERCA,
//                         3 ERROR_SEGURO (sensor mudo o lecturas imposibles),
//                         4 ACTUANDO_REMOTO (GT6: comando ON activo)
//   lat, lon      grados  posicion GPS (se omiten si no hay fix)
//   sin_gps       0-1     1 = sin fix GPS (o no llegan tramas del modulo)
//   estado_mqtt   0-1     1 = conexion MQTT estable desde el envio anterior;
//                         0 = hubo una caida y reconexion (primer dato al volver)
// Ejemplo: {"distancia_cm":123.4,"sin_eco":0,"estado":1,"lat":-33.456790,"lon":-70.602057,"sin_gps":0,"estado_mqtt":1}
// Serial (115200): transiciones de estado (con su motivo), comandos recibidos
// ([cmd recibido] ON/OFF) y fallos de conexion MQTT.
// Nunca se envia 999 ni texto en un campo numerico (rompe InfluxDB).
// En Grafana: Value mappings (sin_eco: 1 = "Sin eco", 0 = "Con eco").
//
// ---- Correcciones NP1 hechas en este codigo ----
//  [ALTA]  TIMEOUT_US 12000 -> 16000 (~2,8 m de alcance). Con 12000 el
//          alcance era ~2,06 m y el umbral de 250 cm era inalcanzable.
//  [ALTA]  "Sin eco" YA NO se convierte en 999 cm. La lectura se separa
//          en medicionValida (bool) y distancia_cm (solo se usa si la
//          medicion es valida). Sensor MUDO (ECHO no responde al disparo)
//          o lecturas imposibles repetidas -> ERROR_SEGURO. Un "sin eco"
//          con el sensor vivo (nada delante) NO es falla.
//  [ALTA]  Estado ERROR_SEGURO explicito (antes ERROR_MEDICION), con sus
//          contadores de error (contMudo, contImposibles); se publica como
//          estado = 3. Recuperacion segura: solo rearma si el sensor volvio
//          Y el usuario presiona el boton.
//  [MEDIA] pulseIn() bloqueaba hasta 12 ms (ahora seria hasta 16 ms). El
//          eco se mide ahora con una interrupcion + micros(): el lazo
//          nunca espera al sensor.
//  [MEDIA] GPS: si no llegan tramas (revisar cruce TX->RX / RX->TX,
//          alimentacion y baudios) se publica sin_gps = 1.
//  [ALTA]  Calibracion rehecha con 113 lecturas de 50 a 250 cm (cubre el
//          rango donde decide la FSM). Los umbrales de histeresis son los
//          definidos por el equipo (seccion 3).
//
// GT4 + FSM: la red (WiFi/MQTT) corre en una TAREA APARTE (nucleo 0).
// mqtt.connect() puede bloquear varios segundos si el broker no
// responde; en el lazo principal congelaria la FSM justo cuando el
// usuario necesita el aviso. Asi la FSM nunca espera a la red.
//
// Conexion:
//   HC-SR04 VCC -> 5V   GND -> GND
//   TRIG -> GPIO 26 (directo)
//   ECHO -> divisor 10 k / 20 k -> GPIO 25   (obligatorio: ECHO sale
//                                              en 5V, el ESP32 lee 3,3V)
//   Buzzer pasivo -> GPIO 32 (+ resistencia en serie si el modulo la pide)
//   Motor de vibracion -> GPIO 33 -> base de transistor NPN (con su
//          resistencia); colector al motor; motor al riel de 5V; emisor a
//          GND comun; diodo flyback en paralelo con el motor.
//   Boton -> un terminal a 3,3V y el otro a GPIO 27 (INPUT_PULLDOWN):
//            suelto = LOW, presionado = HIGH. Opcional: resistencia de
//            10 k de GPIO 27 a GND para un pull-down mas firme.
//   Indicador remoto (GT6) -> GPIO 2: LED integrado de la mayoria de los
//            ESP32 DevKit (sin cableado). Si tu placa no lo tiene: LED externo
//            con resistencia de 220 ohm de GPIO 2 a GND.
//   GPS NEO-6M -> VCC a 3,3V o 5V (segun el modulo), GND a GND comun
//                 TX del GPS -> GPIO 17 (RX del ESP32, UART2)
//                 RX del GPS -> GPIO 16 (TX del ESP32, UART2)
//
// Nota: tone()/noTone() requieren el core ESP32 Arduino 2.x o superior.
//
// PROTOTIPO EDUCATIVO. No es un dispositivo de asistencia ni
// sustituye al baston blanco.
// ============================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>     // v7.x
#include <TinyGPS++.h>        // libreria "TinyGPSPlus" de Mikal Hart
#include "config.h"          // credenciales del equipo, NO va al repositorio

// 1 = broker del curso (envia usuario y clave)
// 0 = test.mosquitto.org (rechaza con rc=5 si recibe credenciales)
#define USAR_AUTH 1

// Acepta el config.h de p26 (ID_EQUIPO, ID_PROYECTO, ID_NODO) o el del
// demo del curso (MQTT_USER, PROYECTO, NODO).
#ifndef ID_EQUIPO
#define ID_EQUIPO   MQTT_USER
#endif
#ifndef ID_PROYECTO
#define ID_PROYECTO PROYECTO
#endif
#ifndef ID_NODO
#define ID_NODO     NODO
#endif

// ---------- 1. Pines ----------
const int PIN_TRIG   = 26;
const int PIN_ECHO   = 25;
const int PIN_BUZZER = 32;   // buzzer pasivo, se maneja con tone()
const int PIN_VIBRA  = 33;   // motor de vibracion, via transistor NPN
const int PIN_BOTON  = 27;   // boton de rearme a 3,3V, INPUT_PULLDOWN (presionado = HIGH)
const int PIN_INDICADOR = 2; // GT6: LED indicador del comando remoto (LED integrado)
const int GPS_RX_PIN = 17;   // ESP32 RX  <- TX del GPS
const int GPS_TX_PIN = 16;   // ESP32 TX  -> RX del GPS

// ---------- 2. Sensor ----------
// Alcance util: el timeout debe cubrir el umbral MAS lejano de la FSM
// (D_LEJOS_OFF_CM = 250 cm). Un eco a 250 cm tarda ~14,3 ms; con 16 ms
// el alcance llega a ~2,8 m y 250 cm si es medible (NP1).
const unsigned long TIMEOUT_US    = 16000;   // ~2,8 m de alcance
const unsigned long MARGEN_ECO_US = 2000;    // el eco sube ~0,5 ms despues del disparo
const float VELOCIDAD_CM_US       = 0.0343;
const unsigned long PERIODO_MS    = 60;      // pausa minima entre disparos
const float DIST_MIN_PLAUSIBLE_CM = 2.0;     // bajo esto se descarta (ruido)

// Calibracion propia (item D1): regresion lineal de lecturas crudas
// (t_eco * 0,0343 / 2) contra distancias reales medidas con huincha, en 5
// distancias (50/100/150/200/250 cm; n = 23/24/22/19/25 = 113 lecturas):
//     medido = CAL_M * real + CAL_B      R2 = 0,9988, error tipico 2,5 cm
// medido = lo que lee el sensor; real = la huincha. Para corregir se
// despeja:   real = (medido - CAL_B) / CAL_M.   Rango validado: 50-250 cm.
const float CAL_M = 0.9843;
const float CAL_B = 1.5110;   // cm

// ---------- 2.1 Deteccion de falla -> ERROR_SEGURO ----------
// Dos condiciones INDEPENDIENTES, cada una detecta una falla distinta:
//   - Sensor MUDO: tras cada disparo, un HC-SR04 conectado SIEMPRE
//     responde en ECHO: un pulso del ancho del eco o, si no hay nada
//     delante, un pulso largo (~38 ms) que significa "sin eco". Si tras
//     CICLOS_MUDO_MAX disparos seguidos ECHO no se mueve (o queda fijo en
//     alto) no llega NADA del sensor: desconectado, cable roto, sin
//     alimentacion -> ERROR_SEGURO en menos de 1 s.
//   - LECTURAS_IMPOSIBLES_MAX: eco a una distancia fisicamente imposible
//     (< 2 cm) de forma repetida -> ruido o falla electrica.
// "Sin eco" con el sensor VIVO (nada delante) NO es falla: es camino
// libre, por largo que sea el tramo despejado.
// Si tu modulo NO da el pulso largo cuando no hay nada delante (algunos
// clones), pon SENSOR_DA_PULSO_SIN_ECO en 0: se vuelve a la regla de
// TIMEOUT_SIN_ECO_MS (mucho tiempo sin ningun eco valido); en ese modo un
// tramo despejado de mas de 8 s SI entra a ERROR_SEGURO.
#define SENSOR_DA_PULSO_SIN_ECO 1
const uint8_t  CICLOS_MUDO_MAX         = 5;      // 5 disparos seguidos sin senal
const uint32_t ECHO_ATASCADO_MS        = 100;    // ECHO en alto mas que esto (el pulso largo dura ~38 ms) = atascado
const uint8_t  LECTURAS_IMPOSIBLES_MAX = 5;      // "cinco lecturas imposibles"
const uint32_t TIMEOUT_SIN_ECO_MS      = 8000;   // solo si SENSOR_DA_PULSO_SIN_ECO = 0

// Sin eco "corto": cuantas lecturas SEGUIDAS sin eco (5 x 60 ms = 0,3 s)
// se interpretan como "nada en rango" (camino libre). No es una distancia:
// es una bandera. Tambien es la bandera sin_eco que se publica por MQTT.
const uint8_t  LECTURAS_SIN_ECO_LIBRE  = 5;

// ---------- 3. Umbrales de la FSM (histeresis) ----------
// Umbrales definidos por el equipo; todos caen dentro del rango calibrado
// (50-250 cm).
const float D_LEJOS_ON_CM  = 170.0;   // < 1,7 m -> entra a OBSTACULO_LEJOS
const float D_LEJOS_OFF_CM = 190.0;   // > 1,9 m -> vuelve a REPOSO
const float D_CERCA_ON_CM  = 90.0;    // < 0,9 m -> entra a OBSTACULO_CERCA
const float D_CERCA_OFF_CM = 110.0;   // > 1,1 m -> vuelve a OBSTACULO_LEJOS

// ---------- 4. Ritmo de aviso: buzzer + motor por estado ----------
const uint32_t T_BUZZER_LEJOS_MS = 500;    // pitido lento: aviso suave
const uint16_t FREQ_BUZZER_LEJOS = 1500;   // tono grave
const uint32_t T_BUZZER_CERCA_MS = 150;    // pitido rapido: aviso urgente
const uint16_t FREQ_BUZZER_CERCA = 2500;   // tono agudo
const uint32_t T_VIBRA_LEJOS_MS  = 500;    // pulso del motor en LEJOS

// Patron de ERROR_SEGURO: tres pitidos cortos y agudos del buzzer, luego
// pausa; distinto de los avisos de distancia.
const uint16_t FREQ_BUZZER_ERROR = 3500;
const uint32_t T_ERROR_PULSO_MS  = 120;
const uint32_t T_ERROR_PAUSA_MS  = 700;
const uint8_t  N_ERROR_PULSOS    = 3;

const uint32_t T_ANTIRREBOTE_MS  = 40;   // antirrebote del boton

// ---------- 5. Estado de la FSM ----------
// El numero de cada estado es el que viaja en el JSON (campo "estado"):
// NO reordenar sin actualizar Grafana (value mappings) y Node-RED (ESTADOS).
enum Estado : uint8_t { REPOSO, OBSTACULO_LEJOS, OBSTACULO_CERCA, ERROR_SEGURO, ACTUANDO_REMOTO };
const char* const NOMBRES_ESTADO[] = {"REPOSO", "OBSTACULO_LEJOS", "OBSTACULO_CERCA", "ERROR_SEGURO", "ACTUANDO_REMOTO"};
Estado estado = REPOSO;
uint32_t t_entrada = 0;   // millis() al entrar al estado actual

// ---------- 5.1 Comando remoto (GT6, Anexo C) ----------
// El callback MQTT (nucleo 0) solo ANOTA el comando aqui; la FSM (nucleo 1)
// lo toma en cada vuelta con tomarComandoRemoto() y decide. Se protege con
// una seccion critica porque lo tocan dos nucleos distintos.
enum CmdRemoto : uint8_t { CMD_NINGUNO, CMD_ON, CMD_OFF };
volatile CmdRemoto cmdRemoto = CMD_NINGUNO;
portMUX_TYPE muxCmd = portMUX_INITIALIZER_UNLOCKED;
uint32_t t_actuacion = 0;                    // millis() del ultimo ON (entrada o refresco)
const uint32_t T_MAX_MS = 60000;             // maximo en ACTUANDO_REMOTO sin refresco
                                             // (>= 3 x PERIODO_PUB_MS, ver Anexo D)

CmdRemoto tomarComandoRemoto() {             // leer y borrar, de forma atomica
  portENTER_CRITICAL(&muxCmd);
  CmdRemoto c = cmdRemoto;
  cmdRemoto = CMD_NINGUNO;
  portEXIT_CRITICAL(&muxCmd);
  return c;
}

// ---------- 6. Medicion (NP1: validez SEPARADA de la distancia) ----------
// medicionValida: la ULTIMA lectura fue un eco valido. distancia_cm solo
// tiene sentido si medicionValida es true (o como "ultima distancia buena"
// para telemetria); NUNCA se usa un valor centinela como 999.
bool     medicionValida = false;
float    distancia_cm   = NAN;
uint8_t  contSinEco     = 0;     // lecturas seguidas SIN eco (nada delante: camino libre)
uint8_t  contMudo       = 0;     // disparos seguidos SIN NINGUNA senal en ECHO (sensor mudo)
uint8_t  contImposibles = 0;     // lecturas seguidas con eco < 2 cm (ruido)
uint32_t t_ultimo_eco_valido = 0;   // millis() del ultimo eco valido
uint32_t t_sensor = 0;

// ---------- 6.1 Captura del eco por interrupcion (NP1: sin pulseIn) ----------
// La interrupcion solo anota los instantes de subida y bajada del pulso
// ECHO; la cuenta se hace despues, en el lazo, sin esperar al sensor.
volatile uint32_t t_subida_us   = 0;
volatile uint32_t ancho_echo_us = 0;
volatile bool     echo_completo = false;
volatile bool     subida_vista  = false;   // ECHO subio tras el ultimo disparo: el sensor RESPONDE

void IRAM_ATTR isrEcho() {
  if (digitalRead(PIN_ECHO)) {
    t_subida_us = micros();                      // flanco de subida
    subida_vista = true;
  } else {
    ancho_echo_us = micros() - t_subida_us;      // flanco de bajada
    echo_completo = true;
  }
}

enum FaseSensor : uint8_t { S_ESPERA, S_MIDIENDO };
FaseSensor faseSensor = S_ESPERA;
uint32_t   t_disparo_us = 0;

// ---------- 7. GPS: objeto TinyGPS++ (valores ya calculados) ----------
const uint32_t GPS_EDAD_MAX_MS = 5000;   // posicion mas vieja que esto = sin fix
TinyGPSPlus gps;

// ---------- 9. Boton: antirrebote con millis() ----------
bool botonCruda    = LOW;    // ultima lectura sin filtrar (suelto = LOW)
bool botonEstable  = LOW;    // lectura ya antirrebotada
uint32_t t_antirrebote = 0;
bool botonFlanco   = false;  // true por un ciclo cuando hay flanco de presion

// ---------- 10. Red (GT4): WiFi + MQTT ----------
#define TOPIC_BASE "curso/" ID_EQUIPO "/" ID_PROYECTO "/" ID_NODO
const char* TOPIC        = TOPIC_BASE;             // datos (Telegraf)
const char* TOPIC_ESTADO = TOPIC_BASE "/estado";   // solo MQTT
const char* TOPIC_CMD    = TOPIC_BASE "/cmd";      // solo MQTT
const char* CLIENT_ID    = ID_EQUIPO "-" ID_NODO;

// Estado retenido: un suscriptor nuevo ve de inmediato online/offline.
// La diapositiva 9 dice "no usar retained en /estado"; si el docente
// prefiere eso: false.
const bool ESTADO_RETENIDO = true;
// Datos NO retenidos (como el demo): con retained, Telegraf reconecta y
// guarda el ultimo dato otra vez como lectura nueva (punto fantasma).
const bool DATOS_RETENIDOS = false;

const uint32_t PERIODO_PUB_MS     = 5000;   // minimo del curso: 5 s
const uint32_t REINTENTO_WIFI_MS  = 15000;
const uint32_t ESPERA_INICIAL     = 2000;   // backoff MQTT: 2, 4, 8, 16, 30 s
const uint32_t ESPERA_MAXIMA      = 30000;
const uint32_t T_INSTANTANEA_MS   = 250;    // cada cuanto el lazo actualiza la foto para la red

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
uint32_t t_pub = 0;
uint32_t t_wifi = 0;
uint32_t t_reconex = 0;
uint32_t esperaReconex = ESPERA_INICIAL;
bool mqttEstabaConectado = false;   // para detectar una caida de la conexion
bool mqttCaida = false;             // hubo una caida desde el ultimo envio (estado_mqtt = 0)

// "Foto" de los datos: la deja lista el lazo principal (nucleo 1) y la lee
// la tarea de red (nucleo 0). Se copia entera dentro de una seccion critica.
struct Instantanea {
  float   distancia_cm;
  bool    publicarDistancia;   // false: no se envia distancia_cm
  bool    sinEco;
  uint8_t estado;
  bool    hayGps;              // fix reciente
  double  lat;
  double  lon;
};
Instantanea instantanea = {};
portMUX_TYPE muxInstantanea = portMUX_INITIALIZER_UNLOCKED;
uint32_t t_instantanea = 0;

// ============================================================
// Unica funcion que escribe 'estado': asi el reloj del estado nuevo
// siempre parte en cero y queda registrada la transicion.
// ============================================================
void cambiar(Estado e, const char* motivo) {
  Serial.printf("[%lu ms] %s -> %s  (%s)\n", (unsigned long)millis(),
                NOMBRES_ESTADO[estado], NOMBRES_ESTADO[e], motivo);
  estado = e;
  t_entrada = millis();
}

// ============================================================
// Sensor NO bloqueante. Dos fases:
//   S_ESPERA   : cuando pasaron PERIODO_MS y ECHO esta en bajo, dispara.
//   S_MIDIENDO : espera (sin bloquear) a que la interrupcion entregue el
//                ancho del pulso, o a que venza el timeout.
// Un pulso mas largo que TIMEOUT_US (el HC-SR04 deja ECHO en alto ~38 ms
// cuando no hay nada) o ningun pulso = "sin eco".
// ============================================================
void procesarLectura(uint32_t ancho_us) {
  if (ancho_us == 0 || ancho_us > TIMEOUT_US) {
    // Sin eco en rango (nada delante). NO se inventa una distancia: solo
    // se cuenta. Si el sensor estuviera MUDO lo detecta contMudo, en
    // actualizarSensor(), mirando si ECHO respondio al disparo.
    medicionValida = false;
    if (contSinEco < 250) contSinEco++;
    return;
  }

  float d_bruta = (ancho_us * VELOCIDAD_CM_US) / 2.0;
  float d = (d_bruta - CAL_B) / CAL_M;       // calibracion propia (seccion 2)

  if (d >= DIST_MIN_PLAUSIBLE_CM) {
    medicionValida = true;
    distancia_cm = d;
    t_ultimo_eco_valido = millis();          // lectura valida: refresca el reloj
    contImposibles = 0;
    contSinEco = 0;
  } else {
    // Eco recibido pero a una distancia fisicamente imposible: descarta
    // la lectura y suma a la racha de ruido.
    medicionValida = false;
    contSinEco = 0;
    if (contImposibles < 250) contImposibles++;
  }
}

void actualizarSensor() {
  if (faseSensor == S_ESPERA) {
    if (millis() - t_sensor < PERIODO_MS) return;
    if (digitalRead(PIN_ECHO) == HIGH) {
      // El pulso anterior aun no termina (dura < 40 ms)... o ECHO esta
      // ATASCADO en alto (falla): pasado ECHO_ATASCADO_MS se cuenta como
      // "sin senal" y se vuelve a evaluar un rato despues.
      if (millis() - t_sensor > ECHO_ATASCADO_MS) {
        t_sensor = millis();
        if (contMudo < 250) contMudo++;
      }
      return;
    }
    t_sensor = millis();
    echo_completo = false;                       // descarta un flanco viejo
    subida_vista  = false;

    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);                       // protocolo electrico del HC-SR04
    digitalWrite(PIN_TRIG, LOW);
    t_disparo_us = micros();
    faseSensor = S_MIDIENDO;
    return;
  }

  // S_MIDIENDO
  if (echo_completo && subida_vista) {           // pulso completo (un flanco de bajada suelto es ruido)
    uint32_t ancho = ancho_echo_us;
    echo_completo = false;
    contMudo = 0;                                // el sensor respondio
    procesarLectura(ancho);
    faseSensor = S_ESPERA;
  } else if (micros() - t_disparo_us > TIMEOUT_US + MARGEN_ECO_US) {
    // Vencio el tiempo sin pulso completo. Si ECHO SUBIO pero no bajo, es
    // el pulso largo de "nada delante": el sensor esta vivo. Si ni siquiera
    // subio, el sensor no respondio: suma a "mudo".
    if (subida_vista)          contMudo = 0;
    else if (contMudo < 250)   contMudo++;
    procesarLectura(0);                          // sin eco en rango
    faseSensor = S_ESPERA;
  }
}

// ============================================================
// Falla del sensor: dos condiciones independientes (ver 2.1). Se
// consulta desde los tres estados normales antes de sus transiciones.
// ============================================================
bool huboFallaSensor() {
#if SENSOR_DA_PULSO_SIN_ECO
  bool sensorMudo = (contMudo >= CICLOS_MUDO_MAX);                       // NADA llega por ECHO
#else
  bool sensorMudo = (millis() - t_ultimo_eco_valido) > TIMEOUT_SIN_ECO_MS;   // sin ningun eco valido por mucho tiempo
#endif
  bool lecturasImposibles = (contImposibles >= LECTURAS_IMPOSIBLES_MAX);
  return sensorMudo || lecturasImposibles;
}

// "Nada en rango": varias lecturas SEGUIDAS sin eco (bandera, no distancia).
bool caminoLibre() {
  return contSinEco >= LECTURAS_SIN_ECO_LIBRE;
}

// ============================================================
// Actuador: buzzer pasivo (tone()/noTone(), necesita la frecuencia).
// ============================================================
bool buzzerSuena = true;   // true al inicio: asi setup() fuerza un primer noTone()

void buzzerTono(uint16_t frecuencia_hz) {
  tone(PIN_BUZZER, frecuencia_hz);
  buzzerSuena = true;
}

void buzzerApagado() {
  if (!buzzerSuena) return;   // no martillar el driver LEDC en cada vuelta del lazo
  noTone(PIN_BUZZER);
  buzzerSuena = false;
}

void buzzerPulso(uint32_t periodo_ms, uint16_t frecuencia_hz) {
  // Pulso no bloqueante: se pregunta en cada vuelta si ya toca alternar.
  static uint32_t t_ultimo = 0;
  static bool on = false;
  if (millis() - t_ultimo >= periodo_ms) {
    t_ultimo = millis();
    on = !on;
    if (on) buzzerTono(frecuencia_hz);
    else    buzzerApagado();
  }
}

// ============================================================
// Actuador: motor de vibracion. SIEMPRE via transistor NPN (el GPIO solo
// maneja la base). NUNCA el motor directo al GPIO ni al pin de 3,3V.
// ============================================================
void vibrador(bool encendido) {
  digitalWrite(PIN_VIBRA, encendido ? HIGH : LOW);
}

void vibradorPulso(uint32_t periodo_ms) {
  static uint32_t t_ultimo = 0;
  static bool on = false;
  if (millis() - t_ultimo >= periodo_ms) {
    t_ultimo = millis();
    on = !on;
    vibrador(on);
  }
}

// ============================================================
// Indicador remoto (GT6): LED en GPIO 2. Lo maneja SOLO el final de loop():
// queda encendido exactamente mientras estado == ACTUANDO_REMOTO, asi no
// puede quedar prendido al salir del estado por ninguna ruta.
// ============================================================
void indicador(bool encendido) {
  digitalWrite(PIN_INDICADOR, encendido ? HIGH : LOW);
}

// ============================================================
// Actuador: patron de ERROR_SEGURO (3 pitidos cortos + pausa) con el
// buzzer. Se calcula desde t_entrada y solo actua cuando cambia de fase.
// ============================================================
void errorPulso() {
  const uint32_t duracionPulsos = N_ERROR_PULSOS * 2 * T_ERROR_PULSO_MS;
  const uint32_t ciclo          = duracionPulsos + T_ERROR_PAUSA_MS;
  uint32_t fase = (millis() - t_entrada) % ciclo;
  bool encendido = (fase < duracionPulsos) && (((fase / T_ERROR_PULSO_MS) % 2) == 0);

  static bool anterior = false;
  if (encendido != anterior) {
    anterior = encendido;
    if (encendido) buzzerTono(FREQ_BUZZER_ERROR);
    else           buzzerApagado();
  }
}

// ============================================================
// Boton de rearme: antirrebote con millis() (sin delay()).
// ============================================================
void actualizarBoton() {
  bool lectura = digitalRead(PIN_BOTON);
  botonFlanco = false;

  if (lectura != botonCruda) {
    botonCruda = lectura;
    t_antirrebote = millis();
  }

  if ((millis() - t_antirrebote) > T_ANTIRREBOTE_MS && botonCruda != botonEstable) {
    botonEstable = botonCruda;
    if (botonEstable == HIGH) botonFlanco = true;  // flanco de presion (3,3V al pin)
  }
}

// ============================================================
// GPS NEO-6M por UART2 (Serial2): drena la UART sin bloquear y le pasa
// cada byte a TinyGPS++, que arma las tramas NMEA.
// ============================================================
void leerGPS() {
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }
}

// ============================================================
// Foto de datos para la red: se arma en el lazo principal.
// ============================================================
void actualizarInstantanea() {
  Instantanea n;
  n.sinEco            = caminoLibre();
  // No se publica una distancia vieja como si fuera actual: en error o sin
  // eco se omite distancia_cm y el estado/bandera lo explican.
  n.publicarDistancia = !n.sinEco && estado != ERROR_SEGURO && !isnan(distancia_cm);
  n.distancia_cm      = distancia_cm;
  n.estado            = (uint8_t)estado;
  n.hayGps = gps.location.isValid() && gps.location.age() < GPS_EDAD_MAX_MS;
  n.lat    = gps.location.lat();
  n.lon    = gps.location.lng();

  portENTER_CRITICAL(&muxInstantanea);
  instantanea = n;
  portEXIT_CRITICAL(&muxInstantanea);
}

// ============================================================
// Comandos entrantes en .../cmd (GT6, Anexo C.1).
// REGLA: el callback REGISTRA el comando y NO toca el hardware. Corre dentro
// de mqtt.loop() en la tarea de red (nucleo 0); la FSM lo consume en loop().
// Formato esperado: {"cmd":"ON"} o {"cmd":"OFF"} (lo arma Node-RED).
// ============================================================
void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  JsonDocument doc;                                   // v7: sin tamanio
  DeserializationError err = deserializeJson(doc, payload, len);
  if (err) {                                          // JSON invalido: se ignora
    Serial.printf("[cmd] JSON invalido: %s\n", err.c_str());
    return;
  }
  const char* cmd = doc["cmd"] | "";

  CmdRemoto nuevo = CMD_NINGUNO;
  if      (strcmp(cmd, "ON")  == 0) nuevo = CMD_ON;
  else if (strcmp(cmd, "OFF") == 0) nuevo = CMD_OFF;

  if (nuevo == CMD_NINGUNO) {                         // otro valor: se descarta
    Serial.printf("[cmd] comando desconocido: \"%s\"\n", cmd);
    return;
  }

  portENTER_CRITICAL(&muxCmd);
  cmdRemoto = nuevo;
  portEXIT_CRITICAL(&muxCmd);
  Serial.printf("[cmd recibido] %s\n", cmd);
}

// ============================================================
// WiFi sin bloquear y sin cortar una asociacion en curso
// ============================================================
void mantenerWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  uint32_t ahora = millis();
  if (ahora - t_wifi < REINTENTO_WIFI_MS) return;
  t_wifi = ahora;
  WiFi.reconnect();
}

// ============================================================
// MQTT con testamento (LWT) y espera creciente. Item B1: se resuscribe
// a /cmd en CADA reconexion, porque el broker olvida las suscripciones.
// ============================================================
void mantenerMQTT() {
  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;      // sin red no tiene sentido
  uint32_t ahora = millis();
  if (ahora - t_reconex < esperaReconex) return;
  t_reconex = ahora;

  // Testamento: si el nodo desaparece, el BROKER publica "offline" (QoS 1)
  if (mqtt.connect(CLIENT_ID,
                   USAR_AUTH ? MQTT_USER : nullptr,
                   USAR_AUTH ? MQTT_PASS : nullptr,
                   TOPIC_ESTADO, 1, ESTADO_RETENIDO, "offline")) {
    mqtt.publish(TOPIC_ESTADO, "online", ESTADO_RETENIDO);
    mqtt.subscribe(TOPIC_CMD, 1);
    esperaReconex = ESPERA_INICIAL;
  } else {
    Serial.printf("[mqtt] FALLO rc=%d\n", mqtt.state());
    esperaReconex = (esperaReconex * 2 > ESPERA_MAXIMA) ? ESPERA_MAXIMA
                                                        : esperaReconex * 2;
  }
}

// ============================================================
// Publicacion: JSON plano, solo numeros (ver la tabla de variables en la
// cabecera). Lo que falla se omite y se avisa con su bandera.
// ============================================================
// Agregado: identificador numerico del nodo para MQTT.
// Se mantiene el JSON compuesto solo por valores numericos.
void agregarIdentificadorNodo(JsonDocument& doc) {
  doc["nodo"] = 1;
}

// Muestra en el Monitor Serie el mismo JSON enviado correctamente por MQTT.
void mostrarPublicacionSerial(const char* mensaje, size_t longitud) {
  Serial.print("Nodo 1: ");
  Serial.write((const uint8_t*)mensaje, longitud);
  Serial.println();
}

void publicarDatos() {
  if (!mqtt.connected()) return;

  Instantanea s;
  portENTER_CRITICAL(&muxInstantanea);
  s = instantanea;
  portEXIT_CRITICAL(&muxInstantanea);

  // Los textos deben vivir hasta serializeJson()
  char sDist[12] = "";
  if (s.publicarDistancia) snprintf(sDist, sizeof(sDist), "%.1f", s.distancia_cm);
  char sLat[16] = "", sLon[16] = "";
  if (s.hayGps) {
    snprintf(sLat, sizeof(sLat), "%.6f", s.lat);   // 6 decimales ~ 0,1 m
    snprintf(sLon, sizeof(sLon), "%.6f", s.lon);
  }

  JsonDocument doc;                               // v7: sin StaticJsonDocument
  agregarIdentificadorNodo(doc);  // Primera propiedad del mensaje MQTT
  if (s.publicarDistancia) doc["distancia_cm"] = serialized(sDist);
  doc["sin_eco"] = s.sinEco ? 1 : 0;
  doc["estado"]  = (int)s.estado;
  if (s.hayGps) {
    doc["lat"] = serialized(sLat);
    doc["lon"] = serialized(sLon);
  }
  doc["sin_gps"] = s.hayGps ? 0 : 1;
  doc["estado_mqtt"] = mqttCaida ? 0 : 1;         // 0 solo en el primer dato tras una caida

  char buf[160];
  size_t n = serializeJson(doc, buf, sizeof(buf));

  if (mqtt.publish(TOPIC, (const uint8_t*)buf, n, DATOS_RETENIDOS)) {
    mqttCaida = false;
    mostrarPublicacionSerial(buf, n);
  }
}

// ============================================================
// Tarea de red: corre en el nucleo 0, aparte de la FSM (nucleo 1).
// redPaso() es una vuelta completa; tareaRed() solo la repite.
// ============================================================
void redPaso() {
  mantenerWiFi();
  mantenerMQTT();
  mqtt.loop();                  // keepalive + mensajes entrantes

  bool conectado = mqtt.connected();
  if (mqttEstabaConectado && !conectado) mqttCaida = true;   // se cayo: se avisa en el proximo envio
  mqttEstabaConectado = conectado;

  uint32_t ahora = millis();
  if (ahora - t_pub >= PERIODO_PUB_MS) {
    t_pub = ahora;
    publicarDatos();
  }
}

void tareaRed(void*) {
  for (;;) {
    redPaso();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_VIBRA, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLDOWN);   // suelto = LOW, presionado = HIGH
  pinMode(PIN_INDICADOR, OUTPUT);
  indicador(false);                     // estado seguro al arrancar

  buzzerApagado();
  vibrador(false);

  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), isrEcho, CHANGE);

  t_ultimo_eco_valido = millis();   // margen inicial antes de exigir eco

  // Red: se inicia SIN esperar (la FSM arranca de inmediato) y la tarea
  // de red se encarga de conectar y reconectar.
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  t_wifi = millis();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(15);        // "offline" aparece ~23 s despues de desenchufar
  mqtt.setSocketTimeout(3);

  actualizarInstantanea();
  cambiar(REPOSO, "arranque");

  xTaskCreatePinnedToCore(tareaRed, "red", 8192, nullptr, 1, nullptr, 0);
}

// ============================================================
// LOOP: nada bloquea. El sensor, el boton y el GPS (si lo hay) se
// autolimitan; luego el switch/case recorre el grafo.
//
// GT6: el comando remoto es una ENTRADA mas de la FSM. Orden de prioridad
// dentro de cada estado: 1) falla del sensor, 2) obstaculo (logica local),
// 3) comando remoto. Si el comando y la logica local discrepan, manda la local.
// ============================================================
void loop() {
  actualizarSensor();
  leerGPS();
  actualizarBoton();

  CmdRemoto cmd = tomarComandoRemoto();   // leer y borrar, una sola vez por vuelta
  bool cmdAtendido = false;
  Estado estadoAlLlegar = estado;

  switch (estado) {

    case REPOSO:
      if (huboFallaSensor()) { cambiar(ERROR_SEGURO, "falla del sensor"); break; }
      buzzerApagado();
      vibrador(false);
      if (medicionValida && distancia_cm < D_LEJOS_ON_CM) {
        cambiar(OBSTACULO_LEJOS, "obstaculo < D_LEJOS_ON_CM");
      } else if (cmd == CMD_ON) {                          // ON: solo se acepta desde REPOSO
        cmdAtendido = true;
        t_actuacion = millis();
        cambiar(ACTUANDO_REMOTO, "cmd remoto: ON");
      } else if (cmd == CMD_OFF) {
        cmdAtendido = true;                                // ya esta apagado: nada que hacer
      }
      break;

    case ACTUANDO_REMOTO:
      // Sin avisos de distancia (igual que REPOSO), pero el indicador queda
      // encendido. El baston SIGUE vigilando: la logica local tiene prioridad.
      if (huboFallaSensor()) { cambiar(ERROR_SEGURO, "falla del sensor: manda la logica local"); break; }
      buzzerApagado();
      vibrador(false);
      if (medicionValida && distancia_cm < D_LEJOS_ON_CM) {
        cambiar(OBSTACULO_LEJOS, "obstaculo: manda la logica local");
      } else if (cmd == CMD_OFF) {
        cmdAtendido = true;
        cambiar(REPOSO, "cmd remoto: OFF");
      } else if (cmd == CMD_ON) {
        cmdAtendido = true;
        t_actuacion = millis();                            // ON repetido: solo refresca el tiempo
        Serial.println("[cmd] ON repetido: se reinicia T_MAX_MS");
      } else if (millis() - t_actuacion > T_MAX_MS) {
        cambiar(REPOSO, "T_MAX_MS sin refresco: apagado de seguridad");
      }
      break;

    case OBSTACULO_LEJOS:
      if (huboFallaSensor()) { cambiar(ERROR_SEGURO, "falla del sensor"); break; }
      buzzerPulso(T_BUZZER_LEJOS_MS, FREQ_BUZZER_LEJOS);   // aviso suave
      vibradorPulso(T_VIBRA_LEJOS_MS);                     // pulso suave
      if (medicionValida && distancia_cm < D_CERCA_ON_CM) cambiar(OBSTACULO_CERCA, "obstaculo < D_CERCA_ON_CM");
      else if (caminoLibre() || (medicionValida && distancia_cm > D_LEJOS_OFF_CM)) cambiar(REPOSO, "camino libre");
      break;

    case OBSTACULO_CERCA:
      if (huboFallaSensor()) { cambiar(ERROR_SEGURO, "falla del sensor"); break; }
      buzzerPulso(T_BUZZER_CERCA_MS, FREQ_BUZZER_CERCA);   // aviso urgente
      vibrador(true);                                      // continuo
      if (caminoLibre() || (medicionValida && distancia_cm > D_CERCA_OFF_CM)) cambiar(OBSTACULO_LEJOS, "obstaculo > D_CERCA_OFF_CM");
      break;

    case ERROR_SEGURO:
      vibrador(false);       // se apaga para no confundirlo con un aviso real
      errorPulso();          // salida local inequivoca
      // Rearme deliberado: solo si el sensor YA volvio a dar lecturas
      // normales y, ademas, el usuario presiona el boton.
      if (!huboFallaSensor() && botonFlanco) {
        cambiar(REPOSO, "boton: rearme");
      }
      break;
  }

  // El comando se consume SIEMPRE: uno ignorado no queda en espera.
  if (cmd != CMD_NINGUNO && !cmdAtendido) {
    Serial.printf("[cmd] %s IGNORADO en %s: la logica local tiene prioridad\n",
                  cmd == CMD_ON ? "ON" : "OFF", NOMBRES_ESTADO[estadoAlLlegar]);
  }

  // Indicador: encendido solo mientras el estado sea ACTUANDO_REMOTO.
  indicador(estado == ACTUANDO_REMOTO);

  // Foto de datos para la tarea de red.
  if (millis() - t_instantanea >= T_INSTANTANEA_MS) {
    t_instantanea = millis();
    actualizarInstantanea();
  }
}

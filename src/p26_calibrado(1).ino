// ============================================================
// Fundamentos de IoT 2026-2 · P26 · Baston de asistencia para
// discapacidad visual · E32 · LAB 7
// Sensores/actuadores: HC-SR04 (ultrasonico) + buzzer + motor de
// vibracion + boton/LED + GPS NEO-6M
//
// PASO 4 - Sobre el paso anterior (FSM de distancia + buzzer + motor +
// GPS) se agregan las correcciones pendientes de GT2 (item 2) y las
// prioridades 1 y 2 de la seccion 5 del informe integrador:
//   - Estado ERROR_MEDICION explicito en la FSM (correccion del item 2
//     de GT2: el grafo debia distinguir un estado de error, con salida
//     y recuperacion, en vez de tratar la falla como "sin obstaculo").
//   - Deteccion de falla que YA NO depende solo de "5 lecturas
//     imposibles seguidas" (< 2 cm): ahora tambien se vigila cuanto
//     tiempo pasa sin ningun eco valido. Asi se distingue "no hay
//     obstaculo en rango" (normal, dura poco) de una desconexion o
//     perdida total de señal (persiste, y antes se leia igual que
//     "camino libre" -> prioridad 1, critica, seccion 5).
//   - Salida local inequivoca de la falla: patron propio de LED +
//     buzzer en ERROR_MEDICION (antes el estado dejaba todo en
//     silencio) -> prioridad 2, critica, seccion 5. El motor de
//     vibracion se apaga en error para no confundirlo con un aviso de
//     obstaculo real.
//   - Boton fisico con antirrebote (40 ms, sin delay()): fuera de
//     error, el LED refleja el boton de inmediato para evidenciar que
//     el lazo no se bloquea (paso 5 de la secuencia de demostracion,
//     seccion 6); en ERROR_MEDICION, el boton solo rearma el sistema
//     si el sensor ya volvio a entregar lecturas normales (rearme
//     deliberado, paso 6 de la secuencia).
//   Item 4: FSM con enum + switch/case, sin delay(), con millis().
//   Item 5/6: actuacion de DOS actuadores (buzzer + motor) mas LED.
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
// Pendiente (no es un cambio de codigo): validar experimentalmente los
// umbrales de histeresis (200/250 cm y 100/150 cm) con mediciones
// repetidas cerca de 1, 1,5, 2 y 2,5 m en superficies duras/blandas e
// inclinadas -> prioridad 3, alta, seccion 5 del informe. Los valores
// se mantienen sin cambios porque todavia no hay datos propios que los
// respalden o ajusten.
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
//   LED -> GPIO 27 -> resistencia en serie (ej. 220 ohm) -> LED -> GND.
//   Boton -> GPIO 4 (INPUT_PULLUP) -> un terminal del boton; el otro
//            terminal a GND (presionado = LOW).
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
const int PIN_LED    = 27;   // LED: patron de error + demo de no bloqueo
const int PIN_BOTON  = 4;    // boton de rearme, INPUT_PULLUP (presionado = LOW)
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

// ---------- 2.1 Deteccion de falla (prioridad 1, seccion 5) ----------
// Dos condiciones INDEPENDIENTES disparan ERROR_MEDICION, porque cada
// una detecta un tipo distinto de falla:
//   - LECTURAS_IMPOSIBLES_MAX: eco recibido pero a una distancia
//     fisicamente imposible (< 2 cm) de forma repetida -> ruido o
//     problema electrico puntual.
//   - TIMEOUT_SIN_ECO_MS: mucho tiempo sin NINGUN eco valido -> esto
//     es lo que antes se leia siempre como "camino libre" (999 cm) y
//     que una desconexion, mala orientacion o perdida total de señal
//     produce exactamente igual. El umbral se deja bastante mas largo
//     que un ciclo de sensor para no confundir un tramo normal sin
//     obstaculos (pasillo despejado) con una falla real.
const uint8_t  LECTURAS_IMPOSIBLES_MAX = 5;      // "cinco lecturas imposibles"
const uint32_t TIMEOUT_SIN_ECO_MS      = 8000;   // 8 s sin eco valido

// ---------- 3. Umbrales de la FSM (histeresis) ----------
// Un umbral de entrada y uno de salida por frontera, para que el
// ruido del sensor no haga oscilar el estado cerca del limite.
// PENDIENTE (prioridad 3, seccion 5): validar estos valores con
// mediciones repetidas en el rango real (1 / 1,5 / 2 / 2,5 m, distintas
// superficies); por ahora se mantienen sin cambios.
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

// Patron de ERROR_MEDICION (prioridad 2, seccion 5): distinto de los
// avisos de distancia para que no se confunda con un obstaculo lejos
// o cerca -> tres pulsos cortos y agudos de LED+buzzer, luego pausa.
const uint16_t FREQ_BUZZER_ERROR = 3500;
const uint32_t T_ERROR_PULSO_MS  = 120;
const uint32_t T_ERROR_PAUSA_MS  = 700;
const uint8_t  N_ERROR_PULSOS    = 3;

const uint32_t T_ANTIRREBOTE_MS  = 40;   // antirrebote del boton

// ---------- 5. Estado de la FSM ----------
enum Estado : uint8_t { REPOSO, OBSTACULO_LEJOS, OBSTACULO_CERCA, ERROR_MEDICION };
Estado estado = REPOSO;
uint32_t t_entrada = 0;   // millis() al entrar al estado actual

// ---------- 6. Variables de medicion ----------
unsigned long t_sensor = 0;
float distancia_cm = SIN_OBSTACULO_CM;
uint8_t  contLecturasImposibles = 0;   // lecturas <2 cm seguidas
uint32_t t_ultimo_eco_valido    = 0;   // millis() de la ultima lectura valida

// ---------- 7. GPS: objeto TinyGPS++ (valores ya calculados) ----------
TinyGPSPlus gps;
uint32_t t_gps = 0;

// ---------- 8. Diagnostico por Serial (no bloqueante) ----------
uint32_t t_serial = 0;

// ---------- 9. Boton: antirrebote con millis() ----------
// INPUT_PULLUP: sin presionar = HIGH, presionado = LOW.
bool botonCruda    = HIGH;   // ultima lectura sin filtrar
bool botonEstable  = HIGH;   // lectura ya antirrebotada
uint32_t t_antirrebote = 0;
bool botonFlanco   = false;  // true por un ciclo cuando hay flanco de presion

// ============================================================
// Unica funcion que escribe 'estado': asi el reloj del estado nuevo
// siempre parte en cero y queda registrada la transicion.
// ============================================================
void cambiar(Estado e) {
  estado = e;
  t_entrada = millis();
  const char* nombre[] = {"REPOSO", "OBSTACULO_LEJOS", "OBSTACULO_CERCA", "ERROR_MEDICION"};
  Serial.printf("[%lu ms] -> %s\n", millis(), nombre[e]);
}

// ============================================================
// Sensor: mismo driver del paso 1, gateado por millis() (sin delay(),
// salvo los delayMicroseconds() del protocolo electrico del HC-SR04).
// Ahora tambien alimenta la deteccion de falla: cada lectura valida
// reinicia el reloj de "sin eco" y corta la racha de imposibles; una
// lectura imposible (<2 cm) suma a esa racha; un "sin eco" no toca la
// racha de imposibles (es un tipo de falla distinto, ver 2.1).
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
    // Sin eco dentro de los ~2 m de alcance util: puede ser "nada
    // delante" (normal) o una desconexion/perdida total de señal. No
    // se decide aqui: huboFallaSensor() se fija en cuanto tiempo lleva
    // pasando esto (ver 2.1) en vez de asumir siempre "camino libre".
    distancia_cm = SIN_OBSTACULO_CM;
  } else {
    float d_bruta = (t_us * VELOCIDAD_CM_US) / 2.0;
    float d = (d_bruta - CAL_B) / CAL_M;   // corrige con la calibracion propia
    if (d >= DIST_MIN_PLAUSIBLE_CM) {
      distancia_cm = d;
      t_ultimo_eco_valido = millis();      // lectura valida: refresca el reloj
      contLecturasImposibles = 0;          // y corta la racha de ruido
    } else {
      // Eco recibido pero a una distancia fisicamente imposible: se
      // descarta como ruido (se mantiene la ultima distancia valida)
      // y esta racha SI cuenta para la deteccion de falla.
      if (contLecturasImposibles < 250) contLecturasImposibles++;
    }
  }
}

// ============================================================
// Falla del sensor (prioridad 1, seccion 5): dos condiciones
// independientes, ver 2.1. Se consulta desde los tres estados
// normales de la FSM antes de evaluar sus propias transiciones.
// ============================================================
bool huboFallaSensor() {
  bool sinEcoProlongado   = (millis() - t_ultimo_eco_valido) > TIMEOUT_SIN_ECO_MS;
  bool lecturasImposibles = (contLecturasImposibles >= LECTURAS_IMPOSIBLES_MAX);
  return sinEcoProlongado || lecturasImposibles;
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
// Actuador: patron de ERROR_MEDICION (prioridad 2, seccion 5). LED y
// buzzer juntos, con una cadencia (3 pulsos cortos + pausa) distinta
// de los avisos de OBSTACULO_LEJOS/CERCA para que la falla no se
// confunda con un aviso de distancia. Se calcula a partir de
// t_entrada -- el reloj que cambiar() ya deja registrado al entrar al
// estado -- y solo actua sobre el LED/buzzer cuando el patron cambia
// de fase (mismo estilo "actuar solo en el cambio" que buzzerPulso()).
// ============================================================
void errorPulso() {
  const uint32_t duracionPulsos = N_ERROR_PULSOS * 2 * T_ERROR_PULSO_MS;
  const uint32_t ciclo          = duracionPulsos + T_ERROR_PAUSA_MS;
  uint32_t fase = (millis() - t_entrada) % ciclo;
  bool encendido = (fase < duracionPulsos) && (((fase / T_ERROR_PULSO_MS) % 2) == 0);

  static bool anterior = false;
  if (encendido != anterior) {
    anterior = encendido;
    digitalWrite(PIN_LED, encendido ? HIGH : LOW);
    if (encendido) tone(PIN_BUZZER, FREQ_BUZZER_ERROR);
    else           buzzerApagado();
  }
}

// ============================================================
// Boton de rearme: antirrebote clasico con millis() (sin delay()).
// botonEstable se usa para el "espejo" del LED fuera de error (paso 5
// de la secuencia de demostracion); botonFlanco (un solo ciclo por
// presion ya antirrebotada) se usa para el rearme deliberado en
// ERROR_MEDICION (paso 6).
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
    if (botonEstable == LOW) botonFlanco = true;   // flanco de presion
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
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BOTON, INPUT_PULLUP);

  buzzerApagado();
  vibrador(false);
  digitalWrite(PIN_LED, LOW);

  t_ultimo_eco_valido = millis();   // da un margen inicial antes de exigir eco

  Serial.println("P26 paso 4 - FSM + error + boton/LED + motor + GPS");
  cambiar(REPOSO);
}

// ============================================================
// LOOP: leerDistancia(), leerGPS() y actualizarBoton() cada vuelta
// (todas se autolimitan/drenan/antirrebotan sin bloquear), luego el
// switch/case recorre el grafo. Sin delay() en ningun punto del lazo
// ni de las funciones que llama.
// ============================================================
void loop() {
  leerDistancia();
  leerGPS();
  actualizarBoton();

  switch (estado) {

    case REPOSO:
      if (huboFallaSensor()) { cambiar(ERROR_MEDICION); break; }
      buzzerApagado();
      vibrador(false);
      if (distancia_cm < D_LEJOS_ON_CM) cambiar(OBSTACULO_LEJOS);
      break;

    case OBSTACULO_LEJOS:
      if (huboFallaSensor()) { cambiar(ERROR_MEDICION); break; }
      buzzerPulso(T_BUZZER_LEJOS_MS, FREQ_BUZZER_LEJOS);   // aviso suave
      vibradorPulso(T_VIBRA_LEJOS_MS);                     // pulso suave
      if (distancia_cm < D_CERCA_ON_CM)       cambiar(OBSTACULO_CERCA);
      else if (distancia_cm > D_LEJOS_OFF_CM) cambiar(REPOSO);
      break;

    case OBSTACULO_CERCA:
      if (huboFallaSensor()) { cambiar(ERROR_MEDICION); break; }
      buzzerPulso(T_BUZZER_CERCA_MS, FREQ_BUZZER_CERCA);   // aviso urgente
      vibrador(true);                                      // continuo
      if (distancia_cm > D_CERCA_OFF_CM) cambiar(OBSTACULO_LEJOS);
      break;

    case ERROR_MEDICION:
      vibrador(false);       // se apaga para no confundirlo con un aviso real
      errorPulso();          // salida local inequivoca (prioridad 2, seccion 5)
      // Rearme deliberado (paso 6): solo si el sensor YA volvio a dar
      // lecturas normales y, ademas, el usuario presiona el boton.
      if (!huboFallaSensor() && botonFlanco) {
        cambiar(REPOSO);
      }
      break;
  }

  // LED fuera de error: espejo inmediato del boton, evidencia de que
  // el lazo no se bloquea aunque el buzzer mantenga su pulso (paso 5).
  if (estado != ERROR_MEDICION) {
    digitalWrite(PIN_LED, (botonEstable == LOW) ? HIGH : LOW);
  }

  // Diagnostico en vivo, no bloqueante: ahora tambien muestra el
  // estado de las dos rachas de falla (util para demostrar el paso 6).
  if (millis() - t_serial >= T_SERIAL_MS) {
    t_serial = millis();
    Serial.printf("d=%.1f cm  estado=%u  imposibles=%u  sinEco=%lu ms\n",
                  distancia_cm, (unsigned)estado, contLecturasImposibles,
                  millis() - t_ultimo_eco_valido);
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

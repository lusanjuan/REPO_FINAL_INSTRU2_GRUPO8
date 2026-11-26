// =============================================================================
// FATIGOMETRO_ESP32.ino — ESP32-C3 SuperMini
// =============================================================================
//
// Lee el EMG del bíceps, se lo pasa al analizador (fatiga_dsp.h), muestra el
// resultado en la pantalla OLED y se lo manda a la app por Bluetooth (BLE).
// Todo el procesamiento se hace acá; la app solo manda START/STOP y recibe un JSON.
//
// Interrupciones (pedido de la cátedra):
//   1. Timer de hardware a 2000 Hz -> isrTimer() despierta a la tarea que lee el ADC.
//   2. Botón de START, flanco descendente -> isrBoton() pide iniciar una serie.
//   (El BLE, además, avisa conexión y comandos por callbacks.)
//
// BLE (dispositivo "MyoSignals", servicio 8d1f0001-...):
//   Comando  8d1f0004-...  WRITE   la app escribe 'N' = NUEVA SESIÓN (serie 1: calibra la referencia)
//                                                 'I' = START (serie siguiente: misma referencia)
//                                                 'S' = STOP  (terminar serie)
//   Fatiga   8d1f0003-...  NOTIFY  un JSON cada 0,5 s y en cada repetición:
//     {"v":2,"seq":41,"t":612034,"serie":1,"reps":7,"estado":"leve","fatiga":8.1,
//      "mdf":59.1,"ref":64.1,"act":58.9,"rms":80,"cal":3,"rep":1,"sat":0}
//     estado: esperando | relaja | calibrando | sin | leve | moderada | alta | sinref
//     fatiga: caída de la MDF respecto de la referencia [%] (0 si todavía no hay referencia)
//   Diagnóstico 8d1f0002-... NOTIFY  TEMPORAL, solo si ENVIAR_DIAGNOSTICO: cada 50 ms la señal
//                                   procesada y el estado del detector (para graficar_esp.py).
//                                   Formato en PaqueteDiagnostico. La app no lo usa.
//
// Buzzer: dos bips cuando la fatiga llega a ALTA (una vez por serie).
//
// Requiere: core ESP32 de Arduino >= 3.0 y las librerías "Adafruit SSD1306" y "Adafruit GFX".

#include "fatiga_dsp.h"   // primero: sus nombres no tienen que chocar con macros de Arduino
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

// =============================================================================
// CONFIGURACIÓN
// =============================================================================

// ---- Pines ----
constexpr int PIN_EMG   = 0;    // salida de la etapa analógica -> GPIO0 (ADC1_CH0)
constexpr int PIN_LED   = 3;    // prendido = conectado por BLE
constexpr int PIN_BOTON = 20;   // botón de START (activo en bajo)
constexpr int PIN_SDA   = 8;    // pantalla OLED (I2C)
constexpr int PIN_SCL   = 9;    // pantalla OLED (I2C)
constexpr int PIN_BUZZER = 7;

// ---- Muestreo ----
constexpr int FS_HZ          = 2000;   // tiene que coincidir con fatiga::FS
constexpr int MUESTRAS_BLOQUE = 100;   // la tarea entrega bloques de 50 ms al loop()
constexpr int BLOQUES_BUFFER  = 16;    // margen de 800 ms si el loop() se atrasa
constexpr adc_attenuation_t ATENUACION_ADC = ADC_6db;   // rango ~0-1750 mV (señal centrada en ~0,66 V)

// ---- Botón ----
constexpr uint32_t ANTIRREBOTE_MS = 300;

// ---- Buzzer ----
constexpr bool     BUZZER_PASIVO   = false;  // true si es pasivo (necesita un tono para sonar)
constexpr int      BUZZER_FREC_HZ  = 2000;   // tono, solo para buzzer pasivo
constexpr int      BUZZER_BIPS     = 2;      // bips al llegar a fatiga ALTA
constexpr uint32_t BUZZER_BIP_MS   = 150;    // duración de cada bip
constexpr uint32_t BUZZER_PAUSA_MS = 150;    // silencio entre bips

// ---- Pantalla ----
constexpr uint8_t  OLED_DIRECCION   = 0x3C;   // algunas pantallas usan 0x3D
constexpr int      OLED_ANCHO       = 128;
constexpr int      OLED_ALTO        = 64;
constexpr uint32_t OLED_PERIODO_MS  = 200;    // como máximo 5 refrescos por segundo (~25 ms cada uno)
constexpr float    OLED_CAIDA_LLENA = 25.0f;  // la barra se llena con 25 % de caída
constexpr int      OLED_HIST_MDF    = 64;     // repeticiones en el gráfico de MDF

// ---- Bluetooth ----
#define BLE_NOMBRE    "MyoSignals"
#define UUID_SERVICIO "8d1f0001-6c3a-4b8e-9a2e-3f1b2c4d5e60"
#define UUID_DIAGNOSTICO "8d1f0002-6c3a-4b8e-9a2e-3f1b2c4d5e60"
#define UUID_FATIGA   "8d1f0003-6c3a-4b8e-9a2e-3f1b2c4d5e60"
#define UUID_COMANDO  "8d1f0004-6c3a-4b8e-9a2e-3f1b2c4d5e60"
constexpr uint16_t BLE_MTU             = 247;   // el JSON (~150 bytes) entra en una notificación
constexpr uint32_t BLE_REANUNCIAR_MS   = 500;   // espera antes de volver a anunciarse
constexpr bool     ENVIAR_DIAGNOSTICO  = true;  // TEMPORAL: señal procesada para graficar_esp.py
                                                // (false cuando se use solo con la app)
constexpr float    DIAG_ESCALA_MUESTRA = 10.0f; // la señal va en décimas de mV (int16: ±3276 mV)
constexpr char     CMD_INICIAR         = 'I';
constexpr char     CMD_DETENER         = 'S';
constexpr char     CMD_NUEVA_SESION    = 'N';   // nueva sesión: olvida la referencia y arranca la serie 1
constexpr int      JSON_MAX            = 200;

static_assert(FS_HZ == (int)fatiga::FS, "FS_HZ tiene que ser igual a fatiga::FS");
static_assert(MUESTRAS_BLOQUE == fatiga::ENV_PASO, "un bloque tiene que ser un punto de la envolvente");
static_assert(PIN_BUZZER != PIN_SDA && PIN_BUZZER != PIN_SCL && PIN_BUZZER != PIN_BOTON &&
              PIN_SDA != PIN_BOTON && PIN_SCL != PIN_BOTON,
              "dos cosas comparten pin: revisar PIN_SDA, PIN_SCL, PIN_BUZZER y PIN_BOTON");

// Paquete de diagnóstico (TEMPORAL), little-endian, 234 bytes. En Python: "<IfffffffBB100h".
// Todo corresponde al final del bloque de 50 ms.
struct __attribute__((packed)) PaqueteDiagnostico {
  uint32_t bloque;          // número de bloque: la muestra i del bloque es la n = bloque*100 + i
  float    envolvente;      // RMS de los últimos 0,5 s [mV]
  float    limiteRecorte;   // ± límite del recorte de picos [mV] (0 = todavía sin recorte)
  float    reposo;          // nivel de reposo [mV] (0 = todavía no se midió)
  float    maximo;          // máximo de la envolvente desde el último valle [mV]
  float    minimo;          // mínimo desde el último pico [mV]
  float    umbralPico;      // buscando pico: la envolvente tiene que bajar de acá (máximo · (1 − caída))
  float    umbralValle;     // buscando valle: tiene que subir de acá para otra contracción (mínimo · (1 + subida))
  uint8_t  etapa;           // fatiga::Etapa
  uint8_t  buscandoPico;    // 1 = buscando pico, 0 = buscando valle
  int16_t  muestras[MUESTRAS_BLOQUE];   // señal filtrada y recortada, en décimas de mV
};
static_assert(sizeof(PaqueteDiagnostico) == 234, "el paquete de diagnóstico tiene que medir 234 bytes");

// =============================================================================
// ESTADO COMPARTIDO CON LAS INTERRUPCIONES Y CALLBACKS
// (volatile: lo cambian una ISR o el BLE y lo lee loop())
// =============================================================================

enum class Comando : uint8_t { NINGUNO, INICIAR, DETENER, NUEVA_SESION };
static volatile Comando comandoPendiente = Comando::NINGUNO;
static volatile bool    bleConectado     = false;

// Buffer de muestreo: la tarea escribe bloques, loop() los lee
static uint16_t          bufferAdc[BLOQUES_BUFFER][MUESTRAS_BLOQUE];
static volatile uint32_t bloquesEscritos   = 0;
static volatile uint32_t ticksSinLeer      = 0;   // interrupciones que la tarea no llegó a atender
static uint32_t          bloquesLeidos     = 0;
static uint32_t          bloquesPerdidos   = 0;

static hw_timer_t   *timerMuestreo = nullptr;
static TaskHandle_t  tareaMuestreo = nullptr;
static volatile uint32_t ultimoBotonMs = 0;

// =============================================================================
// ESTADO DEL PROGRAMA
// =============================================================================

fatiga::Analizador analizador;   // global: ocupa ~45 KB, no entra en la pila
fatiga::Resultado  ultimo;
bool     hayResultado = false;
uint16_t numeroSerie  = 0;       // series iniciadas desde que se prendió
uint32_t seqPaquete   = 0;

float histMdf[OLED_HIST_MDF];
int   nHistMdf = 0;

Adafruit_SSD1306 pantalla(OLED_ANCHO, OLED_ALTO, &Wire, -1);
bool     pantallaOk = false;
bool     redibujar  = true;
uint32_t ultimoDibujoMs = 0;

BLECharacteristic *caracteristicaDiag   = nullptr;
BLECharacteristic *caracteristicaFatiga = nullptr;

// =============================================================================
// INTERRUPCIONES
// =============================================================================

// Timer cada 500 us. La ISR no lee el ADC (en el core 3.x analogRead usa un mutex y
// no se puede llamar desde una ISR): solo despierta a la tarea de muestreo, que es de
// alta prioridad y lee enseguida. Patrón FreeRTOS: ISR corta + tarea que hace el trabajo.
void ARDUINO_ISR_ATTR isrTimer() {
  BaseType_t despertar = pdFALSE;
  vTaskNotifyGiveFromISR(tareaMuestreo, &despertar);
  portYIELD_FROM_ISR(despertar);
}

// Botón de START: pide iniciar una serie (misma referencia de la sesión). El trabajo se hace en loop(), no acá.
void ARDUINO_ISR_ATTR isrBoton() {
  const uint32_t ahora = millis();
  if (ahora - ultimoBotonMs < ANTIRREBOTE_MS) return;
  ultimoBotonMs = ahora;
  comandoPendiente = Comando::INICIAR;
}

// =============================================================================
// MUESTREO
// =============================================================================

// Tarea de alta prioridad: espera cada aviso del timer y lee una muestra
void tareaLeerAdc(void *) {
  uint16_t posicion = 0;
  for (;;) {
    const uint32_t avisos = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (avisos > 1) ticksSinLeer += avisos - 1;            // no debería pasar
    bufferAdc[bloquesEscritos % BLOQUES_BUFFER][posicion] = analogReadMilliVolts(PIN_EMG);
    if (++posicion >= MUESTRAS_BLOQUE) {
      posicion = 0;
      bloquesEscritos++;                                     // bloque completo para loop()
    }
  }
}

// Copia el próximo bloque completo. Devuelve false si no hay ninguno nuevo.
bool tomarBloque(uint16_t *bloque, uint32_t &numero) {
  if (bloquesLeidos >= bloquesEscritos) return false;
  if (bloquesEscritos - bloquesLeidos > BLOQUES_BUFFER - 1) {   // loop() se atrasó: se saltean bloques
    bloquesPerdidos += bloquesEscritos - (BLOQUES_BUFFER - 1) - bloquesLeidos;
    bloquesLeidos = bloquesEscritos - (BLOQUES_BUFFER - 1);
    Serial.printf("Atraso: %lu bloques perdidos en total\n", (unsigned long)bloquesPerdidos);
  }
  memcpy(bloque, bufferAdc[bloquesLeidos % BLOQUES_BUFFER], sizeof(uint16_t) * MUESTRAS_BLOQUE);
  numero = bloquesLeidos;
  bloquesLeidos++;
  return true;
}

// =============================================================================
// BLUETOOTH
// =============================================================================

class ServidorCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override    { bleConectado = true; }
  void onDisconnect(BLEServer *) override { bleConectado = false; }
};

// La app escribe un comando de un carácter
class ComandoCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    if (c->getLength() == 0) return;
    const char cmd = (char)toupper(c->getData()[0]);
    if (cmd == CMD_INICIAR)      comandoPendiente = Comando::INICIAR;
    else if (cmd == CMD_DETENER) comandoPendiente = Comando::DETENER;
    else if (cmd == CMD_NUEVA_SESION) comandoPendiente = Comando::NUEVA_SESION;
  }
};

// Texto del campo "estado" del JSON (el que la app muestra)
const char *estadoParaApp(const fatiga::Resultado &r) {
  using fatiga::Etapa;
  using fatiga::Nivel;
  switch (r.etapa) {
    case Etapa::RELAJA:         return "relaja";
    case Etapa::CALIBRANDO:     return "calibrando";
    case Etapa::SIN_REFERENCIA: return "sinref";
    case Etapa::MIDIENDO:
      switch (r.nivel) {
        case Nivel::LEVE:     return "leve";
        case Nivel::MODERADA: return "moderada";
        case Nivel::ALTA:     return "alta";
        default:              return "sin";
      }
    default:                    return "esperando";
  }
}

// Arma el JSON del resultado. Devuelve el largo (0 si no entró).
int armarJson(const fatiga::Resultado &r, char *json, size_t tamano) {
  const bool midiendo = r.etapa == fatiga::Etapa::MIDIENDO;
  const int n = snprintf(json, tamano,
      "{\"v\":2,\"seq\":%lu,\"t\":%lu,\"serie\":%u,\"reps\":%u,\"estado\":\"%s\",\"fatiga\":%.1f,"
      "\"mdf\":%.1f,\"ref\":%.1f,\"act\":%.1f,\"rms\":%.0f,\"cal\":%u,\"rep\":%d,\"sat\":%d}",
      (unsigned long)seqPaquete++, (unsigned long)(r.t_s * 1000.0f), numeroSerie, r.reps,
      estadoParaApp(r), midiendo ? r.caida_pct : 0.0f,
      r.mdf, r.mdf_ref, r.mdf_actual, r.rms, r.repsCalibracion, r.repNueva ? 1 : 0, r.saturada ? 1 : 0);
  return (n > 0 && n < (int)tamano) ? n : 0;
}

void enviarResultado(const fatiga::Resultado &r) {
  char json[JSON_MAX];
  const int largo = armarJson(r, json, sizeof(json));
  if (largo == 0) return;
  caracteristicaFatiga->setValue((uint8_t *)json, largo);
  if (bleConectado) caracteristicaFatiga->notify();
}

// TEMPORAL: la señal procesada del bloque y el estado del detector, para graficar_esp.py
void enviarDiagnostico(uint32_t numeroBloque, const int16_t *muestras) {
  if (!ENVIAR_DIAGNOSTICO || !bleConectado) return;
  static PaqueteDiagnostico p;
  p.bloque        = numeroBloque;
  p.envolvente    = analizador.envolvente();
  p.limiteRecorte = analizador.limiteRecorte();
  p.reposo        = analizador.reposo();
  p.maximo        = analizador.maximo();
  p.minimo        = analizador.minimo();
  p.umbralPico    = analizador.maximo() * (1 - fatiga::REP_CAIDA_PICO);
  p.umbralValle   = analizador.minimo() * (1 + fatiga::REP_SUBIDA_VALLE);
  p.etapa         = (uint8_t)analizador.etapa();
  p.buscandoPico  = analizador.buscandoPico() ? 1 : 0;
  memcpy(p.muestras, muestras, sizeof(p.muestras));
  caracteristicaDiag->setValue((uint8_t *)&p, sizeof(p));
  caracteristicaDiag->notify();
}

// Muestra procesada [mV] -> décimas de mV en int16 (con saturación)
int16_t aDecimasDeMv(float mV) {
  const float v = mV * DIAG_ESCALA_MUESTRA;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

// =============================================================================
// BUZZER (no bloquea: actualizarBuzzer() prende y apaga mirando el reloj)
// =============================================================================

int      bipsPendientes = 0;
bool     buzzerSonando  = false;
uint32_t buzzerCambioMs = 0;
bool     avisoAltaDado  = false;   // el aviso suena una sola vez por serie

void buzzer(bool prender) {
  if (BUZZER_PASIVO) {
    if (prender) tone(PIN_BUZZER, BUZZER_FREC_HZ); else noTone(PIN_BUZZER);
  } else {
    digitalWrite(PIN_BUZZER, prender ? HIGH : LOW);
  }
}

void pedirBips(int cantidad) {
  bipsPendientes = cantidad;
  buzzerSonando = false;
  buzzerCambioMs = millis() - BUZZER_PAUSA_MS;   // el primer bip arranca enseguida
}

void actualizarBuzzer() {
  if (bipsPendientes == 0) return;               // nada que sonar: queda apagado
  const uint32_t ahora = millis();
  if (buzzerSonando) {
    if (ahora - buzzerCambioMs < BUZZER_BIP_MS) return;
    buzzer(false);
    buzzerSonando = false;
    bipsPendientes--;
  } else {
    if (ahora - buzzerCambioMs < BUZZER_PAUSA_MS) return;
    buzzer(true);
    buzzerSonando = true;
  }
  buzzerCambioMs = ahora;
}

void avisarSiFatigaAlta(const fatiga::Resultado &r) {
  if (avisoAltaDado) return;
  if (r.etapa == fatiga::Etapa::MIDIENDO && r.nivel == fatiga::Nivel::ALTA) {
    avisoAltaDado = true;
    pedirBips(BUZZER_BIPS);
  }
}

// =============================================================================
// PANTALLA
// =============================================================================

void guardarMdfParaGrafico(float mdf) {
  if (nHistMdf == OLED_HIST_MDF) {
    memmove(histMdf, histMdf + 1, sizeof(float) * (OLED_HIST_MDF - 1));
    nHistMdf--;
  }
  histMdf[nHistMdf++] = mdf;
}

void dibujarEstado() {
  using fatiga::Etapa;
  const Etapa etapa = hayResultado ? ultimo.etapa : Etapa::ESPERANDO;
  pantalla.setCursor(0, 0);
  switch (etapa) {
    case Etapa::ESPERANDO:      pantalla.print("Esperando START"); break;
    case Etapa::RELAJA:         pantalla.print("Relaja el brazo..."); break;
    case Etapa::SIN_REFERENCIA: pantalla.print("Sin referencia"); break;
    case Etapa::CALIBRANDO:
      pantalla.printf("Calibrando %d/%d", min((int)ultimo.repsCalibracion, fatiga::REF_REPS), fatiga::REF_REPS);
      break;
    case Etapa::MIDIENDO:
      pantalla.print("FATIGA: ");
      pantalla.print(ultimo.nivel == fatiga::Nivel::SIN_FATIGA ? "NO" : fatiga::nombreNivel(ultimo.nivel));
      break;
  }
  if (hayResultado && ultimo.contraido) { pantalla.setCursor(98, 0); pantalla.print("*"); }
  if (bleConectado) { pantalla.setCursor(110, 0); pantalla.print("BT"); }
}

void dibujarNumeros() {
  pantalla.setTextSize(2);                                 // repeticiones, grande
  pantalla.setCursor(0, 11);
  pantalla.print(ultimo.reps);
  pantalla.setTextSize(1);
  pantalla.setCursor(pantalla.getCursorX() + 2, 18);
  pantalla.print("reps");

  pantalla.setCursor(70, 10);
  if (ultimo.mdf_ref > 0) {
    pantalla.printf("ref %.1f", ultimo.mdf_ref);
    pantalla.setCursor(70, 20);
    pantalla.printf("%.1f%%", -ultimo.caida_pct);
  } else {
    pantalla.printf("mdf %.1f", ultimo.mdf);
  }

  pantalla.drawRect(0, 29, OLED_ANCHO, 6, SSD1306_WHITE);  // barra de caída
  const float llenado = constrain(ultimo.caida_pct / OLED_CAIDA_LLENA, 0.0f, 1.0f);
  pantalla.fillRect(1, 30, (int)(llenado * (OLED_ANCHO - 2)), 4, SSD1306_WHITE);

  pantalla.setCursor(0, 38);
  pantalla.printf("MDF %.1f Hz", ultimo.mdf_ref > 0 ? ultimo.mdf_actual : ultimo.mdf);
}

// MDF de cada repetición (línea) y referencia (punteada)
void dibujarGraficoMdf(int y0, int alto) {
  if (nHistMdf < 2) return;
  float lo = histMdf[0], hi = histMdf[0];
  for (int i = 0; i < nHistMdf; i++) { lo = fminf(lo, histMdf[i]); hi = fmaxf(hi, histMdf[i]); }
  if (ultimo.mdf_ref > 0) { lo = fminf(lo, ultimo.mdf_ref); hi = fmaxf(hi, ultimo.mdf_ref); }
  if (hi - lo < 10) { const float m = (hi + lo) / 2; lo = m - 5; hi = m + 5; }
  auto yDe = [&](float v) { return y0 + alto - 1 - (int)((v - lo) / (hi - lo) * (alto - 1)); };

  if (ultimo.mdf_ref > 0) {
    const int yRef = yDe(ultimo.mdf_ref);
    for (int x = 0; x < OLED_ANCHO; x += 4) pantalla.drawPixel(x, yRef, SSD1306_WHITE);
  }
  for (int i = 1; i < nHistMdf; i++)
    pantalla.drawLine((i - 1) * 2, yDe(histMdf[i - 1]), i * 2, yDe(histMdf[i]), SSD1306_WHITE);
}

void dibujarPantalla() {
  if (!pantallaOk) return;
  pantalla.clearDisplay();
  pantalla.setTextColor(SSD1306_WHITE);
  pantalla.setTextSize(1);
  dibujarEstado();
  if (hayResultado && ultimo.reps > 0) {
    dibujarNumeros();
    dibujarGraficoMdf(47, 17);
  }
  pantalla.display();
}

// =============================================================================
// SERIAL (para depurar)
// =============================================================================

void imprimirEvento(const fatiga::Resultado &r) {
  if (!r.evento) return;
  const char *etapa = r.etapa == fatiga::Etapa::MIDIENDO ? fatiga::nombreNivel(r.nivel) : fatiga::nombreEtapa(r.etapa);
  Serial.printf("t=%.1f [%s] %s", r.t_s, etapa, r.evento);
  if (r.repNueva) Serial.printf(" | rep %u  mdf=%.1f  rms=%.0f%s", r.reps, r.mdf, r.rms, r.saturada ? " (saturo)" : "");
  if (r.mdf_ref > 0)
    Serial.printf(" | ref=%.1f  actual=%.1f  caida=%.1f%%  pend=%+.2f Hz/rep",
                  r.mdf_ref, r.mdf_actual, r.caida_pct, r.pendiente);
  Serial.println();
}

// =============================================================================
// ACCIONES
// =============================================================================

// Sesión nueva (app: "Iniciar entrenamiento"): la ESP olvida la referencia y arranca la serie 1
void nuevaSesion() {
  analizador.nuevaSesion();
  numeroSerie++;
  avisoAltaDado = false;
  nHistMdf = 0;
  redibujar = true;
}

// Serie siguiente (app: "Iniciar serie N", o el botón): mantiene la referencia de la sesión
void iniciarSerie() {
  analizador.iniciarSerie();
  numeroSerie++;
  avisoAltaDado = false;
  nHistMdf = 0;
  redibujar = true;
}

void detenerSerie() {
  analizador.detenerSerie();
  redibujar = true;
}

void procesarResultado(const fatiga::Resultado &r) {
  ultimo = r;
  hayResultado = true;
  if (r.repNueva) guardarMdfParaGrafico(r.mdf);
  avisarSiFatigaAlta(r);
  enviarResultado(r);
  imprimirEvento(r);
  redibujar = true;
}

// =============================================================================
// SETUP
// =============================================================================

void configurarAdc() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_EMG, ATENUACION_ADC);
}

void configurarPantalla() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  pantallaOk = pantalla.begin(SSD1306_SWITCHCAPVCC, OLED_DIRECCION);
  if (!pantallaOk) Serial.println("No se encontro la pantalla OLED; sigo sin pantalla");
  dibujarPantalla();
}

void configurarBluetooth() {
  BLEDevice::init(BLE_NOMBRE);
  BLEDevice::setMTU(BLE_MTU);
  BLEServer *servidor = BLEDevice::createServer();
  servidor->setCallbacks(new ServidorCallbacks());
  BLEService *servicio = servidor->createService(UUID_SERVICIO);

  caracteristicaDiag = servicio->createCharacteristic(UUID_DIAGNOSTICO, BLECharacteristic::PROPERTY_NOTIFY);
  caracteristicaDiag->addDescriptor(new BLE2902());

  caracteristicaFatiga = servicio->createCharacteristic(
      UUID_FATIGA, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  caracteristicaFatiga->addDescriptor(new BLE2902());

  BLECharacteristic *comando = servicio->createCharacteristic(
      UUID_COMANDO, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  comando->setCallbacks(new ComandoCallbacks());

  servicio->start();
  BLEDevice::getAdvertising()->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Anunciando como '" BLE_NOMBRE "'");
}

void configurarBuzzer() {
  pinMode(PIN_BUZZER, OUTPUT);
  buzzer(false);
}

void configurarBoton() {
  pinMode(PIN_BOTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BOTON), isrBoton, FALLING);
}

void configurarMuestreo() {
  // Primero la tarea (la ISR la necesita), con prioridad por encima del BLE y de loop()
  xTaskCreate(tareaLeerAdc, "adc", 4096, nullptr, configMAX_PRIORITIES - 3, &tareaMuestreo);
  // Timer de hardware: cuenta a 1 MHz y dispara cada 1e6 / FS_HZ cuentas, con autorrecarga
  timerMuestreo = timerBegin(1000000);
  timerAttachInterrupt(timerMuestreo, &isrTimer);
  timerAlarm(timerMuestreo, 1000000 / FS_HZ, true, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  configurarAdc();
  configurarPantalla();
  configurarBluetooth();
  configurarBuzzer();
  configurarBoton();
  configurarMuestreo();

  Serial.println("FATIGOMETRO listo: START desde la app (o el boton)");
}

// =============================================================================
// LOOP
// =============================================================================

void atenderConexion() {
  static bool estabaConectado = false;
  if (bleConectado == estabaConectado) return;
  estabaConectado = bleConectado;
  digitalWrite(PIN_LED, bleConectado ? HIGH : LOW);
  redibujar = true;
  if (bleConectado) {
    Serial.println("Conectado");
  } else {
    Serial.println("Desconectado, vuelvo a anunciar");
    delay(BLE_REANUNCIAR_MS);
    BLEDevice::startAdvertising();
  }
}

void atenderComandos() {
  const Comando c = comandoPendiente;   // lo escribió la ISR del botón o el BLE
  comandoPendiente = Comando::NINGUNO;
  if (c == Comando::INICIAR) iniciarSerie();
  else if (c == Comando::NUEVA_SESION) nuevaSesion();
  else if (c == Comando::DETENER) detenerSerie();
}

void procesarMuestras() {
  uint16_t bloque[MUESTRAS_BLOQUE];
  int16_t procesadas[MUESTRAS_BLOQUE];
  uint32_t numero;
  fatiga::Resultado r;
  while (tomarBloque(bloque, numero)) {
    for (int i = 0; i < MUESTRAS_BLOQUE; i++) {
      if (analizador.agregar((float)bloque[i], r)) procesarResultado(r);
      procesadas[i] = aDecimasDeMv(analizador.muestraProcesada());
    }
    enviarDiagnostico(numero, procesadas);   // TEMPORAL
  }
}

void avisarAtrasos() {
  static uint32_t informados = 0;
  if (ticksSinLeer == informados) return;
  informados = ticksSinLeer;
  Serial.printf("Aviso: %lu interrupciones del timer sin leer\n", (unsigned long)informados);
}

void actualizarPantalla() {
  if (!redibujar || millis() - ultimoDibujoMs < OLED_PERIODO_MS) return;
  ultimoDibujoMs = millis();
  redibujar = false;
  dibujarPantalla();
}

void loop() {
  atenderConexion();
  atenderComandos();
  procesarMuestras();
  avisarAtrasos();
  actualizarPantalla();
  actualizarBuzzer();
  delay(1);
}
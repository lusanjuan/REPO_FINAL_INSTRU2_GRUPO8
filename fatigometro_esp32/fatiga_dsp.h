// =============================================================================
// fatiga_dsp.h — Procesamiento del EMG: cuenta repeticiones y estima la fatiga
//                muscular a partir de la frecuencia mediana (MDF) de cada una.
// =============================================================================
//
// No usa nada de Arduino: el mismo archivo compila en la ESP32 y en la PC
// (así se prueba el algoritmo con señales grabadas o simuladas).
//
// Uso desde el .ino:
//     fatiga::Analizador analizador;
//     analizador.nuevaSesion();                  // NUEVA SESIÓN (app): olvida la referencia y arranca la serie 1
//     analizador.iniciarSerie();                 // START (app) o botón: serie siguiente, misma referencia
//     fatiga::Resultado r;
//     if (analizador.agregar(muestra_mV, r)) {   // 2000 veces por segundo
//       ... r tiene un resultado nuevo: mostrarlo / mandarlo ...
//     }
//     analizador.detenerSerie();                 // STOP (app)
//
// Etapas de una serie:
//   ESPERANDO       No hay serie en curso (al prender, o después de STOP).
//   RELAJA          Espera REPOSO_DURACION_S seguidos con el brazo quieto y mide
//                   el nivel de reposo. Lo anterior (acomodarse) se ignora.
//   CALIBRANDO      Cuenta repeticiones. Después de REF_REPS_OBSERVAR repeticiones,
//                   la referencia es la mediana de la MDF de las primeras REF_REPS
//                   repeticiones VÁLIDAS (ver "Repetición válida").
//   MIDIENDO        Cada repetición se compara con la referencia y da un nivel de
//                   fatiga: SIN_FATIGA, LEVE, MODERADA o ALTA.
//
//   Referencia de la sesión (REF_MANTENER_ENTRE_SERIES = true):
//     La referencia se calibra en la primera serie y se mantiene en las
//     siguientes: la serie 2 en adelante va de RELAJA directo a MIDIENDO y su
//     fatiga se mide contra el músculo descansado de la serie 1. Se olvida con
//     nuevaSesion() (o al apagar). Si la serie 1 no pudo calibrar, la siguiente
//     vuelve a calibrar.
//   SIN_REFERENCIA  No logró calibrar en REF_INTENTOS_MAX repeticiones. Las
//                   repeticiones se siguen contando.
//
// Procesamiento de cada muestra (igual al análisis en Python del grupo):
//   1. Filtro pasabanda 20-450 Hz (Butterworth orden 4) + notch 50 Hz.
//   2. Recorte de picos (ECG, golpes de cable): cada muestra se limita a
//      ±RECORTE_K veces la escala de la señal (mediana de la RMS de los últimos
//      10 bloques de 50 ms). Un pico ocupa 1 bloque y no mueve la mediana.
//   3. Envolvente: RMS de los últimos 0,5 s, un punto cada 50 ms.
//   4. Repetición = pico de la envolvente: después del máximo cae REP_CAIDA_PICO,
//      antes subió REP_SUBIDA_VALLE desde el valle, supera REP_FACTOR_REPOSO veces
//      el reposo y está a REP_SEPARACION_MIN_S de la anterior.
//   5. MDF, MNF y RMS de la repetición: espectro de Welch (tramos de 512, Hann,
//      50 % de solape) en una ventana de 1,5 s centrada en el pico.
//   6. Fatiga (método "M2 + MDF o MNF" de metodos.py, el que mejor dio en las pruebas):
//      - Recta de Theil-Sen de la MDF (y otra de la MNF) vs número de repetición
//        válida, sobre toda la serie. Su valor en la última rep es la "actual".
//      - Referencia = valor de la recta en la primera rep válida. En las series
//        siguientes se usa la de la serie 1, congelada.
//      - caída % = 100 · (referencia − actual) / referencia, para MDF y MNF.
//      - La MNF cae menos: se divide por MNF_FACTOR_UMBRALES para usar los mismos
//        umbrales. Con FATIGA_SENAL = MDF_O_MNF manda la mayor de las dos.
//      El nivel sube si la caída supera el umbral FATIGA_CONFIRMAR_REPS
//      repeticiones seguidas, y no baja dentro de la serie.
//      (FATIGA_RECTA_ROBUSTA, FATIGA_REF_DE_LA_RECTA y FATIGA_SENAL permiten volver
//       al método anterior: false, false y MDF.)
//
// Repetición válida (igual que los scripts de análisis del grupo):
//   TODAS las repeticiones se cuentan. Pero solo las válidas entran en la
//   referencia y en la recta de fatiga. Una repetición NO es válida si:
//     - su RMS es menor que VALIDA_RMS_MIN veces la mediana de las RMS de la
//       serie (acomodar el brazo, una repetición a medias, la última al soltar), o
//     - saturó el ADC (VALIDA_SIN_SATURAR).

#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <algorithm>

namespace fatiga {

// =============================================================================
// CONFIGURACIÓN — todo lo que se puede ajustar está acá
// =============================================================================

// ---- Señal ----
constexpr float FS                 = 2000.0f;  // frecuencia de muestreo [Hz] (igual que el .ino)
constexpr float FILTRO_F_BAJA      = 20.0f;    // pasaaltos [Hz] (20 = scripts del grupo; 30 saca más ECG
                                               //   pero también parte del sEMG, que en algunas personas está en 30-60 Hz)
constexpr float FILTRO_F_ALTA      = 450.0f;   // pasabajos [Hz]
constexpr bool  NOTCH_ACTIVO       = true;     // notch para la interferencia de la red
constexpr float NOTCH_F            = 50.0f;    // [Hz]
constexpr float NOTCH_Q            = 10.0f;    // más alto = notch más angosto
constexpr float FILTRO_ESPERA_S    = 1.0f;     // tiempo para que se asienten los filtros al prender

// ---- Recorte de picos (artefactos) ----
constexpr bool  RECORTE_ACTIVO     = true;
constexpr float RECORTE_K          = 4.0f;     // límite = K · mediana de la RMS por bloque

// ---- Envolvente ----
constexpr int   ENV_PASO           = 100;      // un punto de la envolvente cada 100 muestras (50 ms)
constexpr float ENV_VENTANA_S      = 0.5f;     // la envolvente es la RMS de los últimos 0,5 s
constexpr bool  ENV_CENTRADA       = true;     // ubica el pico en el centro de la ventana (como Python)

// ---- Reposo (etapa RELAJA) ----
constexpr float REPOSO_DURACION_S  = 2.0f;     // segundos seguidos con el brazo quieto
constexpr float REPOSO_MAX_MV      = 30.0f;    // envolvente máxima para considerar "quieto" [mV]

// ---- Detección de repeticiones ----
constexpr float REP_CAIDA_PICO       = 0.30f;  // la envolvente cae 30 % desde el máximo -> hubo pico
constexpr float REP_SUBIDA_VALLE     = 0.30f;  // y antes subió 30 % desde el valle anterior
constexpr float REP_FACTOR_REPOSO    = 2.0f;   // el pico supera 2 veces el nivel de reposo
constexpr float REP_SEPARACION_MIN_S = 2.0f;   // separación mínima entre dos repeticiones [s]

// ---- Espectro de cada repetición ----
constexpr float ESPECTRO_VENTANA_S = 1.5f;     // ventana centrada en el pico [s]
constexpr int   NFFT               = 512;      // largo de cada tramo de Welch (potencia de 2)
constexpr int   LOG2_NFFT          = 9;        // log2(NFFT)
constexpr float ESPECTRO_F_MIN     = FILTRO_F_BAJA;  // banda donde se calculan MDF y MNF [Hz]
constexpr float ESPECTRO_F_MAX     = FILTRO_F_ALTA;

// ---- Saturación del ADC (solo se informa; la repetición se cuenta igual) ----
constexpr float SAT_MIN_MV         = 10.0f;
constexpr float SAT_MAX_MV         = 2500.0f;
constexpr float SAT_FRACCION_MAX   = 0.02f;    // más del 2 % de muestras saturadas -> se marca

// ---- Repetición válida (para la referencia y la fatiga; el conteo no cambia) ----
constexpr float VALIDA_RMS_MIN       = 0.70f;  // RMS >= 70 % de la mediana de la serie
constexpr bool  VALIDA_SIN_SATURAR   = true;   // las repeticiones saturadas no entran

// ---- Referencia (calibración) ----
constexpr int   REF_REPS             = 3;      // repeticiones válidas que forman la referencia
constexpr int   REF_REPS_OBSERVAR    = 5;      // se espera a tener 5 repeticiones para saber cuál es
                                               //   la RMS "normal" y descartar las del arranque
constexpr bool  REF_EXIGIR_PAREJAS   = false;  // false: la referencia sale de las primeras REF_REPS
                                               //        repeticiones, sin revisar ritmo, fuerza ni MDF
constexpr int   REF_INTENTOS_MAX     = 8;      // si no calibra en 8 repeticiones -> SIN_REFERENCIA
constexpr bool  REF_INCLUIR_PRIMERA  = true;   // false: la 1.ª repetición se cuenta pero no calibra
constexpr float REF_TOL_RITMO        = 0.30f;  // intervalos entre picos: ±30 % de su mediana
constexpr float REF_TOL_RMS          = 0.25f;  // RMS de cada repetición: ±25 % de la mediana
constexpr float REF_CV_MDF_MAX       = 0.08f;  // dispersión de las MDF (desvío / promedio)
constexpr bool  REF_MANTENER_ENTRE_SERIES = true;  // la referencia de la serie 1 vale para toda la sesión

// ---- Fatiga ----
constexpr float FATIGA_LEVE_PCT        = 7.0f;   // caída de la MDF respecto de la referencia [%]
constexpr float FATIGA_MODERADA_PCT    = 12.0f;
constexpr float FATIGA_ALTA_PCT        = 20.0f;
constexpr bool  FATIGA_USAR_RECTA      = true;   // MDF actual = recta de toda la serie
constexpr bool  FATIGA_RECTA_ROBUSTA   = true;   // recta de Theil-Sen (mediana de las pendientes entre
                                                 //   pares de reps): una rep rara casi no la mueve.
                                                 //   false = mínimos cuadrados (como antes)
constexpr bool  FATIGA_REF_DE_LA_RECTA = true;   // referencia = valor de la recta en la primera rep válida
                                                 //   (no la mediana de 3 reps sueltas). false = como antes
// Qué se mira para la fatiga:
//   MDF        solo la MDF (como antes)
//   MNF        solo la MNF (con sus umbrales)
//   MDF_Y_MNF  tienen que caer las dos
//   MDF_O_MNF  alcanza con que caiga una de las dos
enum class Senal : uint8_t { MDF, MNF, MDF_Y_MNF, MDF_O_MNF };
constexpr Senal FATIGA_SENAL           = Senal::MDF_O_MNF;
constexpr float MNF_FACTOR_UMBRALES    = 0.7f;   // umbrales de la MNF = 0,7 × los de la MDF (4,9 / 8,4 / 14 %)
constexpr int   FATIGA_REPS_MIN_RECTA  = 4;      // con menos repeticiones se usa la mediana:
constexpr int   FATIGA_REPS_MEDIANA    = 3;      //   mediana de las últimas 3 MDF
constexpr int   FATIGA_CONFIRMAR_REPS  = 2;      // repeticiones seguidas sobre el umbral para subir
constexpr bool  FATIGA_SOLO_SUBE       = true;   // el nivel no baja dentro de una serie

// ---- Salida y memoria ----
constexpr float INFORME_PERIODO_S  = 0.5f;     // además de los eventos, un resultado cada 0,5 s
constexpr float BUFFER_S           = 4.0f;     // señal filtrada guardada (4 s = 32 KB)
constexpr int   MAX_REPS_SERIE     = 100;      // MDF guardadas por serie para la recta

// ---- Derivadas (no hace falta tocarlas) ----
constexpr int BLOQUES_ENV       = (int)(ENV_VENTANA_S * FS) / ENV_PASO;   // 10
constexpr int MUESTRAS_ESPECTRO = (int)(ESPECTRO_VENTANA_S * FS);         // 3000
constexpr int BUF_N             = (int)(BUFFER_S * FS);                   // 8000
constexpr int SOLAPE            = NFFT / 2;                               // 50 %
constexpr int PUNTOS_REPOSO     = (int)(REPOSO_DURACION_S * FS) / ENV_PASO;   // 40
constexpr int MUESTRAS_INFORME  = (int)(INFORME_PERIODO_S * FS);          // 1000
constexpr int BLOQUES_SAT       = BUF_N / ENV_PASO;                       // 80
static_assert((1 << LOG2_NFFT) == NFFT, "NFFT tiene que ser 2^LOG2_NFFT");
static_assert(REF_REPS_OBSERVAR >= REF_REPS, "REF_REPS_OBSERVAR tiene que ser >= REF_REPS");
static_assert(REF_INTENTOS_MAX >= REF_REPS_OBSERVAR, "REF_INTENTOS_MAX tiene que ser >= REF_REPS_OBSERVAR");
static_assert(REF_REPS <= 16, "REF_REPS: la mediana corta admite hasta 16 valores");
static_assert(MUESTRAS_ESPECTRO + ENV_PASO * BLOQUES_ENV < BUF_N, "el buffer no alcanza para la ventana");

// =============================================================================
// TIPOS
// =============================================================================

enum class Etapa : uint8_t { ESPERANDO, RELAJA, CALIBRANDO, MIDIENDO, SIN_REFERENCIA };
enum class Nivel : uint8_t { SIN_FATIGA, LEVE, MODERADA, ALTA };

inline const char *nombreEtapa(Etapa e) {
  switch (e) {
    case Etapa::RELAJA:         return "RELAJA";
    case Etapa::CALIBRANDO:     return "CALIBRANDO";
    case Etapa::MIDIENDO:       return "MIDIENDO";
    case Etapa::SIN_REFERENCIA: return "SIN REFERENCIA";
    default:                    return "ESPERANDO";
  }
}

inline const char *nombreNivel(Nivel n) {
  switch (n) {
    case Nivel::LEVE:     return "LEVE";
    case Nivel::MODERADA: return "MODERADA";
    case Nivel::ALTA:     return "ALTA";
    default:              return "SIN FATIGA";
  }
}

// Lo que el analizador le devuelve al .ino
struct Resultado {
  float    t_s;             // tiempo [s] (si trae una repetición nueva: el del pico)
  Etapa    etapa;
  Nivel    nivel;           // válido en MIDIENDO
  uint16_t reps;            // repeticiones de la serie
  float    caida_pct;       // caída de la MDF respecto de la referencia [%] (0 si no hay referencia)
  float    mdf;             // MDF de la última repetición [Hz]
  float    mnf;             // MNF de la última repetición [Hz]
  float    mdf_ref;         // referencia [Hz] (0 si todavía no hay)
  float    mdf_actual;      // MDF actual (recta de la serie) [Hz]
  float    pendiente;       // pendiente de la recta [Hz por repetición]
  float    mnf_ref;         // referencia de la MNF [Hz]
  float    mnf_actual;      // MNF actual (recta de la serie) [Hz]
  float    caida_mdf_pct;   // caída de la MDF [%]
  float    caida_mnf_pct;   // caída de la MNF [%] (sin escalar)
  float    rms;             // RMS de la repetición (si es nueva) o envolvente actual [mV]
  uint8_t  repsCalibracion; // repeticiones usadas en la calibración hasta ahora
  bool     repNueva;        // este resultado trae una repetición nueva
  bool     saturada;        // esa repetición saturó el ADC
  bool     repValida;       // esa repetición entra en la referencia / fatiga
  bool     contraido;       // la envolvente está por encima del umbral de reposo
  const char *evento;       // qué pasó (para el Serial); nullptr si es un resultado periódico
};

// =============================================================================
// HERRAMIENTAS
// =============================================================================

// Filtro IIR de 2.º orden (biquad), forma directa II transpuesta.
//   y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] − a1·y[n-1] − a2·y[n-2]
struct Biquad {
  enum Tipo { PASABAJOS, PASAALTOS, NOTCH };
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float z1 = 0, z2 = 0;   // memoria del filtro

  // Coeficientes del "Audio EQ Cookbook" (transformada bilineal con prewarping)
  void disenar(Tipo tipo, float f0, float q, float fs) {
    const float w0 = 2.0f * (float)M_PI * f0 / fs;
    const float c = cosf(w0), alfa = sinf(w0) / (2.0f * q);
    const float a0 = 1.0f + alfa;
    if (tipo == PASABAJOS)      { b0 = (1 - c) / 2; b1 = 1 - c;    b2 = (1 - c) / 2; }
    else if (tipo == PASAALTOS) { b0 = (1 + c) / 2; b1 = -(1 + c); b2 = (1 + c) / 2; }
    else                        { b0 = 1;           b1 = -2 * c;   b2 = 1; }
    a1 = -2 * c;
    a2 = 1 - alfa;
    b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;
    z1 = z2 = 0;
  }

  float paso(float x) {
    const float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

// Mediana de n valores (n chico, hasta 16)
inline float mediana(const float *v, int n) {
  float t[16];
  if (n > 16) n = 16;
  for (int i = 0; i < n; i++) t[i] = v[i];
  for (int i = 1; i < n; i++) {           // ordenamiento por inserción
    const float k = t[i];
    int j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return (n % 2) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
}

// =============================================================================
// ANALIZADOR
// =============================================================================

class Analizador {
public:
  Analizador() { iniciar(); }

  // Empieza una serie: mide el reposo, calibra y después mide la fatiga
  // Serie siguiente: si ya hay referencia (y REF_MANTENER_ENTRE_SERIES), se conserva
  void iniciarSerie() {
    if (!REF_MANTENER_ENTRE_SERIES) { mdfRef_ = 0; mnfRef_ = 0; refCongelada_ = false; }
    // Si ya hay referencia (de la serie anterior), se congela la que se estaba usando
    if (mdfRef_ > 0 && !refCongelada_) {
      refCongelada_ = true;
      refCongeladaMdf_ = mdfRefUsada_;
      refCongeladaMnf_ = mnfRefUsada_;
    }
    reiniciarSerie();
    etapa_ = Etapa::RELAJA;
    evento_ = mdfRef_ > 0 ? "serie iniciada (misma referencia): relaja el brazo" : "serie iniciada: relaja el brazo";
  }

  // Sesión nueva (otro día, otra persona, electrodos recolocados): olvida la referencia
  void nuevaSesion() {
    mdfRef_ = 0; mnfRef_ = 0; refCongelada_ = false;
    iniciarSerie();
    evento_ = "sesion nueva: relaja el brazo";
  }

  bool tieneReferencia() const { return mdfRef_ > 0; }

  // Termina la serie: deja de contar. Se conservan las repeticiones y la fatiga
  // de la serie para que el último resultado las informe.
  void detenerSerie() {
    if (etapa_ == Etapa::ESPERANDO) return;
    repPendiente_ = false;
    etapa_ = Etapa::ESPERANDO;
    evento_ = "serie detenida";
  }

  Etapa etapa() const { return etapa_; }

  // ---- Diagnóstico: lo que el analizador está viendo (solo lectura, para graficar) ----
  float muestraProcesada() const { return muestraProcesada_; }   // última muestra filtrada y recortada [mV]
  float envolvente() const { return envolvente_; }               // RMS de los últimos 0,5 s [mV]
  float limiteRecorte() const { return (RECORTE_ACTIVO && escala_ > 0) ? RECORTE_K * escala_ : 0; }
  float reposo() const { return reposo_; }                       // 0 hasta que se mide
  bool  buscandoPico() const { return buscandoPico_; }           // false = buscando valle
  float maximo() const { return maximo_; }                       // máximo desde el último valle
  float minimo() const { return minimo_; }                       // mínimo desde el último pico

  // Procesa una muestra cruda [mV]. Devuelve true cuando `out` tiene un resultado
  // nuevo: cada INFORME_PERIODO_S, o enseguida si pasó algo (repetición, cambio de etapa).
  bool agregar(float mV, Resultado &out) {
    const float filtrada = filtrar(mV);
    const float y = recortarPico(filtrada);
    muestraProcesada_ = y;
    guardarMuestra(y, mV);
    acumularBloque(filtrada, y);

    bool hayNovedad = false;
    if (total_ % ENV_PASO == 0) {                 // cada 50 ms: un punto de la envolvente
      cerrarBloque();
      hayNovedad = procesarPuntoEnvolvente();
    }
    if (repPendiente_ && total_ >= nPico_ + MUESTRAS_ESPECTRO / 2) hayNovedad = cerrarRepeticion();
    if (evento_) hayNovedad = true;

    if (++desdeInforme_ >= MUESTRAS_INFORME || hayNovedad) {
      desdeInforme_ = 0;
      armarResultado(out);
      return true;
    }
    return false;
  }

private:
  struct Repeticion { float mdf, mnf, rms, t; bool saturada; };   // t = tiempo del pico [s]

  // ---- Filtros y FFT ----
  Biquad filtros_[5];
  int nFiltros_ = 0;
  bool primeraMuestra_ = true;
  float dc0_ = 0;                          // continua que se resta (primera muestra)
  float hann_[NFFT], cos_[NFFT / 2], sin_[NFFT / 2];
  uint16_t bitrev_[NFFT];
  float re_[NFFT], im_[NFFT];
  float psd_[NFFT / 2 + 1];

  // ---- Señal reciente: la muestra i está en buf_[i % BUF_N] ----
  float buf_[BUF_N];
  uint8_t satBloque_[BLOQUES_SAT];         // muestras saturadas por bloque de 50 ms
  uint32_t total_ = 0;                     // muestras procesadas desde que se prendió
  int desdeInforme_ = 0;

  // ---- Envolvente y escala del recorte ----
  double sumaBloque_ = 0, sumaBloqueSinRecorte_ = 0;
  float potBloque_[BLOQUES_ENV], rmsBloqueSinRecorte_[BLOQUES_ENV];
  int posBloque_ = 0, nBloques_ = 0;
  float envolvente_ = 0, escala_ = 0;
  float muestraProcesada_ = 0;

  // ---- Serie ----
  Etapa etapa_ = Etapa::ESPERANDO;
  Nivel nivel_ = Nivel::SIN_FATIGA;
  const char *evento_ = nullptr;

  // Reposo
  int puntosQuietos_ = 0;
  double sumaReposo_ = 0;
  float reposo_ = 0;

  // Detector de picos
  bool buscandoPico_ = true;
  float maximo_ = 0, minimo_ = 0;
  uint32_t muestraMaximo_ = 0;
  float tUltimoPico_ = -1e9f;
  bool repPendiente_ = false;
  uint32_t nPico_ = 0;

  // Repeticiones
  uint16_t reps_ = 0;
  Repeticion ultima_ = {0, 0, 0, 0, false};
  bool repNueva_ = false;
  Repeticion serie_[MAX_REPS_SERIE];      // todas las repeticiones de la serie
  int nRepsSerie_ = 0;
  bool ultimaValida_ = false;
  int intentosCalibracion_ = 0;

  // Fatiga
  float mdfRef_ = 0;
  float mdfSerie_[MAX_REPS_SERIE];
  float mnfSerie_[MAX_REPS_SERIE];
  int nSerie_ = 0;
  float mdfActual_ = 0, caida_ = 0, pendiente_ = 0;
  float mnfRef_ = 0, mnfActual_ = 0, caidaMdf_ = 0, caidaMnf_ = 0;
  float mdfRefUsada_ = 0, mnfRefUsada_ = 0;    // la referencia que se usó en la última rep
  // Referencia de la sesión congelada al terminar la serie 1 (las series siguientes la usan tal cual)
  bool  refCongelada_ = false;
  float refCongeladaMdf_ = 0, refCongeladaMnf_ = 0;
  // Espacio para Theil-Sen: todas las pendientes entre pares de reps
  float pares_[MAX_REPS_SERIE * (MAX_REPS_SERIE - 1) / 2];
  float aux_[MAX_REPS_SERIE];
  int repsSobreUmbral_ = 0;

  // ===========================================================================
  // Inicialización
  // ===========================================================================
  void iniciar() {
    disenarFiltros();
    prepararFft();
    primeraMuestra_ = true; dc0_ = 0; total_ = 0; desdeInforme_ = 0;
    memset(buf_, 0, sizeof(buf_));
    memset(satBloque_, 0, sizeof(satBloque_));
    sumaBloque_ = 0; sumaBloqueSinRecorte_ = 0;
    posBloque_ = 0; nBloques_ = 0; envolvente_ = 0; escala_ = 0;
    etapa_ = Etapa::ESPERANDO;
    mdfRef_ = 0; mnfRef_ = 0; refCongelada_ = false;
    reiniciarSerie();
  }

  void disenarFiltros() {
    // Butterworth de orden 4 = 2 biquads con Q = 1/(2·cos(π/8)) y 1/(2·cos(3π/8))
    const float q1 = 0.54119610f, q2 = 1.30656296f;
    nFiltros_ = 0;
    filtros_[nFiltros_++].disenar(Biquad::PASAALTOS, FILTRO_F_BAJA, q1, FS);
    filtros_[nFiltros_++].disenar(Biquad::PASAALTOS, FILTRO_F_BAJA, q2, FS);
    filtros_[nFiltros_++].disenar(Biquad::PASABAJOS, FILTRO_F_ALTA, q1, FS);
    filtros_[nFiltros_++].disenar(Biquad::PASABAJOS, FILTRO_F_ALTA, q2, FS);
    if (NOTCH_ACTIVO) filtros_[nFiltros_++].disenar(Biquad::NOTCH, NOTCH_F, NOTCH_Q, FS);
  }

  // Tablas que la FFT usa siempre: se calculan una sola vez
  void prepararFft() {
    for (int i = 0; i < NFFT; i++) hann_[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / NFFT);
    for (int i = 0; i < NFFT / 2; i++) {
      cos_[i] = cosf(2.0f * (float)M_PI * i / NFFT);
      sin_[i] = -sinf(2.0f * (float)M_PI * i / NFFT);
    }
    for (int i = 0; i < NFFT; i++) {
      int r = 0;
      for (int b = 0; b < LOG2_NFFT; b++) if (i & (1 << b)) r |= 1 << (LOG2_NFFT - 1 - b);
      bitrev_[i] = (uint16_t)r;
    }
  }

  void reiniciarSerie() {
    nivel_ = Nivel::SIN_FATIGA;
    puntosQuietos_ = 0; sumaReposo_ = 0; reposo_ = 0;
    buscandoPico_ = true; maximo_ = 0; minimo_ = 0; muestraMaximo_ = 0;
    tUltimoPico_ = -1e9f; repPendiente_ = false;
    reps_ = 0; repNueva_ = false;
    memset(&ultima_, 0, sizeof(ultima_));
    nRepsSerie_ = 0; ultimaValida_ = false; intentosCalibracion_ = 0;
    nSerie_ = 0;                             // mdfRef_ NO se borra: es de la sesión
    mdfActual_ = 0; caida_ = 0; pendiente_ = 0; repsSobreUmbral_ = 0;
    mnfActual_ = 0; caidaMdf_ = 0; caidaMnf_ = 0;
  }

  // ===========================================================================
  // Cada muestra
  // ===========================================================================
  float filtrar(float mV) {
    if (primeraMuestra_) { dc0_ = mV; primeraMuestra_ = false; }   // evita el salto inicial del pasaaltos
    float y = mV - dc0_;
    for (int i = 0; i < nFiltros_; i++) y = filtros_[i].paso(y);
    return y;
  }

  float recortarPico(float y) const {
    if (!RECORTE_ACTIVO || escala_ <= 0) return y;
    const float limite = RECORTE_K * escala_;
    if (y > limite) return limite;
    if (y < -limite) return -limite;
    return y;
  }

  void guardarMuestra(float y, float mV) {
    buf_[total_ % BUF_N] = y;
    if (mV <= SAT_MIN_MV || mV >= SAT_MAX_MV) satBloque_[(total_ / ENV_PASO) % BLOQUES_SAT]++;
    total_++;
  }

  void acumularBloque(float sinRecorte, float conRecorte) {
    sumaBloqueSinRecorte_ += (double)sinRecorte * sinRecorte;   // para la escala del recorte
    sumaBloque_ += (double)conRecorte * conRecorte;             // para la envolvente
  }

  // ===========================================================================
  // Cada 50 ms (un punto de la envolvente)
  // ===========================================================================
  void cerrarBloque() {
    potBloque_[posBloque_] = (float)(sumaBloque_ / ENV_PASO);
    rmsBloqueSinRecorte_[posBloque_] = (float)sqrt(sumaBloqueSinRecorte_ / ENV_PASO);
    posBloque_ = (posBloque_ + 1) % BLOQUES_ENV;
    if (nBloques_ < BLOQUES_ENV) nBloques_++;
    sumaBloque_ = 0;
    sumaBloqueSinRecorte_ = 0;

    if (nBloques_ == BLOQUES_ENV) escala_ = mediana(rmsBloqueSinRecorte_, BLOQUES_ENV);
    satBloque_[(total_ / ENV_PASO) % BLOQUES_SAT] = 0;   // el bloque que empieza

    double potencia = 0;
    for (int i = 0; i < nBloques_; i++) potencia += potBloque_[i];
    envolvente_ = (float)sqrt(potencia / nBloques_);
  }

  bool procesarPuntoEnvolvente() {
    switch (etapa_) {
      case Etapa::ESPERANDO: return false;
      case Etapa::RELAJA:    return medirReposo();
      default:               detectarPico(); return false;   // calibrando, midiendo o sin referencia
    }
  }

  // RELAJA: REPOSO_DURACION_S seguidos con la envolvente por debajo de REPOSO_MAX_MV
  bool medirReposo() {
    if (total_ < (uint32_t)(FILTRO_ESPERA_S * FS) || nBloques_ < BLOQUES_ENV) return false;
    if (envolvente_ >= REPOSO_MAX_MV) {      // se movió: vuelve a contar
      puntosQuietos_ = 0;
      sumaReposo_ = 0;
      return false;
    }
    puntosQuietos_++;
    sumaReposo_ += envolvente_;
    if (puntosQuietos_ < PUNTOS_REPOSO) return false;

    reposo_ = (float)(sumaReposo_ / puntosQuietos_);
    buscandoPico_ = true; maximo_ = 0; minimo_ = reposo_; tUltimoPico_ = -1e9f;
    if (mdfRef_ > 0) {                         // serie 2 en adelante: ya hay referencia
      etapa_ = Etapa::MIDIENDO;
      evento_ = "reposo medido: empeza (referencia de la serie anterior)";
    } else {
      etapa_ = Etapa::CALIBRANDO;
      evento_ = "reposo medido: empeza";
    }
    return true;
  }

  // Detector de picos: alterna entre buscar un pico y buscar un valle
  void detectarPico() {
    const float v = envolvente_;
    const uint32_t muestra = ENV_CENTRADA ? total_ - (uint32_t)(BLOQUES_ENV * ENV_PASO / 2) : total_;

    if (!buscandoPico_) {                    // buscando valle
      if (v < minimo_) minimo_ = v;
      if (v > minimo_ * (1 + REP_SUBIDA_VALLE)) {   // empezó otra contracción
        buscandoPico_ = true;
        maximo_ = v;
        muestraMaximo_ = muestra;
      }
      return;
    }

    if (v > maximo_) { maximo_ = v; muestraMaximo_ = muestra; }
    const float tMaximo = muestraMaximo_ / FS;
    const bool cayo        = v < maximo_ * (1 - REP_CAIDA_PICO);
    const bool habiaSubido = maximo_ > (1 + REP_SUBIDA_VALLE) * minimo_;
    const bool superaReposo = maximo_ > REP_FACTOR_REPOSO * reposo_;
    const bool separada    = tMaximo - tUltimoPico_ >= REP_SEPARACION_MIN_S;
    if (cayo && habiaSubido && superaReposo && separada) {
      tUltimoPico_ = tMaximo;
      buscandoPico_ = false;
      minimo_ = v;
      repPendiente_ = true;                  // se analiza cuando estén los 0,75 s posteriores al pico
      nPico_ = muestraMaximo_;
    } else if (cayo && !separada) {
      // La envolvente ya cayó pero el máximo está a menos de REP_SEPARACION_MIN_S del pico
      // anterior: es la misma contracción (o un rebote). Se descarta ese máximo y se vuelve a
      // buscar un valle. Si no, el máximo quedaba guardado para siempre (su tiempo no cambia,
      // así que nunca llega a estar "separado") y todos los picos siguientes, más chicos,
      // nunca lo superaban: la ESP dejaba de contar repeticiones.
      buscandoPico_ = false;
      minimo_ = v;
    }
  }

  // ===========================================================================
  // Cada repetición
  // ===========================================================================
  bool cerrarRepeticion() {
    repPendiente_ = false;
    const Repeticion r = analizarVentana(nPico_);
    reps_++;
    ultima_ = r;
    repNueva_ = true;

    if (nRepsSerie_ < MAX_REPS_SERIE) serie_[nRepsSerie_++] = r;
    ultimaValida_ = esValida(r);

    switch (etapa_) {
      case Etapa::CALIBRANDO:
        evento_ = "calibrando";
        intentarCalibrar();
        break;
      case Etapa::SIN_REFERENCIA:
        evento_ = "repeticion (sin referencia)";
        break;
      default:                               // MIDIENDO
        if (ultimaValida_) {
          if (nSerie_ < MAX_REPS_SERIE) { mdfSerie_[nSerie_] = r.mdf; mnfSerie_[nSerie_] = r.mnf; nSerie_++; }
          actualizarFatiga();
          evento_ = "repeticion";
        } else {
          evento_ = "repeticion contada (incompleta o saturada: no entra en la fatiga)";
        }
    }
    return true;
  }

  // MDF, MNF, RMS y saturación en la ventana centrada en el pico
  Repeticion analizarVentana(uint32_t pico) {
    uint32_t ini = pico > (uint32_t)(MUESTRAS_ESPECTRO / 2) ? pico - MUESTRAS_ESPECTRO / 2 : 0;
    uint32_t fin = ini + MUESTRAS_ESPECTRO;
    if (fin > total_) fin = total_;
    if (total_ > (uint32_t)BUF_N && ini < total_ - BUF_N) ini = total_ - BUF_N;   // no debería pasar

    // Espectro de Welch: tramos de NFFT con 50 % de solape
    memset(psd_, 0, sizeof(psd_));
    for (uint32_t a = ini; a + NFFT <= fin; a += SOLAPE) acumularTramo(a);

    Repeticion r;
    calcularFrecuencias(r.mdf, r.mnf);

    double energia = 0;
    for (uint32_t i = ini; i < fin; i++) energia += (double)buf_[i % BUF_N] * buf_[i % BUF_N];
    r.rms = (float)sqrt(energia / (fin - ini));

    int saturadas = 0;
    for (uint32_t b = ini / ENV_PASO; b <= (fin - 1) / ENV_PASO; b++) saturadas += satBloque_[b % BLOQUES_SAT];
    r.saturada = saturadas > SAT_FRACCION_MAX * (fin - ini);

    r.t = pico / FS;
    return r;
  }

  // ===========================================================================
  // Repetición válida
  // ===========================================================================
  // Mediana de las RMS de todas las repeticiones de la serie (hasta MAX_REPS_SERIE)
  float rmsMedianaSerie() const {
    float t[MAX_REPS_SERIE];
    const int n = nRepsSerie_;
    for (int i = 0; i < n; i++) {                        // inserción ordenada (n <= 100)
      float v = serie_[i].rms;
      int j = i - 1;
      while (j >= 0 && t[j] > v) { t[j + 1] = t[j]; j--; }
      t[j + 1] = v;
    }
    if (n == 0) return 0;
    return (n % 2) ? t[n / 2] : 0.5f * (t[n / 2 - 1] + t[n / 2]);
  }

  bool esValida(const Repeticion &r) const {
    if (VALIDA_SIN_SATURAR && r.saturada) return false;
    return r.rms >= VALIDA_RMS_MIN * rmsMedianaSerie();
  }

  // ===========================================================================
  // Calibración de la referencia
  // ===========================================================================
  void intentarCalibrar() {
    intentosCalibracion_++;
    if (intentosCalibracion_ < REF_REPS_OBSERVAR) return;     // todavía junta repeticiones

    int elegidas[REF_REPS];
    const int n = elegirReferencia(elegidas);
    if (n == REF_REPS && (!REF_EXIGIR_PAREJAS || repeticionesParejas(elegidas))) {
      fijarReferencia(elegidas);
    } else if (intentosCalibracion_ >= REF_INTENTOS_MAX) {
      etapa_ = Etapa::SIN_REFERENCIA;
      evento_ = "no se pudo calibrar (las repeticiones se siguen contando): revisa electrodos y reinicia la serie";
    } else {
      evento_ = "calibrando: faltan repeticiones validas";
    }
  }

  // Las primeras REF_REPS repeticiones válidas de la serie (índices en serie_)
  int elegirReferencia(int *elegidas) const {
    int n = 0;
    for (int i = REF_INCLUIR_PRIMERA ? 0 : 1; i < nRepsSerie_ && n < REF_REPS; i++)
      if (esValida(serie_[i])) elegidas[n++] = i;
    return n;
  }

  // ¿Las repeticiones elegidas tienen ritmo regular, fuerza pareja y MDF poco dispersa?
  bool repeticionesParejas(const int *elegidas) {
    float intervalos[REF_REPS], rms[REF_REPS], mdf[REF_REPS];
    for (int i = 0; i < REF_REPS; i++) {
      const Repeticion &r = serie_[elegidas[i]];
      if (i > 0) intervalos[i - 1] = r.t - serie_[elegidas[i - 1]].t;
      rms[i] = r.rms;
      mdf[i] = r.mdf;
    }

    const float intervaloMediano = mediana(intervalos, REF_REPS - 1);
    for (int i = 0; i < REF_REPS - 1; i++)
      if (fabsf(intervalos[i] - intervaloMediano) > REF_TOL_RITMO * intervaloMediano) {
        evento_ = "calibrando: ritmo irregular";
        return false;
      }

    const float rmsMediana = mediana(rms, REF_REPS);
    for (int i = 0; i < REF_REPS; i++)
      if (fabsf(rms[i] - rmsMediana) > REF_TOL_RMS * rmsMediana) {
        evento_ = "calibrando: fuerza dispar entre repeticiones";
        return false;
      }

    if (coeficienteVariacion(mdf, REF_REPS) > REF_CV_MDF_MAX) {
      evento_ = "calibrando: MDF muy dispersa";
      return false;
    }
    return true;
  }

  static float coeficienteVariacion(const float *v, int n) {
    float promedio = 0, varianza = 0;
    for (int i = 0; i < n; i++) promedio += v[i];
    promedio /= n;
    for (int i = 0; i < n; i++) varianza += (v[i] - promedio) * (v[i] - promedio);
    return sqrtf(varianza / (n - 1)) / promedio;
  }

  void fijarReferencia(const int *elegidas) {
    float mdf[REF_REPS], mnf[REF_REPS];
    for (int i = 0; i < REF_REPS; i++) { mdf[i] = serie_[elegidas[i]].mdf; mnf[i] = serie_[elegidas[i]].mnf; }
    mdfRef_ = mediana(mdf, REF_REPS);
    mnfRef_ = mediana(mnf, REF_REPS);
    // La recta empieza con todas las repeticiones válidas desde la primera elegida
    nSerie_ = 0;
    for (int i = elegidas[0]; i < nRepsSerie_; i++)
      if (esValida(serie_[i]) && nSerie_ < MAX_REPS_SERIE) {
        mdfSerie_[nSerie_] = serie_[i].mdf; mnfSerie_[nSerie_] = serie_[i].mnf; nSerie_++;
      }
    etapa_ = Etapa::MIDIENDO;
    nivel_ = Nivel::SIN_FATIGA;
    repsSobreUmbral_ = 0;
    actualizarFatiga();
    evento_ = "referencia lista";
  }

  // ===========================================================================
  // Fatiga
  // ===========================================================================
  void actualizarFatiga() {
    float ordMdf, ordMnf;
    estimarRecta(mdfSerie_, nSerie_, mdfActual_, ordMdf, pendiente_);
    float pendMnf;
    estimarRecta(mnfSerie_, nSerie_, mnfActual_, ordMnf, pendMnf);

    // Referencia: la congelada de la serie 1, o la de esta serie
    float refMdf = mdfRef_, refMnf = mnfRef_;
    if (refCongelada_) {
      refMdf = refCongeladaMdf_;
      refMnf = refCongeladaMnf_;
    } else if (FATIGA_REF_DE_LA_RECTA && nSerie_ >= FATIGA_REPS_MIN_RECTA) {
      refMdf = ordMdf;                         // valor de la recta en la primera rep válida
      refMnf = ordMnf;
    }
    mdfRefUsada_ = refMdf;
    mnfRefUsada_ = refMnf;

    caidaMdf_ = refMdf > 0 ? 100.0f * (refMdf - mdfActual_) / refMdf : 0;
    caidaMnf_ = refMnf > 0 ? 100.0f * (refMnf - mnfActual_) / refMnf : 0;
    const float mnfEnEscalaMdf = caidaMnf_ / MNF_FACTOR_UMBRALES;   // así comparten los umbrales
    switch (FATIGA_SENAL) {
      case Senal::MDF:       caida_ = caidaMdf_; break;
      case Senal::MNF:       caida_ = mnfEnEscalaMdf; break;
      case Senal::MDF_Y_MNF: caida_ = caidaMdf_ < mnfEnEscalaMdf ? caidaMdf_ : mnfEnEscalaMdf; break;
      case Senal::MDF_O_MNF: caida_ = caidaMdf_ > mnfEnEscalaMdf ? caidaMdf_ : mnfEnEscalaMdf; break;
    }
    aplicarNivel(nivelPorCaida(caida_));
  }

  // Recta y = ordenada + pendiente·i (i = posición entre las reps válidas, la primera es 0).
  // actual = valor de la recta en la última rep. Con menos de FATIGA_REPS_MIN_RECTA reps:
  // actual = mediana de las últimas FATIGA_REPS_MEDIANA y ordenada = esa misma mediana.
  void estimarRecta(const float *y, int n, float &actual, float &ordenada, float &pendiente) {
    const int k = n < FATIGA_REPS_MEDIANA ? n : FATIGA_REPS_MEDIANA;
    actual = n > 0 ? mediana(y + n - k, k) : 0;
    ordenada = actual;
    pendiente = 0;
    if (n < FATIGA_REPS_MIN_RECTA) return;

    if (FATIGA_RECTA_ROBUSTA) {
      // Theil-Sen: pendiente = mediana de (y[j] − y[i]) / (j − i) para todos los pares i < j
      int m = 0;
      for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) pares_[m++] = (y[j] - y[i]) / (float)(j - i);
      pendiente = medianaLarga(pares_, m);
      // ordenada = mediana de (y[i] − pendiente·i)
      for (int i = 0; i < n; i++) aux_[i] = y[i] - pendiente * i;
      ordenada = medianaLarga(aux_, n);
    } else {
      // Mínimos cuadrados: b = Σ(i − ī)(yi − ȳ) / Σ(i − ī)²
      const float iMedio = (n - 1) / 2.0f;
      float yMedia = 0, sxy = 0, sxx = 0;
      for (int i = 0; i < n; i++) yMedia += y[i];
      yMedia /= n;
      for (int i = 0; i < n; i++) {
        sxy += (i - iMedio) * (y[i] - yMedia);
        sxx += (i - iMedio) * (i - iMedio);
      }
      pendiente = sxy / sxx;
      ordenada = yMedia - pendiente * iMedio;
    }
    if (FATIGA_USAR_RECTA) actual = ordenada + pendiente * (n - 1);
  }

  // Mediana de muchos valores (desordena el arreglo: se le pasa un espacio de trabajo)
  static float medianaLarga(float *v, int n) {
    if (n <= 0) return 0;
    std::nth_element(v, v + n / 2, v + n);
    const float alta = v[n / 2];
    if (n % 2) return alta;
    const float baja = *std::max_element(v, v + n / 2);
    return 0.5f * (baja + alta);
  }

  static Nivel nivelPorCaida(float caida) {
    if (caida >= FATIGA_ALTA_PCT)     return Nivel::ALTA;
    if (caida >= FATIGA_MODERADA_PCT) return Nivel::MODERADA;
    if (caida >= FATIGA_LEVE_PCT)     return Nivel::LEVE;
    return Nivel::SIN_FATIGA;
  }

  // Para subir de nivel hay que confirmarlo FATIGA_CONFIRMAR_REPS repeticiones seguidas
  void aplicarNivel(Nivel propuesto) {
    if (propuesto > nivel_) {
      if (++repsSobreUmbral_ >= FATIGA_CONFIRMAR_REPS) {
        nivel_ = propuesto;
        repsSobreUmbral_ = 0;
      }
      return;
    }
    repsSobreUmbral_ = 0;
    if (!FATIGA_SOLO_SUBE) nivel_ = propuesto;
  }

  // ===========================================================================
  // Espectro
  // ===========================================================================
  // Un tramo de Welch que empieza en la muestra `a`: resta la media, ventana de Hann, FFT,
  // y suma |X(k)|² al espectro de la repetición.
  void acumularTramo(uint32_t a) {
    float media = 0;
    for (int i = 0; i < NFFT; i++) media += buf_[(a + i) % BUF_N];
    media /= NFFT;
    for (int i = 0; i < NFFT; i++) {
      re_[i] = (buf_[(a + i) % BUF_N] - media) * hann_[i];
      im_[i] = 0;
    }
    fft();
    for (int k = 0; k <= NFFT / 2; k++) psd_[k] += re_[k] * re_[k] + im_[k] * im_[k];
  }

  // MNF = Σ f·P / Σ P.  MDF = frecuencia que deja la mitad de la potencia de cada lado.
  void calcularFrecuencias(float &mdf, float &mnf) const {
    const float df = FS / NFFT;
    const int kMin = (int)ceilf(ESPECTRO_F_MIN / df), kMax = (int)floorf(ESPECTRO_F_MAX / df);
    double potencia = 0, momento = 0;
    for (int k = kMin; k <= kMax; k++) { potencia += psd_[k]; momento += psd_[k] * k * df; }
    mdf = mnf = 0;
    if (potencia <= 0) return;
    mnf = (float)(momento / potencia);

    const double mitad = potencia / 2;
    double acumulada = 0;
    for (int k = kMin; k <= kMax; k++) {
      if (acumulada + psd_[k] >= mitad) {    // interpolación lineal (como np.interp en Python)
        const double fraccion = psd_[k] > 0 ? (mitad - acumulada) / psd_[k] : 0;
        mdf = (float)((k - 1 + fraccion) * df);
        return;
      }
      acumulada += psd_[k];
    }
  }

  // FFT radix-2 (Cooley-Tukey) sobre re_/im_, en el lugar
  void fft() {
    for (int i = 0; i < NFFT; i++) {         // reordenar por bit-reverso
      const int j = bitrev_[i];
      if (j > i) {
        float t = re_[i]; re_[i] = re_[j]; re_[j] = t;
        t = im_[i]; im_[i] = im_[j]; im_[j] = t;
      }
    }
    for (int largo = 2; largo <= NFFT; largo <<= 1) {   // mariposas
      const int mitad = largo >> 1, salto = NFFT / largo;
      for (int i = 0; i < NFFT; i += largo) {
        for (int k = 0; k < mitad; k++) {
          const float wr = cos_[k * salto], wi = sin_[k * salto];
          const int a = i + k, b = a + mitad;
          const float tr = re_[b] * wr - im_[b] * wi;
          const float ti = re_[b] * wi + im_[b] * wr;
          re_[b] = re_[a] - tr; im_[b] = im_[a] - ti;
          re_[a] += tr;         im_[a] += ti;
        }
      }
    }
  }

  // ===========================================================================
  // Salida
  // ===========================================================================
  void armarResultado(Resultado &r) {
    memset(&r, 0, sizeof(r));
    r.t_s = repNueva_ ? ultima_.t : total_ / FS;
    r.etapa = etapa_;
    r.nivel = nivel_;
    r.reps = reps_;
    r.caida_pct = mdfRef_ > 0 ? caida_ : 0;
    r.mdf = ultima_.mdf;
    r.mnf = ultima_.mnf;
    r.mdf_ref = refCongelada_ || nSerie_ > 0 ? mdfRefUsada_ : mdfRef_;
    r.mdf_actual = mdfActual_;
    r.pendiente = pendiente_;
    r.mnf_ref = refCongelada_ || nSerie_ > 0 ? mnfRefUsada_ : mnfRef_;
    r.mnf_actual = mnfActual_;
    r.caida_mdf_pct = mdfRef_ > 0 ? caidaMdf_ : 0;
    r.caida_mnf_pct = mdfRef_ > 0 ? caidaMnf_ : 0;
    r.rms = repNueva_ ? ultima_.rms : envolvente_;
    r.repsCalibracion = (uint8_t)intentosCalibracion_;
    r.repNueva = repNueva_;
    r.saturada = repNueva_ && ultima_.saturada;
    r.repValida = repNueva_ && ultimaValida_;
    r.contraido = reposo_ > 0 && envolvente_ > REP_FACTOR_REPOSO * reposo_;
    r.evento = evento_;
    evento_ = nullptr;
    repNueva_ = false;
  }
};

}  // namespace fatiga
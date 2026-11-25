"""graficar_esp.py — TEMPORAL: muestra en tiempo real lo que procesa la ESP32.

La ESP manda por BLE (con ENVIAR_DIAGNOSTICO = true en el .ino):
  - cada 50 ms, la señal ya filtrada y recortada y el estado del detector de repeticiones;
  - cada 0,5 s y en cada repetición, el mismo JSON que recibe la app.
Este programa no recalcula el procesamiento: dibuja lo que calculó la ESP. Lo único que
calcula es el espectro de cada repetición (con la misma señal y el mismo método que la ESP)
para poder mostrarlo y comparar su MDF con la de la ESP.

Ventana:
  1. Señal procesada (pasabanda + notch + recorte de picos), el límite del recorte y la
     envolvente encima.
  2. Envolvente y detector: nivel de reposo, pico mínimo, el umbral que está mirando el
     detector en cada momento, los picos detectados y la ventana de 1,5 s de cada uno.
  3. Espectro de la última repetición (y el de la primera, para ver el corrimiento) con su MDF.
  4. MDF de cada repetición, MDF actual (recta), referencia y umbrales de fatiga.
Botones START y STOP: mandan 'I' y 'S' a la ESP, como lo va a hacer la app.

Uso (con el entorno virtual activado):
    python graficar_esp.py                 # hasta cerrar la ventana
    python graficar_esp.py 120             # 120 segundos
    python graficar_esp.py --sin-iniciar   # no manda START al conectar

Guarda en outputs/: senal_<fecha>.csv, detector_<fecha>.csv y resultados_<fecha>.csv.
Necesita: pip install bleak numpy scipy matplotlib
"""

import argparse
import asyncio
import csv
import json
import os
import struct
import threading
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
from bleak import BleakClient, BleakScanner
from matplotlib.animation import FuncAnimation
from matplotlib.ticker import MaxNLocator
from matplotlib.widgets import Button
from scipy import signal

# =============================================================================
# CONFIGURACIÓN (tiene que coincidir con el firmware)
# =============================================================================

# ---- BLE ----
NOMBRE_BLE = "MyoSignals"
UUID_DIAGNOSTICO = "8d1f0002-6c3a-4b8e-9a2e-3f1b2c4d5e60"
UUID_FATIGA = "8d1f0003-6c3a-4b8e-9a2e-3f1b2c4d5e60"
UUID_COMANDO = "8d1f0004-6c3a-4b8e-9a2e-3f1b2c4d5e60"
CMD_INICIAR, CMD_DETENER = b"I", b"S"

# ---- Paquete de diagnóstico (PaqueteDiagnostico en el .ino) ----
FORMATO_DIAG = "<IfffffffBB100h"
CAMPOS_DIAG = ["bloque", "envolvente", "limite_recorte", "reposo", "maximo", "minimo",
               "umbral_pico", "umbral_valle", "etapa", "buscando_pico"]
ESCALA_MUESTRA = 10.0         # la ESP manda décimas de mV

# ---- Procesamiento (fatiga_dsp.h) ----
FS = 2000
MUESTRAS_BLOQUE = 100
ENV_VENTANA_S = 0.5
ENV_CENTRADA = True           # la envolvente se dibuja en el centro de su ventana, como la ESP
REP_FACTOR_REPOSO = 2.0
ESPECTRO_VENTANA_S = 1.5
NFFT = 512
BANDA_HZ = (30, 450)
NIVELES_PCT = {"leve": 7, "moderada": 12, "alta": 20}

# ---- Gráfico ----
VENTANA_S = 10                # segundos visibles de señal y envolvente
HISTORIA_S = 30               # segundos de señal que se guardan en memoria (para los espectros)
REFRESCO_MS = 100
DIEZMADO = 2                  # se dibuja 1 de cada 2 muestras (solo el dibujo)

ESTADOS = {   # estado del JSON -> (texto, color)
    "esperando": ("Esperando START", "gray"),
    "relaja": ("Relajá el brazo", "gray"),
    "calibrando": ("Calibrando", "tab:blue"),
    "sin": ("Sin fatiga", "tab:green"),
    "leve": ("Fatiga leve", "goldenrod"),
    "moderada": ("Fatiga moderada", "tab:orange"),
    "alta": ("Fatiga ALTA", "tab:red"),
    "sinref": ("Sin referencia", "black"),
}

CARPETA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "outputs")


# =============================================================================
# DATOS COMPARTIDOS ENTRE EL HILO BLE Y EL GRÁFICO
# =============================================================================
class Datos:
    """Todo se lee y escribe con `lock` tomado."""

    def __init__(self):
        self.lock = threading.Lock()
        n = int(HISTORIA_S * FS)
        self.n_muestra = deque(maxlen=n)      # número de muestra (t = n / FS)
        self.senal = deque(maxlen=n)          # señal procesada [mV]
        nb = int(HISTORIA_S * FS / MUESTRAS_BLOQUE)
        self.det = {c: deque(maxlen=nb) for c in ("t", "envolvente", "limite", "reposo",
                                                    "umbral_pico", "umbral_valle")}
        self.reps = []                        # una entrada (dict) por repetición
        self.ultimo = None                    # último JSON
        self.conexion = "Buscando la ESP32..."
        self.cerrar = threading.Event()
        self.terminado = threading.Event()
        self.enviar_comando = None            # lo completa el hilo BLE


# =============================================================================
# RECEPCIÓN
# =============================================================================
class Receptor:
    def __init__(self, datos, escritores):
        self.datos = datos
        self.csv = escritores                 # {"senal", "detector", "resultados"}
        self.anterior = {"estado": None, "cal": None}

    def al_recibir_diagnostico(self, _, paquete: bytearray):
        if len(paquete) != struct.calcsize(FORMATO_DIAG):
            return
        valores = struct.unpack(FORMATO_DIAG, paquete)
        d = dict(zip(CAMPOS_DIAG, valores[:len(CAMPOS_DIAG)]))
        muestras = np.array(valores[len(CAMPOS_DIAG):], float) / ESCALA_MUESTRA
        n0 = d["bloque"] * MUESTRAS_BLOQUE
        n = n0 + np.arange(MUESTRAS_BLOQUE)
        # El detector usa la envolvente corrida media ventana (ENV_CENTRADA): mismo tiempo que los picos
        fin = n0 + MUESTRAS_BLOQUE
        t_env = (fin - (ENV_VENTANA_S * FS / 2 if ENV_CENTRADA else 0)) / FS
        buscando = d["buscando_pico"] == 1

        self.csv["senal"].writerows(zip(n, np.round(n / FS, 4), np.round(muestras, 1)))
        self.csv["detector"].writerow([round(t_env, 3)] + [round(d[c], 3) if isinstance(d[c], float) else d[c]
                                                           for c in CAMPOS_DIAG[1:]])
        with self.datos.lock:
            self.datos.n_muestra.extend(n)
            self.datos.senal.extend(muestras)
            det = self.datos.det
            det["t"].append(t_env)
            det["envolvente"].append(d["envolvente"])
            det["limite"].append(d["limite_recorte"] or np.nan)
            det["reposo"].append(d["reposo"] or np.nan)
            det["umbral_pico"].append(d["umbral_pico"] if buscando and d["maximo"] > 0 else np.nan)
            det["umbral_valle"].append(d["umbral_valle"] if not buscando else np.nan)

    def al_recibir_fatiga(self, _, paquete: bytearray):
        try:
            j = json.loads(bytes(paquete).decode("utf-8"))
        except ValueError:
            print(f"Paquete que no es JSON: {bytes(paquete)[:60]!r}")
            return
        self.csv["resultados"].writerow([j.get(c) for c in COLUMNAS_RESULTADOS])
        self.imprimir(j)
        with self.datos.lock:
            self.datos.ultimo = j
            if j.get("rep"):
                self.datos.reps.append({"t": j["t"] / 1000, "numero": j["reps"], "mdf": j["mdf"],
                                        "actual": j["act"], "ref": j["ref"], "estado": j["estado"],
                                        "saturada": bool(j.get("sat")), "espectro": None})

    def imprimir(self, j):
        """Una línea por repetición, por cambio de estado y por avance de la calibración."""
        t = j["t"] / 1000
        texto = ESTADOS.get(j["estado"], (j["estado"], ""))[0]
        if j.get("rep"):
            linea = f"t={t:7.1f}s  REP {j['reps']:3d}  MDF={j['mdf']:5.1f} Hz  RMS={j['rms']:5.0f} mV"
            if j["ref"] > 0:
                linea += f"  | ref={j['ref']:5.1f}  actual={j['act']:5.1f} Hz  caída={j['fatiga']:5.1f}%"
            if j.get("sat"):
                linea += "  (saturó)"
            print(linea + f"  -> {texto}")
        elif j["estado"] != self.anterior["estado"] or (j["estado"] == "calibrando" and j["cal"] != self.anterior["cal"]):
            extra = f" {min(j['cal'], 3)}/3" if j["estado"] == "calibrando" else ""
            print(f"t={t:7.1f}s  {texto}{extra}")
        self.anterior = {"estado": j["estado"], "cal": j["cal"]}


COLUMNAS_RESULTADOS = ["seq", "t", "serie", "reps", "estado", "fatiga", "mdf", "ref", "act",
                       "rms", "cal", "rep", "sat"]


async def recibir(datos, duracion, iniciar):
    print(f"Buscando {NOMBRE_BLE}...")
    dispositivo = await BleakScanner.find_device_by_name(NOMBRE_BLE, timeout=15)
    if dispositivo is None:
        print(f"No se encontró '{NOMBRE_BLE}'. Revisá el Bluetooth y que la ESP esté prendida.")
        datos.conexion = "No se encontró la ESP32"
        return

    os.makedirs(CARPETA, exist_ok=True)
    fecha = f"{datetime.now():%Y%m%d_%H%M%S}"
    rutas = {k: os.path.join(CARPETA, f"{k}_{fecha}.csv") for k in ("senal", "detector", "resultados")}
    archivos = {k: open(r, "w", newline="", encoding="utf-8") for k, r in rutas.items()}
    try:
        escritores = {k: csv.writer(f) for k, f in archivos.items()}
        escritores["senal"].writerow(["n", "t_s", "mV_procesada"])
        escritores["detector"].writerow(["t_s"] + CAMPOS_DIAG[1:])
        escritores["resultados"].writerow(COLUMNAS_RESULTADOS)
        receptor = Receptor(datos, escritores)

        async with BleakClient(dispositivo) as cliente:
            print("Conectada.")
            datos.conexion = "Conectada"
            loop = asyncio.get_running_loop()

            def enviar_comando(cmd):   # se llama desde los botones (otro hilo)
                asyncio.run_coroutine_threadsafe(cliente.write_gatt_char(UUID_COMANDO, cmd, response=True), loop)
                print("-> START" if cmd == CMD_INICIAR else "-> STOP")
            datos.enviar_comando = enviar_comando

            await cliente.start_notify(UUID_DIAGNOSTICO, receptor.al_recibir_diagnostico)
            await cliente.start_notify(UUID_FATIGA, receptor.al_recibir_fatiga)
            if iniciar:
                await cliente.write_gatt_char(UUID_COMANDO, CMD_INICIAR, response=True)
                print("START: relajá el brazo 2 s y después empezá las repeticiones.")

            inicio = loop.time()
            while not datos.cerrar.is_set() and not (duracion and loop.time() - inicio >= duracion):
                await asyncio.sleep(0.2)
            datos.enviar_comando = None
            await cliente.stop_notify(UUID_DIAGNOSTICO)
            await cliente.stop_notify(UUID_FATIGA)
    finally:
        for f in archivos.values():
            f.close()
    datos.conexion = "Desconectada"
    print("Guardado en:\n  " + "\n  ".join(rutas.values()))


def hilo_ble(datos, duracion, iniciar):
    """El BLE corre en su propio hilo para que el gráfico no lo frene."""
    try:
        asyncio.run(recibir(datos, duracion, iniciar))
    except Exception as e:
        print(f"Error de BLE: {e}")
        datos.conexion = f"Error de BLE: {e}"
    finally:
        datos.terminado.set()


# =============================================================================
# ESPECTRO (mismo método que la ESP: Welch, Hann, 512, 50 %)
# =============================================================================
def calcular_espectro(senal_rep):
    f, p = signal.welch(senal_rep, fs=FS, window="hann", nperseg=NFFT, noverlap=NFFT // 2,
                        detrend="constant")
    banda = (f >= BANDA_HZ[0]) & (f <= BANDA_HZ[1])
    fb, pb = f[banda], p[banda]
    acumulada = np.cumsum(pb)
    mdf = float(np.interp(acumulada[-1] / 2, acumulada, fb))
    mnf = float(np.sum(fb * pb) / np.sum(pb))
    return {"f": f, "p": p, "mdf": mdf, "mnf": mnf}


def completar_espectros(reps, n_muestra, senal):
    """Calcula el espectro de las repeticiones cuya ventana ya llegó completa."""
    if not len(n_muestra):
        return
    medio = int(ESPECTRO_VENTANA_S * FS / 2)
    for rep in reps:
        if rep["espectro"] is not None:
            continue
        n_pico = int(round(rep["t"] * FS))
        ini, fin = n_pico - medio, n_pico + medio
        if n_muestra[-1] < fin - 1:
            continue                                  # todavía no llegó toda la ventana
        sel = (n_muestra >= ini) & (n_muestra < fin)
        rep["espectro"] = calcular_espectro(senal[sel]) if sel.sum() >= NFFT else {}


# =============================================================================
# GRÁFICO
# =============================================================================
class Grafico:
    def __init__(self, datos, ventana_s):
        self.datos = datos
        self.ventana_s = ventana_s
        self.fig = plt.figure(figsize=(13, 9.5))
        self.fig.canvas.manager.set_window_title("MyoSignals - procesamiento en la ESP32")
        g = self.fig.add_gridspec(3, 2, height_ratios=[1.1, 1.1, 1], hspace=0.45, wspace=0.18,
                                  left=0.06, right=0.98, top=0.90, bottom=0.07)
        self.ax_sen = self.fig.add_subplot(g[0, :])
        self.ax_env = self.fig.add_subplot(g[1, :], sharex=self.ax_sen)
        self.ax_esp = self.fig.add_subplot(g[2, 0])
        self.ax_mdf = self.fig.add_subplot(g[2, 1])
        self._armar_ejes()
        self._armar_botones()
        self.marcas = []

    def _armar_ejes(self):
        a = self.ax_sen
        a.set_title("1 · Señal procesada en la ESP: pasabanda 30-450 Hz + notch 50 Hz + recorte de picos",
                    loc="left", fontsize=10)
        (self.l_senal,) = a.plot([], [], lw=0.5, color="0.45", label="señal procesada")
        (self.l_lim_sup,) = a.plot([], [], lw=1, ls="--", color="tab:red", label="límite del recorte")
        (self.l_lim_inf,) = a.plot([], [], lw=1, ls="--", color="tab:red")
        (self.l_env_sen,) = a.plot([], [], lw=2, color="tab:purple", label="envolvente")
        a.set_ylabel("mV")
        a.legend(loc="lower left", fontsize=8, ncol=3)

        a = self.ax_env
        a.set_title("2 · Envolvente (RMS 0,5 s) y detector de repeticiones", loc="left", fontsize=10)
        (self.l_env,) = a.plot([], [], lw=2, color="tab:purple", label="envolvente")
        (self.l_reposo,) = a.plot([], [], lw=1, ls=":", color="tab:green", label="reposo")
        (self.l_pico_min,) = a.plot([], [], lw=1, ls="--", color="tab:green",
                                    label=f"pico mínimo ({REP_FACTOR_REPOSO:g} × reposo)")
        (self.l_u_pico,) = a.plot([], [], lw=1.3, color="tab:orange",
                                  label="buscando pico: si baja de acá, hubo pico")
        (self.l_u_valle,) = a.plot([], [], lw=1.3, color="tab:cyan",
                                   label="buscando valle: si sube de acá, empieza otra")
        (self.l_picos,) = a.plot([], [], "v", color="tab:red", ms=9, label="pico = repetición")
        a.set_ylabel("mV")
        a.set_xlabel("t [s]  (mismo tiempo que la terminal)")
        a.legend(loc="upper left", fontsize=8, ncol=3)

        a = self.ax_esp
        a.set_title("3 · Espectro de la repetición (ventana de 1,5 s en el pico)", loc="left", fontsize=10)
        for x0, x1 in ((0, BANDA_HZ[0]), (BANDA_HZ[1], FS / 2)):
            a.axvspan(x0, x1, color="0.92", lw=0)
        (self.l_esp_primera,) = a.plot([], [], lw=1, color="0.6", label="primera repetición")
        (self.l_esp,) = a.plot([], [], lw=1.5, color="tab:blue", label="última repetición")
        self.l_mdf_esp = a.axvline(np.nan, color="tab:red", lw=1.5, label="MDF (ESP)")
        self.l_mnf = a.axvline(np.nan, color="tab:red", lw=1, ls=":", label="MNF")
        self.txt_esp = a.text(0.98, 0.95, "", transform=a.transAxes, ha="right", va="top", fontsize=9)
        a.set_xlim(0, 500)
        a.set_xlabel("frecuencia [Hz]")
        a.set_ylabel("potencia (normalizada)")
        a.legend(loc="center right", fontsize=8)

        a = self.ax_mdf
        a.set_title("4 · MDF por repetición y nivel de fatiga", loc="left", fontsize=10)
        self.sc_mdf = a.scatter([], [], s=30, zorder=3)
        (self.l_actual,) = a.plot([], [], lw=1.8, color="tab:green", label="MDF actual (recta)")
        self.l_ref = a.axhline(np.nan, color="black", ls="--", lw=1, label="referencia")
        self.l_niveles = {k: a.axhline(np.nan, color=ESTADOS[k][1], ls=":", lw=1.2, label=f"−{p} % ({k})")
                          for k, p in NIVELES_PCT.items()}
        a.set_xlabel("repetición")
        a.set_ylabel("MDF [Hz]")
        a.xaxis.set_major_locator(MaxNLocator(integer=True))
        a.legend(loc="lower left", fontsize=7, ncol=2)

    def _armar_botones(self):
        self.b_start = Button(self.fig.add_axes([0.80, 0.935, 0.08, 0.04]), "START", color="#cfe8c5")
        self.b_stop = Button(self.fig.add_axes([0.89, 0.935, 0.08, 0.04]), "STOP", color="#f3c9c4")
        self.b_start.on_clicked(lambda _: self._comando(CMD_INICIAR))
        self.b_stop.on_clicked(lambda _: self._comando(CMD_DETENER))

    def _comando(self, cmd):
        if self.datos.enviar_comando:
            self.datos.enviar_comando(cmd)

    # ---- cada cuadro ----
    def actualizar(self, _):
        d = self.datos
        with d.lock:
            n = np.fromiter(d.n_muestra, float)
            senal = np.fromiter(d.senal, float)
            det = {k: np.fromiter(v, float) for k, v in d.det.items()}
            completar_espectros(d.reps, n, senal)
            reps = [dict(r) for r in d.reps]
            ultimo = d.ultimo
            conexion = d.conexion

        self._titulo(ultimo, conexion, d.terminado.is_set())
        if len(n):
            t = n / FS
            ini = max(0.0, t[-1] - self.ventana_s)
            self._senal(t, senal, det, ini)
            self._envolvente(det, reps, ini, t[-1])
            self._marcas_reps(reps, ini)
        self._espectro(reps)
        self._mdf(reps)
        return []

    def _titulo(self, ultimo, conexion, terminado):
        if ultimo is None:
            self.fig.suptitle(conexion, x=0.06, ha="left", color="gray", fontsize=14)
            return
        texto, color = ESTADOS.get(ultimo["estado"], (ultimo["estado"], "gray"))
        linea = f"{texto}   ·   {ultimo['reps']} {'repetición' if ultimo['reps'] == 1 else 'repeticiones'}"
        if ultimo["ref"] > 0:
            linea += f"   ·   caída de la MDF {ultimo['fatiga']:.1f} %"
        if terminado:
            linea += "   (desconectada)"
        self.fig.suptitle(linea, x=0.06, ha="left", color=color, fontsize=15, fontweight="bold")

    def _senal(self, t, senal, det, ini):
        self.l_senal.set_data(t[::DIEZMADO], senal[::DIEZMADO])
        self.l_lim_sup.set_data(det["t"], det["limite"])
        self.l_lim_inf.set_data(det["t"], -det["limite"])
        self.l_env_sen.set_data(det["t"], det["envolvente"])
        vis = senal[t >= ini]
        alto = float(np.nanmax(np.abs(vis))) if len(vis) else 1.0
        self.ax_sen.set_ylim(-1.1 * alto - 1, 1.1 * alto + 1)

    def _envolvente(self, det, reps, ini, fin):
        te = det["t"]
        self.l_env.set_data(te, det["envolvente"])
        self.l_reposo.set_data(te, det["reposo"])
        self.l_pico_min.set_data(te, REP_FACTOR_REPOSO * det["reposo"])
        self.l_u_pico.set_data(te, det["umbral_pico"])
        self.l_u_valle.set_data(te, det["umbral_valle"])
        tp = np.array([r["t"] for r in reps])
        self.l_picos.set_data(tp, np.interp(tp, te, det["envolvente"]) if len(te) and len(tp) else [])
        self.ax_env.set_xlim(ini, max(fin, ini + 1))
        vis = det["envolvente"][te >= ini]
        alto = float(np.nanmax(vis)) if len(vis) else 1.0
        self.ax_env.set_ylim(0, 1.25 * alto + 1)

    def _marcas_reps(self, reps, ini):
        for m in self.marcas:
            m.remove()
        self.marcas = []
        medio = ESPECTRO_VENTANA_S / 2
        for r in reps:
            if r["t"] + medio < ini:
                continue
            color = ESTADOS.get(r["estado"], ("", "gray"))[1]
            for ax in (self.ax_sen, self.ax_env):
                self.marcas.append(ax.axvspan(r["t"] - medio, r["t"] + medio, color=color, alpha=0.10, lw=0))
                self.marcas.append(ax.axvline(r["t"], color=color, lw=1, alpha=0.7))
            if r["t"] >= ini:
                etiqueta = f"{r['numero']}" + (" sat" if r["saturada"] else "")
                self.marcas.append(self.ax_sen.text(r["t"], 0.97, etiqueta, transform=self.ax_sen.get_xaxis_transform(),
                                                    ha="center", va="top", fontsize=9, color=color,
                                                    fontweight="bold",
                                                    bbox=dict(boxstyle="round,pad=0.15", fc="white", ec="none")))

    def _espectro(self, reps):
        con_espectro = [r for r in reps if r["espectro"]]
        if not con_espectro:
            return
        ultima, primera = con_espectro[-1], con_espectro[0]
        e = ultima["espectro"]
        escala = e["p"].max() or 1
        self.l_esp.set_data(e["f"], e["p"] / escala)
        if primera is not ultima:
            ep = primera["espectro"]
            self.l_esp_primera.set_data(ep["f"], ep["p"] / (ep["p"].max() or 1))
        self.l_mdf_esp.set_xdata([ultima["mdf"]] * 2)
        self.l_mnf.set_xdata([e["mnf"]] * 2)
        self.ax_esp.set_ylim(0, 1.1)
        self.txt_esp.set_text(f"repetición {ultima['numero']}\n"
                              f"MDF ESP {ultima['mdf']:.1f} Hz · Python {e['mdf']:.1f} Hz\n"
                              f"MNF {e['mnf']:.1f} Hz")

    def _mdf(self, reps):
        if not reps:
            return
        x = np.array([r["numero"] for r in reps])
        y = np.array([r["mdf"] for r in reps])
        colores = [ESTADOS.get(r["estado"], ("", "gray"))[1] for r in reps]
        self.sc_mdf.set_offsets(np.column_stack([x, y]))
        self.sc_mdf.set_color(colores)
        actual = np.array([r["actual"] if r["ref"] > 0 else np.nan for r in reps])
        self.l_actual.set_data(x, actual)
        ref = reps[-1]["ref"]
        valores = list(y)
        if ref > 0:
            self.l_ref.set_ydata([ref, ref])
            for k, p in NIVELES_PCT.items():
                self.l_niveles[k].set_ydata([ref * (1 - p / 100)] * 2)
            valores += [ref, ref * (1 - NIVELES_PCT["alta"] / 100)]
        self.ax_mdf.set_xlim(0.5, max(x.max(), 5) + 0.5)
        self.ax_mdf.set_ylim(min(valores) - 5, max(valores) + 5)

    def mostrar(self):
        self.anim = FuncAnimation(self.fig, self.actualizar, interval=REFRESCO_MS, cache_frame_data=False)
        plt.show()


# =============================================================================
# PRINCIPAL
# =============================================================================
def main():
    parser = argparse.ArgumentParser(description="Grafica en tiempo real el procesamiento de la ESP32.")
    parser.add_argument("duracion", nargs="?", type=float, default=0,
                        help="segundos de registro (0 = hasta cerrar la ventana)")
    parser.add_argument("--sin-iniciar", action="store_true", help="no mandar START al conectar")
    parser.add_argument("--ventana", type=float, default=VENTANA_S, help="segundos visibles")
    args = parser.parse_args()

    datos = Datos()
    hilo = threading.Thread(target=hilo_ble, args=(datos, args.duracion, not args.sin_iniciar), daemon=True)
    hilo.start()
    try:
        Grafico(datos, args.ventana).mostrar()   # bloquea hasta cerrar la ventana
    except KeyboardInterrupt:
        pass
    datos.cerrar.set()
    hilo.join(timeout=10)
    print("Listo.")


if __name__ == "__main__":
    main()
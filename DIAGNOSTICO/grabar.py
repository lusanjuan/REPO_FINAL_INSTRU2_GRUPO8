"""grabar.py — Graba una prueba con la ESP y la muestra en vivo (vista liviana).

Guarda todo en  sesiones/<fecha>_<etiqueta>/ :
    senal.csv        n, t_s, mV          señal procesada por la ESP (filtrada y con recorte), 2000 Hz
    detector.csv     cada 50 ms: envolvente, límite del recorte, reposo, umbrales del detector, etapa
    resultados.csv   el JSON que recibe la app (repeticiones, estado, MDF, caída...), + hora de la PC
    marcas.csv       las marcas que pongas con la tecla M (por ejemplo "sentí fatiga")
    meta.json        etiqueta, persona, peso, ejercicio, notas, duración, paquetes perdidos
Al cerrar la ventana corre analizar.py sobre la sesión (salvo con --sin-analizar).

Teclas en la ventana:   N = NUEVA SESIÓN (serie 1, calibra)   S = START (serie siguiente, misma referencia)
                        X = STOP   M = marca   Q = terminar

Uso:
    python grabar.py --etiqueta isometrica --persona Lu --peso 4 --ejercicio "curl"
    python grabar.py --etiqueta reposo --sin-start          (no manda START al conectar)
Necesita la ESP con ENVIAR_DIAGNOSTICO = true. Cerrá la app y cualquier otro programa conectado.
"""
import argparse
import asyncio
import csv
import json
import os
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime

import numpy as np

import config as C

# =============================================================================
# Configuración de la vista
# =============================================================================
VENTANA_VISTA_S = 8          # segundos visibles
DIEZMADO_VISTA = 4           # se dibuja 1 de cada 4 muestras (el CSV guarda todas)
REFRESCO_MS = 80             # ~12 cuadros por segundo
REESCALAR_CADA_S = 1.5       # cada cuánto se ajusta el eje vertical
INFORME_ENLACE_S = 5         # cada cuánto se imprime el estado del enlace en la terminal
MAX_MARCAS_VISIBLES = 12


# =============================================================================
# Sesión: carpeta y archivos
# =============================================================================
class Sesion:
    def __init__(self, etiqueta, meta):
        fecha = datetime.now().strftime("%Y%m%d_%H%M%S")
        nombre = f"{fecha}_{etiqueta}" if etiqueta else fecha
        self.carpeta = os.path.join(C.CARPETA_SESIONES, nombre)
        os.makedirs(self.carpeta, exist_ok=True)
        self.meta = dict(meta, inicio=datetime.now().isoformat(timespec="seconds"))
        self._archivos = {}
        self.senal = self._abrir("senal.csv", ["n", "t_s", "mV"])
        self.detector = self._abrir("detector.csv", ["t_s"] + C.CAMPOS_DIAG[1:])
        self.resultados = self._abrir("resultados.csv",
                                      ["t_pc", "seq", "t", "serie", "reps", "estado", "fatiga", "mdf",
                                       "ref", "act", "rms", "cal", "rep", "sat"])
        self.marcas = self._abrir("marcas.csv", ["t_s", "tipo"])

    def _abrir(self, nombre, columnas):
        f = open(os.path.join(self.carpeta, nombre), "w", newline="", encoding="utf-8")
        self._archivos[nombre] = f
        w = csv.writer(f)
        w.writerow(columnas)
        return w

    def cerrar(self, extra):
        for f in self._archivos.values():
            f.close()
        self.meta.update(extra, fin=datetime.now().isoformat(timespec="seconds"))
        with open(os.path.join(self.carpeta, "meta.json"), "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)


# =============================================================================
# Receptor: desarma los paquetes, guarda y llena los buffers de la vista
# =============================================================================
class Receptor:
    """No depende del BLE: recibe bytes. Así también se puede alimentar desde un archivo."""

    def __init__(self, sesion):
        self.sesion = sesion
        self.lock = threading.Lock()
        n = VENTANA_VISTA_S * C.FS // DIEZMADO_VISTA
        self.vista_t = np.full(n, np.nan)          # buffers circulares diezmados
        self.vista_y = np.full(n, np.nan)
        self.vista_pos = 0
        nb = VENTANA_VISTA_S * C.FS // C.MUESTRAS_BLOQUE
        self.env_t = np.full(nb, np.nan)
        self.env_y = np.full(nb, np.nan)
        self.env_pos = 0
        self.reps_t = []                            # tiempos de pico de las repeticiones
        self.marcas_t = []
        self.ultimo = None                          # último JSON
        self.t_ultimo = 0.0                         # tiempo de la última muestra [s]
        self.bloque_anterior = None
        self.bloques = 0
        self.perdidos = 0
        self.resultados_n = 0
        self.estado_anterior = None

    # ---- paquete de diagnóstico (50 ms de señal) ----
    def al_diagnostico(self, paquete):
        if len(paquete) != struct.calcsize(C.FORMATO_DIAG):
            return
        v = struct.unpack(C.FORMATO_DIAG, bytes(paquete))
        d = dict(zip(C.CAMPOS_DIAG, v[:len(C.CAMPOS_DIAG)]))
        mv = np.asarray(v[len(C.CAMPOS_DIAG):], float) / C.ESCALA_MUESTRA
        n0 = d["bloque"] * C.MUESTRAS_BLOQUE
        n = n0 + np.arange(C.MUESTRAS_BLOQUE)
        t = n / C.FS
        t_env = (n0 + C.MUESTRAS_BLOQUE - C.ENV_VENTANA_S * C.FS / 2) / C.FS

        if self.bloque_anterior is not None and d["bloque"] > self.bloque_anterior + 1:
            self.perdidos += d["bloque"] - self.bloque_anterior - 1
        self.bloque_anterior = d["bloque"]
        self.bloques += 1

        self.sesion.senal.writerows(zip(n, np.round(t, 4), np.round(mv, 1)))
        self.sesion.detector.writerow([round(t_env, 3)] + [round(d[c], 3) if isinstance(d[c], float) else d[c]
                                                           for c in C.CAMPOS_DIAG[1:]])
        with self.lock:
            k = len(mv) // DIEZMADO_VISTA
            idx = (self.vista_pos + np.arange(k)) % len(self.vista_t)
            self.vista_t[idx] = t[::DIEZMADO_VISTA]
            self.vista_y[idx] = mv[::DIEZMADO_VISTA]
            self.vista_pos = (self.vista_pos + k) % len(self.vista_t)
            self.env_t[self.env_pos] = t_env
            self.env_y[self.env_pos] = d["envolvente"]
            self.env_pos = (self.env_pos + 1) % len(self.env_t)
            self.t_ultimo = t[-1]

    # ---- JSON de resultados ----
    def al_resultado(self, paquete):
        try:
            j = json.loads(bytes(paquete).decode("utf-8"))
        except ValueError:
            return
        self.resultados_n += 1
        self.sesion.resultados.writerow([round(time.time(), 3), j.get("seq"), j.get("t"), j.get("serie"),
                                         j.get("reps"), j.get("estado"), j.get("fatiga"), j.get("mdf"),
                                         j.get("ref"), j.get("act"), j.get("rms"), j.get("cal"),
                                         j.get("rep"), j.get("sat")])
        if j.get("rep"):
            linea = f"  rep {j['reps']:3d}   MDF {j['mdf']:5.1f} Hz   RMS {j['rms']:4.0f} mV"
            if j.get("ref", 0) > 0:
                linea += f"   caída {j['fatiga']:5.1f} %"
            print(linea + f"   [{C.TEXTO_ESTADO.get(j['estado'], j['estado'])}]")
        elif j.get("estado") != self.estado_anterior:
            print(f"  → {C.TEXTO_ESTADO.get(j.get('estado'), j.get('estado'))}")
        self.estado_anterior = j.get("estado")
        with self.lock:
            self.ultimo = j
            if j.get("rep"):
                self.reps_t.append(j["t"] / 1000)

    def marcar(self, tipo="fatiga"):
        with self.lock:
            t = self.t_ultimo
            self.marcas_t.append(t)
        self.sesion.marcas.writerow([round(t, 3), tipo])
        print(f"  ✱ marca '{tipo}' en t = {t:.1f} s")

    def estadisticas(self):
        return {"bloques_recibidos": self.bloques, "bloques_perdidos": self.perdidos,
                "resultados_recibidos": self.resultados_n}


# =============================================================================
# BLE (en su propio hilo)
# =============================================================================
class EnlaceBLE:
    def __init__(self, receptor, iniciar, duracion):
        self.receptor = receptor
        self.iniciar = iniciar
        self.duracion = duracion
        self.cerrar = threading.Event()
        self.terminado = threading.Event()
        self.estado = "Buscando la ESP..."
        self._loop = None
        self._cliente = None

    def mandar(self, cmd):
        # Una tecla mantenida apretada repite el comando: se ignora el mismo comando dentro de 1,5 s
        ahora = time.time()
        if cmd == getattr(self, "_ultimo_cmd", None) and ahora - getattr(self, "_t_ultimo_cmd", 0) < 1.5:
            return
        self._ultimo_cmd, self._t_ultimo_cmd = cmd, ahora
        if self._loop and self._cliente:
            asyncio.run_coroutine_threadsafe(self._cliente.write_gatt_char(C.UUID_COMANDO, cmd, response=True),
                                             self._loop)
            print({b"I": "  → START (misma referencia)", b"N": "  → NUEVA SESIÓN (calibra)", b"S": "  → STOP"}[cmd])

    async def _correr(self):
        from bleak import BleakClient, BleakScanner
        print(f"Buscando {C.NOMBRE_BLE}...")
        disp = await BleakScanner.find_device_by_name(C.NOMBRE_BLE, timeout=15)
        if disp is None:
            self.estado = "No se encontró la ESP"
            print("No se encontró la ESP. ¿Está prendida? ¿La app o otro programa está conectado?")
            return
        async with BleakClient(disp) as cliente:
            self._loop, self._cliente = asyncio.get_running_loop(), cliente
            self.estado = "Conectada"
            print("Conectada.  Teclas: N = nueva sesión   S = START serie   X = STOP   M = marca   Q = terminar")
            await cliente.start_notify(C.UUID_DIAGNOSTICO, lambda _, d: self.receptor.al_diagnostico(d))
            await cliente.start_notify(C.UUID_FATIGA, lambda _, d: self.receptor.al_resultado(d))
            if self.iniciar:
                self.mandar(b"N")                     # cada grabación arranca una sesión nueva
            inicio = ultimo_informe = self._loop.time()
            antes = 0
            while not self.cerrar.is_set():
                ahora = self._loop.time()
                if self.duracion and ahora - inicio >= self.duracion:
                    break
                if ahora - ultimo_informe >= INFORME_ENLACE_S:
                    e = self.receptor.estadisticas()
                    ritmo = (e["bloques_recibidos"] - antes) / (ahora - ultimo_informe)
                    aviso = "" if ritmo > 18 else "   ← llegan menos de 20 por segundo: ¿ENVIAR_DIAGNOSTICO = true?"
                    print(f"  [enlace] {ritmo:4.1f} paquetes/s   perdidos: {e['bloques_perdidos']}{aviso}")
                    antes, ultimo_informe = e["bloques_recibidos"], ahora
                await asyncio.sleep(0.2)
            await cliente.stop_notify(C.UUID_DIAGNOSTICO)
            await cliente.stop_notify(C.UUID_FATIGA)
        self.estado = "Desconectada"

    def hilo(self):
        try:
            asyncio.run(self._correr())
        except Exception as e:
            print(f"Error de BLE: {e}")
            self.estado = f"Error de BLE: {e}"
        finally:
            self.terminado.set()


# =============================================================================
# Vista liviana (blitting: solo se redibuja lo que cambia)
# =============================================================================
def mostrar(receptor, enlace):
    import matplotlib
    for k in ("keymap.save", "keymap.quit", "keymap.xscale", "keymap.yscale", "keymap.fullscreen"):
        matplotlib.rcParams[k] = []
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    C.aplicar_estilo()

    fig = plt.figure(figsize=(12, 5.6))
    fig.canvas.manager.set_window_title("MyoSignals · grabación")
    ax = fig.add_axes([0.06, 0.12, 0.92, 0.66])
    ax.set_xlim(-VENTANA_VISTA_S, 0)
    ax.set_ylim(-300, 300)
    ax.set_xlabel("segundos (0 = ahora)")
    ax.set_ylabel("mV")
    (l_senal,) = ax.plot([], [], lw=0.6, color=C.GRIS, animated=True)
    (l_env,) = ax.plot([], [], lw=2.4, color=C.LIMA, animated=True)
    reps = [ax.axvline(np.nan, color=C.NARANJA, lw=1.6, animated=True) for _ in range(MAX_MARCAS_VISIBLES)]
    marcas = [ax.axvline(np.nan, color=C.CLARO, lw=1.4, ls="--", animated=True) for _ in range(MAX_MARCAS_VISIBLES)]
    fuerte = C.titulo_fuerte()
    # Los textos que cambian van en un eje propio (invisible): el blitting solo redibuja ejes
    cabecera = fig.add_axes([0.06, 0.8, 0.92, 0.12])
    cabecera.set_axis_off()
    t_estado = cabecera.text(0, 0.55, "", fontsize=30, color=C.GRIS, animated=True,
                             transform=cabecera.transAxes, **fuerte)
    t_datos = cabecera.text(0, 0.05, "", fontsize=13, color=C.CLARO, animated=True, transform=cabecera.transAxes)
    fig.text(0.98, 0.94, "N sesión nueva   S serie   X STOP   M marca   Q terminar", ha="right", fontsize=10, color=C.GRIS)
    ultimo_reescalado = [0.0]

    def tecla(ev):
        k = (ev.key or "").lower()
        if k == "n":
            enlace.mandar(b"N")
        elif k == "s":
            enlace.mandar(b"I")
        elif k == "x":
            enlace.mandar(b"S")
        elif k == "m":
            receptor.marcar("fatiga")
        elif k == "q":
            plt.close(fig)
    fig.canvas.mpl_connect("key_press_event", tecla)

    def cuadro(_):
        with receptor.lock:
            ahora = receptor.t_ultimo
            vt, vy = receptor.vista_t.copy(), receptor.vista_y.copy()
            et, ey = receptor.env_t.copy(), receptor.env_y.copy()
            rt, mt = list(receptor.reps_t[-MAX_MARCAS_VISIBLES:]), list(receptor.marcas_t[-MAX_MARCAS_VISIBLES:])
            j = receptor.ultimo
        orden = np.argsort(vt)
        l_senal.set_data(vt[orden] - ahora, vy[orden])
        orden_e = np.argsort(et)
        l_env.set_data(et[orden_e] - ahora, ey[orden_e])
        for linea, lista in ((reps, rt), (marcas, mt)):
            for i, l in enumerate(linea):
                x = lista[i] - ahora if i < len(lista) else np.nan
                l.set_xdata([x, x])
        if j:
            t_estado.set_text(C.TEXTO_ESTADO.get(j["estado"], j["estado"]))
            t_estado.set_color(C.COLOR_ESTADO.get(j["estado"], C.GRIS))
            texto = f"{j['reps']} repeticiones   ·   serie {j['serie']}"
            if j.get("ref", 0) > 0:
                texto += f"   ·   referencia {j['ref']:.1f} Hz   ·   caída {j['fatiga']:.1f} %"
        else:
            t_estado.set_text(enlace.estado.upper())
            texto = ""
        e = receptor.estadisticas()
        t_datos.set_text(texto + f"      perdidos: {e['bloques_perdidos']}")
        # Reescalar el eje vertical de vez en cuando (redibujo completo)
        if time.time() - ultimo_reescalado[0] > REESCALAR_CADA_S and np.isfinite(vy).any():
            alto = np.nanpercentile(np.abs(vy), 99.5) * 1.15 + 5
            if abs(alto - ax.get_ylim()[1]) / ax.get_ylim()[1] > 0.15:
                ax.set_ylim(-alto, alto)
                fig.canvas.draw_idle()
            ultimo_reescalado[0] = time.time()
        return [l_senal, l_env, t_estado, t_datos, *reps, *marcas]

    anim = FuncAnimation(fig, cuadro, interval=REFRESCO_MS, blit=True, cache_frame_data=False)
    plt.show()
    return anim


# =============================================================================
# Principal
# =============================================================================
def main():
    p = argparse.ArgumentParser(description="Graba una prueba con la ESP y la muestra en vivo.")
    p.add_argument("--etiqueta", default="", help="nombre corto de la prueba (va en la carpeta)")
    p.add_argument("--persona", default="")
    p.add_argument("--peso", default="", help="peso usado [kg]")
    p.add_argument("--ejercicio", default="curl de bíceps")
    p.add_argument("--notas", default="")
    p.add_argument("--duracion", type=float, default=0, help="segundos (0 = hasta cerrar la ventana)")
    p.add_argument("--sin-start", action="store_true", help="no mandar NUEVA SESIÓN al conectar")
    p.add_argument("--sin-analizar", action="store_true", help="no correr analizar.py al terminar")
    a = p.parse_args()

    sesion = Sesion(a.etiqueta.strip().replace(" ", "_"),
                    {"etiqueta": a.etiqueta, "persona": a.persona, "peso_kg": a.peso,
                     "ejercicio": a.ejercicio, "notas": a.notas})
    print(f"Sesión: {sesion.carpeta}")
    receptor = Receptor(sesion)
    enlace = EnlaceBLE(receptor, iniciar=not a.sin_start, duracion=a.duracion)
    hilo = threading.Thread(target=enlace.hilo, daemon=True)
    hilo.start()
    try:
        mostrar(receptor, enlace)
    except KeyboardInterrupt:
        pass
    enlace.cerrar.set()
    hilo.join(timeout=10)
    e = receptor.estadisticas()
    sesion.cerrar(dict(e, duracion_s=round(receptor.t_ultimo, 1)))
    print(f"\nGuardado en {sesion.carpeta}")
    print(f"  {e['bloques_recibidos']} paquetes de señal ({e['bloques_perdidos']} perdidos), "
          f"{e['resultados_recibidos']} resultados")
    if not a.sin_analizar and e["bloques_recibidos"] > 0:
        subprocess.run([sys.executable, os.path.join(C.CARPETA, "analizar.py"), sesion.carpeta])


if __name__ == "__main__":
    main()
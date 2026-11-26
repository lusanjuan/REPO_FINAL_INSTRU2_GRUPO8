"""analizar.py — Análisis offline de una sesión grabada con grabar.py.

Recalcula todo en la PC, con la señal completa, independientemente de la ESP:
  · enlace: cuánta señal llegó y cuántos huecos hubo
  · ruido de base, relación señal/ruido y cuánto actuó el recorte de picos
  · repeticiones: las de la ESP y un conteo independiente sobre la envolvente
  · MDF, MNF y RMS de cada repetición (Welch, ventana de 1,5 s centrada en el pico)
    y la comparación con la MDF que calculó la ESP
  · tendencia de MDF / MNF / RMS a lo largo de la serie (recta, r, p, caída %)
  · MDF y MNF en ventanas deslizantes de 1 s (sirve para las pruebas isométricas)

Uso:
    python analizar.py ultima                 (la última sesión grabada)
    python analizar.py sesiones/20261127_101500_isometrica
Deja todo en  <sesion>/analisis/ :
    resumen.txt / resumen.json   números y conclusiones
    reps.csv                     una fila por repetición
    ventanas.csv                 MDF/MNF cada 0,25 s
    senal.png  mdf_reps.png  espectros.png  validacion_esp.png  diapo.png
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy import signal, stats
from scipy.ndimage import uniform_filter1d

import config as C
import metodos

# =============================================================================
# Configuración del análisis
# =============================================================================
ACTIVIDAD_FACTOR = 3.0        # una ventana es "activa" si su envolvente supera 3 × el ruido de base
PICOS_SEPARACION_S = 2.0      # conteo independiente: igual que REP_SEPARACION_MIN_S de la ESP
PICOS_FACTOR_REPOSO = 2.0     # el pico tiene que superar 2 × el reposo (igual que la ESP)
PICOS_PROMINENCIA = 0.30      # y sobresalir 30 % de su altura (igual que REP_CAIDA_PICO)
COINCIDENCIA_S = 0.6          # dos picos (ESP y PC) son el mismo si están a menos de 0,6 s
P_SIGNIFICATIVO = 0.05
CRUDA_SI_MEDIANA_MAYOR_A_MV = 50   # si la señal no está centrada en 0, es la cruda del ADC: se filtra en la PC
SATURACION_BAJA_MV = 10            # muestras crudas por debajo de esto = el ADC tocó el piso
# Entrada cruda (CSV): mismos criterios que los scripts del grupo (analisis_repeticiones.py)
BANDA_CRUDA_HZ = C.BANDA_HZ        # filtro y banda de MDF/MNF para la señal cruda (= la ESP)
DESCARTAR_INCOMPLETAS = 0.7        # descarta reps con RMS < 70 % de la mediana (acomodar el brazo, reps a medias)
EXCLUIR_POST_MARCA = True          # las reps después de la marca de fatiga del usuario no entran en la tendencia
BANDA = list(C.BANDA_HZ)           # banda en uso (se cambia a BANDA_CRUDA_HZ si la entrada es cruda)
DPI = 110


# =============================================================================
# Carga
# =============================================================================
def resolver_carpeta(arg):
    if arg in ("ultima", "última", "latest"):
        sesiones = sorted(d for d in glob.glob(os.path.join(C.CARPETA_SESIONES, "*"))
                          if os.path.exists(os.path.join(d, "senal.csv")))
        if not sesiones:
            sys.exit("No hay sesiones grabadas en " + C.CARPETA_SESIONES)
        return sesiones[-1]
    return arg.rstrip("/\\")


def leer_csv(carpeta, nombre):
    ruta = os.path.join(carpeta, nombre)
    if not os.path.exists(ruta):
        return pd.DataFrame()
    try:
        return pd.read_csv(ruta)
    except pd.errors.EmptyDataError:
        return pd.DataFrame()


def cargar(ruta):
    """Acepta una carpeta de grabar.py o un CSV suelto (columnas muestra, t_s, mV y opcionales usuario, sexo, fatiga)."""
    BANDA[:] = C.BANDA_HZ                                       # se vuelve a fijar en cada archivo
    if os.path.isfile(ruta):
        carpeta = os.path.splitext(ruta)[0]                     # los resultados van a una carpeta con el mismo nombre
        os.makedirs(carpeta, exist_ok=True)
        senal = pd.read_csv(ruta)
        meta = {"archivo": os.path.basename(ruta)}
        detector = resultados = pd.DataFrame()
        marcas = pd.DataFrame(columns=["t_s", "tipo"])
    else:
        carpeta = ruta
        meta_ruta = os.path.join(carpeta, "meta.json")
        meta = json.load(open(meta_ruta, encoding="utf-8")) if os.path.exists(meta_ruta) else {}
        senal = leer_csv(carpeta, "senal.csv")
        if senal.empty:                                         # carpeta con un CSV de otro nombre
            otros = [f for f in sorted(glob.glob(os.path.join(carpeta, "*.csv")))
                     if os.path.basename(f) not in ("detector.csv", "resultados.csv", "marcas.csv")]
            if otros:
                senal = pd.read_csv(otros[0])
                print(f"Usando {os.path.basename(otros[0])}")
        detector, resultados, marcas = (leer_csv(carpeta, n) for n in ("detector.csv", "resultados.csv", "marcas.csv"))
    if senal.empty:
        sys.exit("La sesión no tiene señal.")
    # Formatos aceptados: n o muestra (índice), t_s, mV; opcionales usuario, sexo, modo, fatiga
    senal.columns = [str(c).strip() for c in senal.columns]
    senal = senal.rename(columns={"muestra": "n"})
    if "n" not in senal:
        senal["n"] = np.round(senal.t_s * C.FS).astype(int)
    for col, clave in (("usuario", "persona"), ("sexo", "sexo"), ("modo", "modo")):
        if col in senal and not meta.get(clave):
            meta[clave] = str(senal[col].iloc[0])
    senal = senal.drop_duplicates("n").sort_values("n")
    s = {"carpeta": carpeta, "nombre": os.path.basename(carpeta), "meta": meta,
         "detector": detector, "resultados": resultados, "marcas": marcas}
    # Señal continua: las muestras que faltan quedan como NaN
    n0, n1 = int(senal.n.iloc[0]), int(senal.n.iloc[-1])
    y = np.full(n1 - n0 + 1, np.nan)
    y[senal.n.values - n0] = senal.mV.values
    s["n0"] = n0
    s["t"] = (n0 + np.arange(len(y))) / C.FS
    s["t0"] = s["t"][0]
    # Fatiga indicada por el usuario (columna 0/1): franjas y marcas donde empieza
    s["fatiga_usuario"] = None
    if "fatiga" in senal:
        f = np.zeros(len(y))
        f[senal.n.values - n0] = senal.fatiga.values
        s["fatiga_usuario"] = f > 0
        inicios = np.where(np.diff(np.r_[0, (f > 0).astype(int)]) == 1)[0]
        if len(inicios):
            s["marcas"] = pd.DataFrame({"t_s": s["t"][inicios], "tipo": "fatiga (usuario)"})
    s["y"], s["cruda"], s["saturacion"] = acondicionar(y)
    s["y_cruda"] = y if s["cruda"] else None
    return s


def acondicionar(y):
    """Si la señal es la cruda del ADC (centrada en ~650 mV), le aplica los filtros de la ESP:
    pasaaltos y pasabajos Butterworth de orden 4 (30 y 450 Hz) y notch de 50 Hz, sin desfase."""
    mediana = float(np.nanmedian(y))
    if abs(mediana) < CRUDA_SI_MEDIANA_MAYOR_A_MV:
        return y, False, None
    ok = np.isfinite(y)
    saturacion = {"mediana_cruda_mV": round(mediana, 1),
                  "min_mV": round(float(np.nanmin(y)), 1), "max_mV": round(float(np.nanmax(y)), 1),
                  "piso_pct": round(100 * float(np.mean(y[ok] <= SATURACION_BAJA_MV)), 3),
                  "techo_pct": round(100 * float(np.mean(y[ok] >= np.nanmax(y) - 2)), 3)}
    x = np.where(ok, y, mediana) - mediana
    BANDA[:] = BANDA_CRUDA_HZ
    for sos in (signal.butter(4, BANDA[0], "highpass", fs=C.FS, output="sos"),
                signal.butter(4, BANDA[1], "lowpass", fs=C.FS, output="sos"),
                *[signal.tf2sos(*signal.iirnotch(50 * k, 10 * k, fs=C.FS))     # 50 Hz y armónicos,
                  for k in range(1, C.NOTCH_ARMONICOS + 1) if 50 * k < BANDA[1]]):  # como la ESP
        x = signal.sosfiltfilt(sos, x)
    x[~ok] = np.nan
    return x, True, saturacion


# =============================================================================
# Cálculos básicos
# =============================================================================
def envolvente_rms(y, ventana_s=C.ENV_VENTANA_S):
    """RMS centrada (como la de los scripts del grupo). Los huecos cuentan como 0."""
    y0 = np.nan_to_num(y)
    return np.sqrt(uniform_filter1d(y0 ** 2, int(ventana_s * C.FS), mode="nearest"))


def espectro(x):
    """PSD de Welch (Hann, 512, 50 %) y MDF / MNF en la banda 30–450 Hz (igual que la ESP)."""
    x = np.asarray(x, float)
    if len(x) < C.NFFT or np.isnan(x).any():
        return None
    f, p = signal.welch(x - x.mean(), C.FS, window="hann", nperseg=C.NFFT, noverlap=C.NFFT // 2)
    banda = (f >= BANDA[0]) & (f <= BANDA[1])
    fb, pb = f[banda], p[banda]
    acumulada = np.cumsum(pb)
    if acumulada[-1] <= 0:
        return None
    mdf = float(np.interp(acumulada[-1] / 2, acumulada, fb))
    mnf = float(np.sum(fb * pb) / np.sum(pb))
    return {"f": f, "p": p, "mdf": mdf, "mnf": mnf, "rms": float(np.sqrt(np.mean(x ** 2)))}


def tramo(s, t_centro, largo_s):
    i = int(round((t_centro - s["t0"]) * C.FS))
    m = int(largo_s * C.FS) // 2
    if i - m < 0 or i + m > len(s["y"]):
        return None
    return s["y"][i - m:i + m]


def recta(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    ok = np.isfinite(x) & np.isfinite(y)
    if ok.sum() < 3:
        return None
    r = stats.linregress(x[ok], y[ok])
    return {"pendiente": r.slope, "ordenada": r.intercept, "r": r.rvalue, "p": r.pvalue}


def nivel_por_caida(caida):
    if caida is None or not np.isfinite(caida):
        return "—"
    if caida >= C.NIVELES_PCT["alta"]:
        return "alta"
    if caida >= C.NIVELES_PCT["moderada"]:
        return "moderada"
    if caida >= C.NIVELES_PCT["leve"]:
        return "leve"
    return "sin"


# =============================================================================
# Partes del análisis
# =============================================================================
def analizar_enlace(s):
    faltan = np.isnan(s["y"])
    bordes = np.diff(np.r_[0, faltan.astype(int), 0])
    return {"duracion_s": round(len(s["y"]) / C.FS, 1),
            "muestras_recibidas_pct": round(100 * (1 - faltan.mean()), 2),
            "huecos": int((bordes == 1).sum()),
            "hueco_mas_largo_ms": round(1000 * max([0] + list(np.where(bordes == -1)[0] - np.where(bordes == 1)[0])) / C.FS, 0)}


def analizar_ruido(s, env):
    base = float(np.percentile(env, 10))
    alto = float(np.percentile(env, 95))
    det = s["detector"]
    reposo_esp = float(det.reposo[det.reposo > 0].median()) if not det.empty and (det.reposo > 0).any() else None
    # Recorte: cuántas muestras quedaron pegadas al límite que usó la ESP
    recortado_pct = None
    if not det.empty and (det.limite_recorte > 0).any():
        lim = np.interp(s["t"], det.t_s.values + C.ENV_VENTANA_S / 2, det.limite_recorte.values)
        y = s["y"]
        ok = np.isfinite(y) & (lim > 0)
        recortado_pct = round(100 * float(np.mean(np.abs(y[ok]) >= 0.98 * lim[ok])), 3) if ok.any() else None
    return {"ruido_base_mV": round(base, 1), "envolvente_p95_mV": round(alto, 1),
            "snr_dB": round(20 * np.log10(alto / base), 1) if base > 0 else None,
            "reposo_esp_mV": round(reposo_esp, 1) if reposo_esp else None,
            "recortado_pct": recortado_pct, "entrada_cruda": s["cruda"], "saturacion": s["saturacion"]}


def reps_esp(s):
    r = s["resultados"]
    if r.empty or "rep" not in r:
        return pd.DataFrame(columns=["t", "serie", "reps", "mdf", "rms", "estado", "fatiga"])
    r = r[r.rep == 1].drop_duplicates("seq").copy()
    r["t"] = r["t"] / 1000.0
    return r[["t", "serie", "reps", "mdf", "rms", "estado", "fatiga", "ref"]].reset_index(drop=True)


ESTADOS_CONTANDO = ("calibrando", "sin", "leve", "moderada", "alta", "sinref")


def contar_picos(s, env, reposo, estados):
    """Conteo independiente, solo en el tramo en que la ESP estaba contando (si se sabe)."""
    paso = C.MUESTRAS_BLOQUE // 2
    e = env[::paso]                                   # una muestra cada 25 ms alcanza
    alto = PICOS_FACTOR_REPOSO * reposo
    picos, prop = signal.find_peaks(e, height=alto, distance=int(PICOS_SEPARACION_S * C.FS / paso))
    if len(picos):
        prom = signal.peak_prominences(e, picos)[0]
        picos = picos[prom >= PICOS_PROMINENCIA * e[picos]]
    t = s["t0"] + picos * paso / C.FS
    contando = estados[estados.estado.isin(ESTADOS_CONTANDO)]
    if len(contando):
        desde, hasta = contando.t.iloc[0] + s["t0"] - 1.0, contando.t.iloc[-1] + s["t0"] + 1.0
        t = t[(t >= desde) & (t <= hasta)]
    return t


def analizar_reps(s, esp):
    filas = []
    for _, r in esp.iterrows():
        x = tramo(s, r.t, C.VENTANA_REP_S)
        e = espectro(x) if x is not None else None
        filas.append({"serie": int(r.serie), "rep": int(r.reps), "t_s": round(r.t - s["t0"], 3),
                      "mdf_esp": r.mdf, "rms_esp": r.rms, "estado_esp": r.estado, "caida_esp": r.fatiga,
                      "mdf": e["mdf"] if e else np.nan, "mnf": e["mnf"] if e else np.nan,
                      "rms": e["rms"] if e else np.nan, "_psd": e["p"] if e else None})
    return pd.DataFrame(filas)


def depurar_reps(s, reps, excluir_post_marca, reposo):
    """Como la ESP y los scripts del grupo: saca de la tendencia las reps incompletas
    (y, en la entrada cruda, las posteriores a la marca de fatiga) y renumera."""
    if reps.empty:
        return reps, {}
    rms_min = DESCARTAR_INCOMPLETAS * reps.rms.median()
    # Ruidosas (como la ESP): RMS < VALIDA_SNR_MIN × reposo
    ruidosas = (reps.rms < metodos.VALIDA_SNR_MIN * reposo) if metodos.VALIDA_SNR_MIN > 0 and reposo > 0 \
        else pd.Series(False, index=reps.index)
    incompletas = (reps.rms < rms_min) & ~ruidosas
    post = pd.Series(False, index=reps.index)
    if excluir_post_marca and not s["marcas"].empty:
        post = reps.t_s > (s["marcas"].t_s.iloc[0] - s["t0"])
    quedan = reps[~incompletas & ~ruidosas & ~post].copy()
    quedan["rep"] = np.arange(1, len(quedan) + 1)
    return quedan.reset_index(drop=True), {"detectadas": int(len(reps)), "incompletas": int(incompletas.sum()),
                                           "ruidosas": int(ruidosas.sum()),
                                           "post_marca": int((post & ~incompletas & ~ruidosas).sum()),
                                           "usadas": int(len(quedan))}


def tendencia(reps, columna):
    """Referencia = mediana de las primeras 3; caída = cuánto bajó la recta al final, en % de la referencia."""
    v = reps[columna].values.astype(float)
    if np.isfinite(v).sum() < C.REPS_REFERENCIA + 1:
        return None
    ref = float(np.nanmedian(v[:C.REPS_REFERENCIA]))
    i = np.arange(1, len(v) + 1)
    ajuste = recta(i, v)
    if ajuste is None or ref == 0:
        return None
    final = ajuste["ordenada"] + ajuste["pendiente"] * i[-1]
    return {"ref": round(ref, 2), "pendiente_por_rep": round(ajuste["pendiente"], 3),
            "r": round(ajuste["r"], 3), "p": float(f"{ajuste['p']:.3g}"),
            "significativa": bool(ajuste["p"] < P_SIGNIFICATIVO),
            "caida_pct": round(100 * (ref - final) / ref, 1), "cambio_pct": round(100 * (final - ref) / ref, 1),
            "ordenada": ajuste["ordenada"], "pendiente": ajuste["pendiente"]}


def ventanas_deslizantes(s, env, umbral):
    largo = int(C.VENTANA_DESLIZANTE_S * C.FS)
    paso = int(C.PASO_DESLIZANTE_S * C.FS)
    filas = []
    for i in range(0, len(s["y"]) - largo, paso):
        activa = env[i + largo // 2] > umbral
        e = espectro(s["y"][i:i + largo]) if activa else None
        filas.append({"t_s": round((i + largo / 2) / C.FS, 3), "activa": bool(activa),
                      "mdf": e["mdf"] if e else np.nan, "mnf": e["mnf"] if e else np.nan,
                      "rms": e["rms"] if e else np.nan})
    return pd.DataFrame(filas)


def tendencia_temporal(v):
    a = v[v.activa & v.mdf.notna()]
    if len(a) < 8:
        return None
    aj = recta(a.t_s, a.mdf)
    ini = aj["ordenada"] + aj["pendiente"] * a.t_s.iloc[0]
    fin = aj["ordenada"] + aj["pendiente"] * a.t_s.iloc[-1]
    return {"pendiente_hz_por_s": round(aj["pendiente"], 4), "r": round(aj["r"], 3),
            "p": float(f"{aj['p']:.3g}"), "mdf_inicio": round(ini, 1), "mdf_final": round(fin, 1),
            "caida_pct": round(100 * (ini - fin) / ini, 1), "segundos_activos": round(len(a) * C.PASO_DESLIZANTE_S, 1)}


def estados_esp(s):
    r = s["resultados"]
    if r.empty:
        return pd.DataFrame(columns=["t", "estado"])
    r = r.drop_duplicates("seq").sort_values("t")
    return pd.DataFrame({"t": r.t / 1000.0 - s["t0"], "estado": r.estado})


# =============================================================================
# Conclusiones (frases con los números, sin interpretar de más)
# =============================================================================
def conclusiones(res):
    c = []
    e = res["enlace"]
    c.append(f"Enlace: llegó el {e['muestras_recibidas_pct']} % de la señal ({e['huecos']} huecos, "
             f"el más largo de {e['hueco_mas_largo_ms']:.0f} ms) en {e['duracion_s']} s.")
    r = res["ruido"]
    c.append(f"Ruido de base (envolvente, percentil 10): {r['ruido_base_mV']} mV; "
             f"contracción (percentil 95): {r['envolvente_p95_mV']} mV; SNR {r['snr_dB']} dB.")
    if r.get("entrada_cruda"):
        sa = r["saturacion"]
        c.append(f"Entrada cruda del ADC (mediana {sa['mediana_cruda_mV']} mV, rango {sa['min_mV']}–{sa['max_mV']} mV): "
                 f"se filtró en la PC (pasabanda {BANDA[0]}–{BANDA[1]} Hz + notch 50 Hz y armónicos, como la ESP). Muestras en el piso (≤ {SATURACION_BAJA_MV} mV): {sa['piso_pct']} %; "
                 f"en el máximo: {sa['techo_pct']} %.")
        if sa["piso_pct"] > 0.05:
            c.append("   ⚠ La señal toca 0 mV: el ADC está saturando abajo. Eso deforma la señal y puede subir la MDF.")
    if r["recortado_pct"] is not None:
        c.append(f"El recorte de picos tocó el {r['recortado_pct']} % de las muestras.")
    k = res["conteo"]
    if k["esp"] is None:
        c.append(f"Repeticiones contadas en la PC: {k['pc']} (no hay datos de la ESP para comparar).")
    else:
        c.append(f"Repeticiones: la ESP contó {k['esp']}, el conteo independiente {k['pc']} "
                 f"({k['coinciden']} coinciden a menos de {COINCIDENCIA_S} s).")
    dp = res.get("depuracion") or {}
    if dp:
        c.append(f"   Para la tendencia se usan {dp['usadas']}: se descartaron {dp['incompletas']} incompletas "
                 f"(RMS < {DESCARTAR_INCOMPLETAS:.0%} de la mediana), {dp['ruidosas']} ruidosas "
                 f"(RMS < {metodos.VALIDA_SNR_MIN:g} × reposo) y {dp['post_marca']} posteriores a la marca de fatiga. "
                 f"Banda {BANDA[0]}–{BANDA[1]} Hz.")
        if dp["detectadas"] and dp["ruidosas"] > dp["detectadas"] / 2:
            c.append("   ⚠ Más de la mitad de las repeticiones son ruidosas: revisar electrodos, cables y ruido de la electrónica.")
    if res.get("validacion"):
        v = res["validacion"]
        c.append(f"MDF ESP vs PC: diferencia media {v['mae_hz']} Hz (máxima {v['max_hz']} Hz) en {v['n']} repeticiones.")
    for serie in res["series"]:
        pre = f"Serie {serie['serie']} ({serie['reps']} reps): "
        m = serie.get("mdf")
        if m:
            sig = "significativa" if m["significativa"] else "NO significativa"
            c.append(pre + f"MDF {m['ref']} Hz → pendiente {m['pendiente_por_rep']} Hz/rep (r = {m['r']}, "
                           f"p = {m['p']}, {sig}); caída estimada {m['caida_pct']} % → nivel '{nivel_por_caida(m['caida_pct'])}'. "
                           + (f"La ESP terminó en '{serie['estado_final_esp']}' ({serie['caida_final_esp']} %)." if serie['estado_final_esp'] != "—" else ""))
        else:
            c.append(pre + "pocas repeticiones para calcular una tendencia.")
        if serie.get("mnf"):
            c.append(f"   MNF: caída {serie['mnf']['caida_pct']} % (p = {serie['mnf']['p']}).  "
                     f"RMS: cambio {serie['rms']['cambio_pct']:+} % (p = {serie['rms']['p']}).")
    if res.get("isometrica"):
        t = res["isometrica"]
        c.append(f"Ventanas de 1 s ({t['segundos_activos']} s con contracción): MDF {t['mdf_inicio']} → {t['mdf_final']} Hz, "
                 f"caída {t['caida_pct']} %, pendiente {t['pendiente_hz_por_s']} Hz/s (r = {t['r']}, p = {t['p']}).")
    for m in res["marcas"]:
        c.append(f"Marca '{m['tipo']}' en {m['t_s']} s: cerca de la rep {m['rep_cercana']}"
                 + (f"; la ESP decía '{m['estado_esp']}'." if m['estado_esp'] != "—" else "."))
    return c


# =============================================================================
# Gráficos
# =============================================================================
def guardar(fig, carpeta, nombre):
    import matplotlib.pyplot as plt
    fig.savefig(os.path.join(carpeta, nombre), dpi=DPI)
    plt.close(fig)


def graficar_senal(s, env, reps, picos_pc, estados, ventanas, salida):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch
    t = s["t"] - s["t0"]
    d = 4
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(15, 7.5), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    nombre_senal = "señal filtrada en la PC (entrada cruda)" if s["cruda"] else "señal procesada (ESP)"
    a1.plot(t[::d], s["y"][::d], lw=0.4, color=C.GRIS, alpha=0.8, label=nombre_senal)
    if s["fatiga_usuario"] is not None and s["fatiga_usuario"].any():
        a1.fill_between(t[::d], 0, 1, where=s["fatiga_usuario"][::d], transform=a1.get_xaxis_transform(),
                        color=C.NARANJA, alpha=0.12, lw=0, label="fatiga indicada por el usuario")
    a1.plot(t[::d], env[::d], lw=2, color=C.LIMA, label="envolvente RMS 0,5 s (PC)")
    det = s["detector"]
    if not det.empty:
        a1.plot(det.t_s - s["t0"], det.envolvente, lw=1, ls="--", color=C.CLARO, alpha=0.7, label="envolvente ESP")
    for x in reps.t_s:
        a1.axvline(x, color=C.NARANJA, lw=1.2, alpha=0.9)
    a1.plot(picos_pc - s["t0"], np.interp(picos_pc - s["t0"], t, env), "v", color=C.CLARO, ms=7, label="pico PC")
    # Franja de estados de la ESP
    if len(estados):
        tope = np.nanpercentile(np.abs(s["y"]), 99.7)
        a1.set_ylim(-tope * 1.1, tope * 1.25)
        tt = list(estados.t) + [t[-1]]
        for i, e in enumerate(estados.estado):
            a1.axvspan(tt[i], tt[i + 1], ymin=0.955, ymax=1, color=C.COLOR_ESTADO.get(e, C.GRIS), lw=0)
    for _, m in s["marcas"].iterrows() if not s["marcas"].empty else []:
        a1.axvline(m.t_s - s["t0"], color=C.CLARO, lw=1.5, ls=":")
        a1.text(m.t_s - s["t0"], a1.get_ylim()[0] * 0.92, " marca: " + str(m.tipo), color=C.CLARO, fontsize=9)
    a1.set_ylabel("mV")
    a1.set_title(f"Señal y repeticiones · {s['nombre']}")
    manejadores, _ = a1.get_legend_handles_labels()
    manejadores += [plt.Line2D([], [], color=C.NARANJA, lw=1.5, label="repetición" + ("" if s["cruda"] else " (ESP)"))]
    usados = list(dict.fromkeys(estados.estado)) if len(estados) else []
    manejadores += [Patch(color=C.COLOR_ESTADO.get(e, C.GRIS), label=C.TEXTO_ESTADO.get(e, e).lower()) for e in usados]
    a1.legend(handles=manejadores, loc="upper left", ncol=4, fontsize=8)
    a = ventanas[ventanas.activa]
    a2.plot(a.t_s, a.mdf, ".", color=C.GRIS, ms=3, label="MDF ventana 1 s")
    a2.plot(a.t_s, a.mnf, ".", color=C.AMARILLO, alpha=0.5, ms=3, label="MNF ventana 1 s")
    a2.plot(reps.t_s, reps.mdf, "o", color=C.LIMA, ms=6, label="MDF por rep (PC)")
    a2.plot(reps.t_s, reps.mdf_esp, "x", color=C.NARANJA, ms=6, label="MDF por rep (ESP)")
    a2.set_ylabel("Hz")
    a2.set_xlabel("segundos")
    a2.legend(loc="upper right", ncol=4, fontsize=8)
    fig.tight_layout()
    guardar(fig, salida, "senal.png")


def graficar_reps(reps, series, salida):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    fig, ejes = plt.subplots(1, 3, figsize=(15, 4.8))
    for ax, col, titulo in zip(ejes, ["mdf", "mnf", "rms"], ["MDF por repetición", "MNF por repetición", "RMS por repetición"]):
        for serie in series:
            r = reps[reps.serie == serie["serie"]]
            i = np.arange(1, len(r) + 1)
            ax.plot(i, r[col], "o-", color=C.LIMA, lw=1, ms=6, label=f"serie {serie['serie']} (PC)")
            if col == "mdf":
                ax.plot(i, r.mdf_esp, "x", color=C.NARANJA, ms=7, label="ESP")
            t = serie.get(col)
            if t:
                ax.plot(i, t["ordenada"] + t["pendiente"] * i, color=C.CLARO, lw=1.5, ls="--",
                        label=f"recta  p = {t['p']:.2g}")
                ax.axhline(t["ref"], color=C.GRIS_OSCURO, lw=1)
                if col == "mdf":
                    for nivel, color in (("leve", C.LIMA), ("moderada", C.AMARILLO), ("alta", C.NARANJA)):
                        ax.axhline(t["ref"] * (1 - C.NIVELES_PCT[nivel] / 100), color=color, lw=0.8, ls=":")
        ax.set_title(titulo)
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_xlabel("repetición")
        ax.set_ylabel("mV" if col == "rms" else "Hz")
        ax.legend(fontsize=8)
    fig.tight_layout()
    guardar(fig, salida, "mdf_reps.png")


def graficar_espectros(reps, salida):
    import matplotlib.pyplot as plt
    validas = reps[reps._psd.notna()]
    if len(validas) < 2:
        return
    f = np.fft.rfftfreq(C.NFFT, 1 / C.FS)
    k = min(3, len(validas) // 2) or 1
    fig, ax = plt.subplots(figsize=(10, 5))
    for grupo, color, nombre in ((validas.head(k), C.GRIS, f"primeras {k}"), (validas.tail(k), C.LIMA, f"últimas {k}")):
        p = np.mean(np.vstack(grupo._psd.values), axis=0)
        banda = (f >= BANDA[0]) & (f <= BANDA[1])
        p = p / p[banda].max()
        ax.plot(f, p, color=color, lw=2, label=f"{nombre} reps  (MDF {grupo.mdf.mean():.1f} Hz)")
        ax.axvline(grupo.mdf.mean(), color=color, lw=1, ls="--")
    ax.set_xlim(0, 500)
    ax.set_xlabel("Hz")
    ax.set_ylabel("potencia (normalizada)")
    ax.set_title("Espectro: primeras vs últimas repeticiones")
    ax.legend()
    fig.tight_layout()
    guardar(fig, salida, "espectros.png")


def graficar_validacion(reps, conteo, salida):
    import matplotlib.pyplot as plt
    v = reps.dropna(subset=["mdf", "mdf_esp"])
    if v.empty:
        return
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.8))
    lo, hi = min(v.mdf.min(), v.mdf_esp.min()) - 3, max(v.mdf.max(), v.mdf_esp.max()) + 3
    a1.plot([lo, hi], [lo, hi], color=C.GRIS_OSCURO, lw=1)
    a1.plot(v.mdf, v.mdf_esp, "o", color=C.LIMA)
    a1.set_xlabel("MDF PC [Hz]")
    a1.set_ylabel("MDF ESP [Hz]")
    a1.set_title("¿La ESP calcula lo mismo que la PC?")
    a2.bar(["ESP", "PC", "coinciden"], [conteo["esp"], conteo["pc"], conteo["coinciden"]],
           color=[C.NARANJA, C.LIMA, C.GRIS])
    a2.set_title("Repeticiones contadas")
    a2.grid(axis="x", visible=False)
    fig.tight_layout()
    guardar(fig, salida, "validacion_esp.png")


def graficar_diapo(s, reps, serie, salida):
    """Una figura limpia, 1920×1080, para pegar en la presentación."""
    import matplotlib.pyplot as plt
    t = serie.get("mdf") if serie else None
    if t is None:
        return
    r = reps[reps.serie == serie["serie"]]
    i = np.arange(1, len(r) + 1)
    pct = 100 * r.mdf.values / t["ref"]
    fig = plt.figure(figsize=(19.2, 10.8))
    ax = fig.add_axes([0.07, 0.12, 0.88, 0.62])
    abajo = min(np.nanmin(pct), 100 - C.NIVELES_PCT["alta"]) - 4
    zonas = [(100 - C.NIVELES_PCT["leve"], 200, C.VERDE, "sin fatiga"),
             (100 - C.NIVELES_PCT["moderada"], 100 - C.NIVELES_PCT["leve"], C.LIMA, "leve"),
             (100 - C.NIVELES_PCT["alta"], 100 - C.NIVELES_PCT["moderada"], C.AMARILLO, "moderada"),
             (0, 100 - C.NIVELES_PCT["alta"], C.NARANJA, "alta")]
    for y0, y1, color, nombre in zonas:
        ax.axhspan(y0, y1, color=color, alpha=0.10, lw=0)
        ax.text(len(i) + 0.4, max(y0, abajo) + 1, nombre, color=color, fontsize=16, va="bottom", ha="right")
    ax.plot(i, pct, "o", color=C.CLARO, ms=11, zorder=3)
    ax.plot(i, 100 * (t["ordenada"] + t["pendiente"] * i) / t["ref"], color=C.LIMA, lw=4)
    ax.set_ylim(abajo, max(np.nanmax(pct), 100) + 5)
    ax.set_xlim(0.5, len(i) + 0.5)
    from matplotlib.ticker import MaxNLocator
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xlabel("repetición", fontsize=18)
    ax.set_ylabel("MDF  (% de la referencia)", fontsize=18)
    ax.tick_params(labelsize=15)
    fuerte = C.titulo_fuerte()
    c = t["caida_pct"]
    if not t["significativa"]:
        titulo = f"LA MDF NO CAMBIÓ EN {len(i)} REPETICIONES"
    elif c > 0:
        titulo = f"LA MDF CAYÓ {c:.0f} % EN {len(i)} REPETICIONES"
    else:
        titulo = f"LA MDF SUBIÓ {-c:.0f} % EN {len(i)} REPETICIONES"
    fig.text(0.07, 0.86, titulo, fontsize=50,
             color=C.CLARO, **fuerte)
    m = s["meta"]
    detalle = " · ".join(x for x in [m.get("persona"), m.get("ejercicio"),
                                       f"{m['peso_kg']} kg" if m.get("peso_kg") else ""] if x)
    fig.text(0.07, 0.80, f"referencia {t['ref']:.1f} Hz  ·  pendiente {t['pendiente_por_rep']} Hz/rep  ·  "
                         f"r = {t['r']}  ·  p = {t['p']:.2g}" + (f"   ·   {detalle}" if detalle else ""),
             fontsize=18, color=C.GRIS)
    fig.text(0.07, 0.04, "MyoSignals", fontsize=20, color=C.LIMA, **fuerte)
    fig.savefig(os.path.join(salida, "diapo.png"), dpi=100)
    plt.close(fig)


# =============================================================================
# Principal
# =============================================================================
def analizar(carpeta, graficos=True):
    s = cargar(carpeta)
    salida = os.path.join(s["carpeta"], "analisis")
    os.makedirs(salida, exist_ok=True)
    env = envolvente_rms(s["y"])
    res = {"sesion": s["nombre"], "meta": s["meta"], "enlace": analizar_enlace(s), "ruido": analizar_ruido(s, env)}
    reposo = res["ruido"]["reposo_esp_mV"] or res["ruido"]["ruido_base_mV"]

    esp = reps_esp(s)
    estados = estados_esp(s)
    picos_pc = contar_picos(s, env, reposo, estados)
    hay_esp = len(esp) > 0
    if not hay_esp and len(picos_pc):
        # Sin datos de la ESP: las repeticiones son las que cuenta la PC
        esp = pd.DataFrame({"t": picos_pc, "serie": 1, "reps": np.arange(1, len(picos_pc) + 1),
                            "mdf": np.nan, "rms": np.nan, "estado": "", "fatiga": np.nan, "ref": np.nan})
    reps = analizar_reps(s, esp)
    coinciden = sum(np.min(np.abs(picos_pc - t)) < COINCIDENCIA_S for t in esp.t) if len(picos_pc) and len(esp) else 0
    res["conteo"] = {"esp": int(len(esp)) if hay_esp else None, "pc": int(len(picos_pc)),
                     "coinciden": int(coinciden) if hay_esp else None}
    v = reps.dropna(subset=["mdf", "mdf_esp"]) if len(reps) else reps
    if len(v):
        dif = np.abs(v.mdf - v.mdf_esp)
        res["validacion"] = {"n": int(len(v)), "mae_hz": round(float(dif.mean()), 2), "max_hz": round(float(dif.max()), 2)}
    # ---- Comparación de métodos (sobre todas las reps detectadas, antes de depurar) ----
    t_marca = float(s["marcas"].t_s.iloc[0] - s["t0"]) if not s["marcas"].empty else None
    comparacion = None
    if len(reps) >= metodos.REPS_OBSERVAR:
        serie_mayor = reps.serie.value_counts().idxmax()          # la serie más larga
        rm = reps[reps.serie == serie_mayor]
        if t_marca is not None and EXCLUIR_POST_MARCA and not hay_esp:
            rm = rm[rm.t_s <= t_marca]
        car, info = metodos.caracteristicas(s, rm.reset_index(drop=True), env, reposo, BANDA)
        tabla, curvas = metodos.comparar(car, t_marca)
        comparacion = (car, tabla, curvas, info)

    # Igual que la ESP y los scripts del grupo: todas se cuentan, pero solo las válidas entran en la tendencia
    reps, res["depuracion"] = depurar_reps(s, reps, excluir_post_marca=EXCLUIR_POST_MARCA and not hay_esp,
                                           reposo=reposo)

    res["series"] = []
    for numero, r in (reps.groupby("serie") if len(reps) else []):
        r = r.reset_index(drop=True)
        res_serie = s["resultados"][s["resultados"].serie == numero] if hay_esp else pd.DataFrame()
        ultimo = res_serie.iloc[-1] if len(res_serie) else pd.Series({"estado": "—", "fatiga": float("nan")})
        res["series"].append({"serie": int(numero), "reps": int(len(r)),
                              "mdf": tendencia(r, "mdf"), "mnf": tendencia(r, "mnf"), "rms": tendencia(r, "rms"),
                              "estado_final_esp": ultimo.estado, "caida_final_esp": float(ultimo.fatiga)})

    umbral = ACTIVIDAD_FACTOR * res["ruido"]["ruido_base_mV"]
    ventanas = ventanas_deslizantes(s, env, umbral)
    res["isometrica"] = tendencia_temporal(ventanas)

    res["marcas"] = []
    for _, m in s["marcas"].iterrows() if not s["marcas"].empty else []:
        t_rel = m.t_s - s["t0"]
        cerca = int(reps.rep.iloc[np.argmin(np.abs(reps.t_s - t_rel))]) if len(reps) else None
        previos = estados[estados.t <= t_rel]
        res["marcas"].append({"tipo": m.tipo, "t_s": round(t_rel, 1), "rep_cercana": cerca,
                              "estado_esp": previos.estado.iloc[-1] if len(previos) else "—"})

    res["conclusiones"] = conclusiones(res)
    if comparacion:
        car, tabla, curvas, info = comparacion
        res["conclusiones"] += metodos.conclusiones(tabla, t_marca, info)
        res["metodos"] = tabla.to_dict(orient="records")

    # ---- archivos ----
    if len(reps):
        reps.drop(columns="_psd").round(3).to_csv(os.path.join(salida, "reps.csv"), index=False)
    ventanas.round(3).to_csv(os.path.join(salida, "ventanas.csv"), index=False)
    if comparacion:
        car, tabla, curvas, info = comparacion
        tabla.to_csv(os.path.join(salida, "metodos.csv"), index=False)
        detalle = car.copy()
        for (clave, senal), cv in curvas.items():
            if senal == "MDF+MNF":
                detalle[f"caida_{clave}_MDF+MNF"] = np.round(cv["caida"], 2)
                detalle[f"valida_{clave}"] = cv["valida"]
            else:
                detalle[f"caida_{clave}_{senal}"] = np.round(cv["caida"], 2)
        detalle.round(3).to_csv(os.path.join(salida, "metodos_reps.csv"), index=False)
    limpio = json.loads(json.dumps(res, default=lambda o: o.item() if hasattr(o, "item") else str(o)))
    for serie in limpio["series"]:
        for k in ("mdf", "mnf", "rms"):
            if serie[k]:
                serie[k].pop("ordenada"), serie[k].pop("pendiente")
    with open(os.path.join(salida, "resumen.json"), "w", encoding="utf-8") as f:
        json.dump(limpio, f, ensure_ascii=False, indent=2)
    with open(os.path.join(salida, "resumen.txt"), "w", encoding="utf-8") as f:
        f.write(f"Sesión {s['nombre']}\n" + "\n".join(res["conclusiones"]) + "\n")

    if graficos:
        import matplotlib
        if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
            matplotlib.use("Agg")
        C.aplicar_estilo()
        graficar_senal(s, env, reps, picos_pc, estados, ventanas, salida)
        if len(reps):
            graficar_reps(reps, res["series"], salida)
            graficar_espectros(reps, salida)
            graficar_validacion(reps, res["conteo"], salida)
            mejor = max(res["series"], key=lambda x: x["reps"]) if res["series"] else None
            graficar_diapo(s, reps, mejor, salida)
        if comparacion:
            car, tabla, curvas, info = comparacion
            metodos.graficar(car, tabla, curvas, t_marca, salida, s["nombre"])

    print(f"\n=== {s['nombre']} ===")
    print("\n".join(res["conclusiones"]))
    print(f"\nArchivos en {salida}")
    return res


if __name__ == "__main__":
    # Uno o varios: python analizar.py a.csv b.csv sesiones/xxx   (sin nada = la última sesión)
    # (en Windows la terminal no expande *.csv: lo hacemos acá)
    rutas = []
    for arg in (sys.argv[1:] or ["ultima"]):
        rutas += sorted(glob.glob(arg)) if any(c in arg for c in "*?") else [arg]
    for ruta in rutas:
        analizar(resolver_carpeta(ruta))
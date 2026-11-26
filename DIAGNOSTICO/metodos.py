"""metodos.py — Compara formas de medir la fatiga sobre las mismas repeticiones.

Lo usa analizar.py. Cada método simula la ESP repetición por repetición (sin mirar el
futuro) y agrega UN cambio al anterior, así se ve qué aporta cada uno:

  M0  actual         lo que hace hoy la ESP: reps válidas (RMS >= 70 % de la mediana),
                     referencia = mediana de las 3 primeras válidas (después de ver 5),
                     recta por mínimos cuadrados, nivel confirmado con 2 reps y solo sube.
  M1  + Theil-Sen    la pendiente es la mediana de las pendientes entre todos los pares
                     de reps: una rep rara casi no mueve la recta.
  M2  + ref recta    la referencia es el valor de la recta en la primera rep (no 3 sueltas).
  M3  + calidad      además descarta reps saturadas, con mucha energía < 30 Hz
                     (movimiento) o con poca relación señal/ruido.
  M4  + espectro     resta el espectro del ruido (medido en reposo) y rellena el pozo
                     del notch de 50 Hz antes de calcular MDF / MNF.
  M5  + JASA         solo sube el nivel si la RMS no bajó: MDF baja con RMS que sube es
                     fatiga; MDF baja con RMS que baja es menos fuerza.

Y cada método se corre con cuatro señales:
  MDF         la caída de la MDF con los umbrales de siempre (7 / 12 / 20 %)
  MNF         la caída de la MNF con umbrales propios, más bajos (MNF_FACTOR_UMBRALES × los de la MDF),
              porque la MNF cae menos en % que la MDF
  MDF+MNF     tienen que caer las dos (más exigente)
  MDF o MNF   alcanza con que caiga una de las dos (avisa antes)
"""
import numpy as np
from scipy import signal

import config as C

# =============================================================================
# Configuración
# =============================================================================
REPS_OBSERVAR = 5           # igual que REF_REPS_OBSERVAR de la ESP
REPS_REFERENCIA = 3         # igual que REF_REPS
VALIDA_RMS_MIN = 0.70       # igual que VALIDA_RMS_MIN
REPS_MIN_RECTA = 4          # igual que FATIGA_REPS_MIN_RECTA
REPS_MEDIANA = 3            # igual que FATIGA_REPS_MEDIANA
CONFIRMAR_REPS = 2          # igual que FATIGA_CONFIRMAR_REPS

# Calidad (M3)
CALIDAD_FRAC_BAJA_MAX = 0.35   # fracción de potencia entre 20 y 30 Hz (movimiento del cable)
CALIDAD_SNR_MIN = 2.0          # RMS de la rep / RMS del reposo
CALIDAD_SAT_MAX = 0.01         # fracción de muestras crudas pegadas al piso o al techo del ADC

# Espectro limpio (M4)
REPOSO_FACTOR = 1.5            # tramo de reposo: envolvente < 1,5 × reposo durante toda la ventana
REPOSO_MAX_TRAMOS = 20
NOTCH_HZ = (47.0, 53.0)        # bins que se rellenan interpolando

# JASA (M5)
JASA_RMS_MIN_SUBIDA = 0.0      # la RMS actual tiene que ser >= referencia × (1 + esto)

METODOS = [
    {"clave": "M0", "nombre": "actual (ESP)", "theil_sen": False, "ref_recta": False, "calidad": False, "limpio": False, "jasa": False},
    {"clave": "M1", "nombre": "+ Theil-Sen", "theil_sen": True, "ref_recta": False, "calidad": False, "limpio": False, "jasa": False},
    {"clave": "M2", "nombre": "+ ref de la recta", "theil_sen": True, "ref_recta": True, "calidad": False, "limpio": False, "jasa": False},
    {"clave": "M3", "nombre": "+ calidad", "theil_sen": True, "ref_recta": True, "calidad": True, "limpio": False, "jasa": False},
    {"clave": "M4", "nombre": "+ espectro limpio", "theil_sen": True, "ref_recta": True, "calidad": True, "limpio": True, "jasa": False},
    {"clave": "M5", "nombre": "+ JASA (RMS)", "theil_sen": True, "ref_recta": True, "calidad": True, "limpio": True, "jasa": True},
]
# Umbrales de la MNF = este factor × umbrales de la MDF (0,7 -> 4,9 / 8,4 / 14 %).
# Sale de las primeras grabaciones (la MNF cayó ~70 % de lo que cayó la MDF): revisarlo con más datos.
MNF_FACTOR_UMBRALES = 0.7
SENALES = ["MDF", "MNF", "MDF+MNF", "MDF o MNF"]
NIVELES = ["sin", "leve", "moderada", "alta"]


# =============================================================================
# Espectro
# =============================================================================
def psd(x):
    f, p = signal.welch(x - np.mean(x), C.FS, window="hann", nperseg=C.NFFT, noverlap=C.NFFT // 2)
    return f, p


def frecuencias(f, p, banda):
    k = (f >= banda[0]) & (f <= banda[1])
    fb, pb = f[k], np.clip(p[k], 0, None)
    acum = np.cumsum(pb)
    if acum[-1] <= 0:
        return np.nan, np.nan
    return float(np.interp(acum[-1] / 2, acum, fb)), float(np.sum(fb * pb) / np.sum(pb))


def rellenar_notch(f, p):
    p = p.copy()
    k = (f >= NOTCH_HZ[0]) & (f <= NOTCH_HZ[1])
    if k.any() and (~k).sum() > 2:
        p[k] = np.interp(f[k], f[~k], p[~k])
    return p


def espectro_ruido(y, env, reposo):
    """Promedio del espectro en tramos quietos de 1,5 s (envolvente baja en toda la ventana)."""
    largo = int(C.VENTANA_REP_S * C.FS)
    quieto = np.nan_to_num(env) < REPOSO_FACTOR * reposo
    quieto &= np.isfinite(y)
    tramos, i = [], 0
    while i + largo <= len(y) and len(tramos) < REPOSO_MAX_TRAMOS:
        if quieto[i:i + largo].all():
            tramos.append(psd(y[i:i + largo])[1])
            i += largo
        else:
            i += largo // 4
    return (np.mean(tramos, axis=0), len(tramos)) if tramos else (None, 0)


# =============================================================================
# Características de cada repetición
# =============================================================================
def caracteristicas(s, reps, env, reposo, banda):
    """MDF / MNF normales y limpias, RMS y banderas de calidad de cada repetición."""
    p_ruido, n_tramos = espectro_ruido(s["y"], env, reposo)
    rms_reposo = float(np.sqrt(np.mean(s["y"][np.nan_to_num(env) < REPOSO_FACTOR * reposo] ** 2))) \
        if (np.nan_to_num(env) < REPOSO_FACTOR * reposo).any() else reposo
    medio = int(C.VENTANA_REP_S * C.FS) // 2
    cruda = s.get("y_cruda")
    filas = []
    for _, r in reps.iterrows():
        i = int(round(r.t_s * C.FS))
        x = s["y"][i - medio:i + medio] if i - medio >= 0 and i + medio <= len(s["y"]) else None
        if x is None or len(x) < C.NFFT or np.isnan(x).any():
            filas.append(None)
            continue
        f, p = psd(x)
        mdf, mnf = frecuencias(f, p, banda)
        pl = p - p_ruido if p_ruido is not None else p
        pl = rellenar_notch(f, np.clip(pl, 0, None))
        mdf_l, mnf_l = frecuencias(f, pl, banda)
        en_banda = (f >= banda[0]) & (f <= banda[1])
        baja = en_banda & (f < 30)
        frac_baja = float(p[baja].sum() / p[en_banda].sum()) if p[en_banda].sum() > 0 else 0.0
        rms = float(np.sqrt(np.mean(x ** 2)))
        sat = 0.0
        if cruda is not None:
            c = cruda[i - medio:i + medio]
            c = c[np.isfinite(c)]
            if len(c):
                sat = float(np.mean((c <= 10) | (c >= np.nanmax(cruda) - 2)))
        filas.append({"mdf": mdf, "mnf": mnf, "mdf_l": mdf_l, "mnf_l": mnf_l, "rms": rms,
                      "frac_baja": frac_baja, "snr": rms / rms_reposo if rms_reposo > 0 else np.inf, "sat": sat})
    vacia = {"mdf": np.nan, "mnf": np.nan, "mdf_l": np.nan, "mnf_l": np.nan, "rms": np.nan,
             "frac_baja": np.nan, "snr": np.nan, "sat": np.nan}
    import pandas as pd
    out = pd.DataFrame([fila or vacia for fila in filas])
    out["t_s"] = reps.t_s.values
    out["calidad_ok"] = (out.sat <= CALIDAD_SAT_MAX) & (out.frac_baja <= CALIDAD_FRAC_BAJA_MAX) & (out.snr >= CALIDAD_SNR_MIN)
    info = {"tramos_reposo": n_tramos, "rms_reposo": round(rms_reposo, 2),
            "calidad_ok": int(out.calidad_ok.sum()), "reps": int(len(out)),
            "motivos": {"saturadas": int((out.sat > CALIDAD_SAT_MAX).sum()),
                        "movimiento": int((out.frac_baja > CALIDAD_FRAC_BAJA_MAX).sum()),
                        "snr_baja": int((out.snr < CALIDAD_SNR_MIN).sum())}}
    return out, info


# =============================================================================
# Simulación de la ESP, repetición por repetición
# =============================================================================
def ajuste(x, y, theil_sen):
    """Pendiente y ordenada. Theil-Sen: mediana de las pendientes entre pares."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if theil_sen:
        i, j = np.triu_indices(len(x), 1)
        pendiente = float(np.median((y[j] - y[i]) / (x[j] - x[i])))
        ordenada = float(np.median(y - pendiente * x))
    else:
        pendiente, ordenada = np.polyfit(x, y, 1)
    return pendiente, ordenada


def validas(rms, calidad_ok, usar_calidad):
    """Válida al momento de llegar: RMS >= 70 % de la mediana de las RMS hasta ella (como la ESP)."""
    v = np.zeros(len(rms), bool)
    for k in range(len(rms)):
        hasta = rms[:k + 1][np.isfinite(rms[:k + 1])]
        v[k] = np.isfinite(rms[k]) and len(hasta) and rms[k] >= VALIDA_RMS_MIN * np.median(hasta)
        if usar_calidad:
            v[k] &= bool(calidad_ok[k])
    return v


def simular_caida(valores, rms, valida, m):
    """Caída [%] después de cada repetición y si la RMS habilita subir de nivel (JASA)."""
    n = len(valores)
    caida = np.full(n, np.nan)
    habilita = np.ones(n, bool)
    elegidas = None
    for k in range(n):
        if elegidas is None:
            if k + 1 < REPS_OBSERVAR:
                continue
            cand = [i for i in range(k + 1) if valida[i] and np.isfinite(valores[i])][:REPS_REFERENCIA]
            if len(cand) < REPS_REFERENCIA:
                continue
            elegidas = cand
        idx = [i for i in range(elegidas[0], k + 1) if valida[i] and np.isfinite(valores[i])]
        y, r = valores[idx], rms[idx]
        x = np.arange(len(idx), dtype=float)            # posición entre las válidas, como la ESP
        ref = float(np.median(valores[elegidas]))
        rms_ref = float(np.median(rms[elegidas]))
        if len(idx) >= REPS_MIN_RECTA:
            pend, ord_ = ajuste(x, y, m["theil_sen"])
            actual = ord_ + pend * x[-1]
            if m["ref_recta"]:
                ref = ord_
            pr, orr = ajuste(x, r, m["theil_sen"])
            rms_actual = orr + pr * x[-1]
        else:
            actual = float(np.median(y[-REPS_MEDIANA:]))
            rms_actual = float(np.median(r[-REPS_MEDIANA:]))
        caida[k] = 100 * (ref - actual) / ref if ref else np.nan
        if m["jasa"]:
            habilita[k] = rms_actual >= rms_ref * (1 + JASA_RMS_MIN_SUBIDA)
    return caida, habilita


def niveles(caida, habilita):
    """Nivel después de cada rep: sube si 2 reps seguidas lo superan, nunca baja (como la ESP)."""
    umbrales = [C.NIVELES_PCT["leve"], C.NIVELES_PCT["moderada"], C.NIVELES_PCT["alta"]]
    nivel, seguidas, out = 0, 0, []
    for c, h in zip(caida, habilita):
        propuesto = 0 if not np.isfinite(c) else sum(c >= u for u in umbrales)
        if propuesto > nivel and h:
            seguidas += 1
            if seguidas >= CONFIRMAR_REPS:
                nivel, seguidas = propuesto, 0
        else:
            seguidas = 0
        out.append(nivel)
    return np.array(out)


def residuo_pct(valores, valida, theil_sen):
    """Dispersión de las reps alrededor de la recta final (MAD, % del nivel): menos = más estable."""
    y = valores[valida & np.isfinite(valores)]
    if len(y) < REPS_MIN_RECTA:
        return np.nan
    x = np.arange(len(y), dtype=float)
    p, o = ajuste(x, y, theil_sen)
    res = y - (o + p * x)
    return float(100 * 1.4826 * np.median(np.abs(res - np.median(res))) / np.median(y))


# =============================================================================
# Comparación completa
# =============================================================================
def comparar(car, t_marca):
    """Corre todos los métodos × señales. Devuelve (tabla resumen, curvas por rep)."""
    import pandas as pd
    filas, curvas = [], {}
    rms = car.rms.values.astype(float)
    for m in METODOS:
        valida = validas(rms, car.calidad_ok.values, m["calidad"])
        cols = ("mdf_l", "mnf_l") if m["limpio"] else ("mdf", "mnf")
        c_mdf, h = simular_caida(car[cols[0]].values.astype(float), rms, valida, m)
        c_mnf, _ = simular_caida(car[cols[1]].values.astype(float), rms, valida, m)
        # La MNF se lleva a la escala de la MDF dividiendo por el factor: así comparten umbrales
        c_mnf_eq = c_mnf / MNF_FACTOR_UMBRALES
        for senal, caida in (("MDF", c_mdf), ("MNF", c_mnf_eq), ("MDF+MNF", np.fmin(c_mdf, c_mnf_eq)),
                             ("MDF o MNF", np.fmax(c_mdf, c_mnf_eq))):
            nv = niveles(caida, h)
            curvas[(m["clave"], senal)] = {"caida": caida, "nivel": nv, "valida": valida}
            real = caida * MNF_FACTOR_UMBRALES if senal == "MNF" else caida      # la MNF se informa en su % real
            fila = {"metodo": m["clave"], "nombre": m["nombre"], "senal": senal,
                    "reps_usadas": int(valida.sum()), "reps_total": int(len(valida)),
                    "caida_final_pct": round(float(real[np.isfinite(real)][-1]), 1) if np.isfinite(real).any() else np.nan,
                    "nivel_final": NIVELES[nv[-1]] if len(nv) else "sin"}
            for k, nombre in enumerate(NIVELES[1:], start=1):
                cruce = np.where(nv >= k)[0]
                fila[f"rep_{nombre}"] = int(cruce[0] + 1) if len(cruce) else np.nan
                fila[f"t_{nombre}_s"] = round(float(car.t_s.iloc[cruce[0]]), 1) if len(cruce) else np.nan
                if t_marca is not None:
                    fila[f"antes_marca_{nombre}_s"] = round(t_marca - fila[f"t_{nombre}_s"], 1) if len(cruce) else np.nan
            col = cols[1] if senal == "MNF" else cols[0]
            fila["dispersion_pct"] = round(residuo_pct(car[col].values.astype(float), valida, m["theil_sen"]), 2)
            filas.append(fila)
    return pd.DataFrame(filas), curvas


def conclusiones(tabla, t_marca, info):
    """Frases cortas con lo que cambia cada método (sin interpretar de más)."""
    c = [f"Métodos (espectro de ruido medido en {info['tramos_reposo']} tramos de reposo de 1,5 s; "
         f"reps que pasan el filtro de calidad: {info['calidad_ok']}/{info['reps']}):"]
    for senal in SENALES:
        t = tabla[tabla.senal == senal]
        partes = []
        for _, r in t.iterrows():
            if not np.isfinite(r.caida_final_pct):
                partes.append(f"{r.metodo} sin referencia ({r.reps_usadas}/{r.reps_total} reps pasaron)")
                continue
            aviso = f"alta en rep {int(r.rep_alta)}" if np.isfinite(r.rep_alta) else (
                f"{r.nivel_final}" if r.nivel_final != "sin" else "sin aviso")
            partes.append(f"{r.metodo} {r.caida_final_pct:+.0f} % ({aviso})")
        c.append(f"   {senal:9s} " + " · ".join(partes))
    mo = info["motivos"]
    if info["calidad_ok"] < info["reps"]:
        c.append(f"   Calidad (M3+): {mo['saturadas']} saturadas, {mo['movimiento']} con movimiento, {mo['snr_baja']} con SNR baja.")
    c.append(f"   (MNF con umbrales × {MNF_FACTOR_UMBRALES}; en MDF+MNF y MDF o MNF el % está en la escala de la MDF)")
    if t_marca is not None:
        c.append(f"   Marca de fatiga del usuario en {t_marca:.1f} s (antes_marca > 0 = el aviso llegó antes).")
    return c


# =============================================================================
# Gráfico
# =============================================================================
def graficar(car, tabla, curvas, t_marca, salida, nombre_sesion):
    import os
    import matplotlib.pyplot as plt
    colores = [C.GRIS, "#5AC8FA", C.CLARO, C.AMARILLO, C.NARANJA, C.LIMA]
    fig, ejes = plt.subplots(3, 2, figsize=(16, 13), gridspec_kw={"width_ratios": [1, 1.25]})
    rep = np.arange(1, len(car) + 1)
    rep_marca = None
    if t_marca is not None and len(car):
        antes = np.where(car.t_s.values <= t_marca)[0]
        rep_marca = antes[-1] + 1.5 if len(antes) else None
    for fila_ax, (senal, col, col_l) in enumerate((("MDF", "mdf", "mdf_l"), ("MNF", "mnf", "mnf_l"))):
        a, b = ejes[fila_ax]
        ref = np.nanmedian(car[col].values[:REPS_REFERENCIA + 2])
        val0 = curvas[("M0", senal)]["valida"]
        a.plot(rep[val0], 100 * car[col][val0] / ref, "o", color=C.GRIS, ms=6, label=f"{senal} (reps válidas)")
        a.plot(rep[~val0], 100 * car[col][~val0] / ref, "x", color=C.GRIS_OSCURO, ms=7, label="descartadas (M0)")
        a.plot(rep, 100 * car[col_l] / ref, ".", color=C.LIMA, ms=6, alpha=0.8, label=f"{senal} espectro limpio")
        for theil, color, nombre in ((False, C.GRIS, "mínimos cuadrados"), (True, C.CLARO, "Theil-Sen")):
            y = car[col].values[val0]
            if len(y) >= REPS_MIN_RECTA:
                x = np.arange(len(y), dtype=float)
                p, o = ajuste(x, y, theil)
                a.plot(rep[val0], 100 * (o + p * x) / ref, color=color, lw=2, ls="--" if theil else "-", label=nombre)
        a.set_title(f"{senal} por repetición (% de las primeras)")
        a.set_xlabel("repetición detectada")
        a.set_ylabel("%")
        a.legend(fontsize=8)
        escala = MNF_FACTOR_UMBRALES if senal == "MNF" else 1.0       # la MNF tiene umbrales propios
        for i, m in enumerate(METODOS):
            cv = curvas[(m["clave"], senal)]
            b.plot(rep, cv["caida"] * escala, "-", lw=2.2 if m["clave"] in ("M0", "M5") else 1.4,
                   color=colores[i], label=f"{m['clave']} {m['nombre']}")
        for nivel, color in (("leve", C.LIMA), ("moderada", C.AMARILLO), ("alta", C.NARANJA)):
            b.axhline(C.NIVELES_PCT[nivel] * escala, color=color, lw=0.8, ls=":")
        if rep_marca:
            for ax in (a, b):
                ax.axvline(rep_marca, color=C.CLARO, lw=1.2, ls=":")
            b.text(rep_marca, b.get_ylim()[1] * 0.92, " marca", color=C.CLARO, fontsize=9)
        b.axhline(0, color=C.GRIS_OSCURO, lw=0.8)
        b.set_title(f"Caída de {senal} que vería la ESP después de cada rep"
                    + (f"  (umbrales × {MNF_FACTOR_UMBRALES})" if senal == "MNF" else ""))
        b.set_xlabel("repetición detectada")
        b.set_ylabel("caída %")
        b.legend(fontsize=8, ncol=2)
    # JASA: MDF vs RMS
    a = ejes[2, 0]
    ref_m = np.nanmedian(car.mdf.values[:REPS_REFERENCIA + 2])
    ref_r = np.nanmedian(car.rms.values[:REPS_REFERENCIA + 2])
    xr, ym = 100 * car.rms / ref_r - 100, 100 * car.mdf / ref_m - 100
    sc = a.scatter(xr, ym, c=rep, cmap="viridis", s=40, zorder=3)
    a.axhline(0, color=C.GRIS_OSCURO, lw=1)
    a.axvline(0, color=C.GRIS_OSCURO, lw=1)
    lim_x, lim_y = a.get_xlim(), a.get_ylim()
    for (x, y, texto) in ((0.97, 0.04, "FATIGA\n(MDF↓ RMS↑)"), (0.03, 0.04, "menos fuerza\n(MDF↓ RMS↓)"),
                          (0.97, 0.96, "más fuerza\n(MDF↑ RMS↑)"), (0.03, 0.96, "recuperación\n(MDF↑ RMS↓)")):
        a.text(x, y, texto, transform=a.transAxes, ha="right" if x > 0.5 else "left",
               va="bottom" if y < 0.5 else "top", fontsize=9, color=C.NARANJA if "FATIGA" in texto else C.GRIS)
    a.set_xlim(lim_x)
    a.set_ylim(lim_y)
    fig.colorbar(sc, ax=a, label="repetición")
    a.set_title("JASA: cambio de MDF vs cambio de RMS")
    a.set_xlabel("RMS (% de cambio)")
    a.set_ylabel("MDF (% de cambio)")
    # Resumen: en qué rep avisa "alta" cada método
    b = ejes[2, 1]
    ancho = 0.2
    for j, senal in enumerate(SENALES):
        t = tabla[tabla.senal == senal]
        y = [r.rep_alta if np.isfinite(r.rep_alta) else 0 for _, r in t.iterrows()]
        barras = b.bar(np.arange(len(t)) + (j - 1.5) * ancho, y, ancho, color=(C.GRIS, C.CLARO, C.LIMA, C.NARANJA)[j], label=senal)
        for barra, r in zip(barras, t.itertuples()):
            if not np.isfinite(r.rep_alta):
                b.text(barra.get_x() + barra.get_width() / 2, 0.3, "—", ha="center", color=C.GRIS, fontsize=9)
    if rep_marca:
        b.axhline(rep_marca, color=C.CLARO, ls=":", lw=1.2, label="marca del usuario")
    b.set_xticks(range(len(METODOS)))
    b.set_xticklabels([f"{m['clave']}\n{m['nombre']}" for m in METODOS], fontsize=8)
    b.set_ylabel("repetición en que avisa ALTA")
    b.set_title("¿En qué repetición llega a fatiga alta?  (— = no llega)")
    b.legend(fontsize=8)
    b.grid(axis="x", visible=False)
    b.set_axisbelow(True)
    fig.suptitle(f"Comparación de métodos · {nombre_sesion}", fontsize=14, color=C.CLARO, x=0.01, ha="left")
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        fig.tight_layout()
    fig.savefig(os.path.join(salida, "metodos.png"), dpi=100)
    plt.close(fig)

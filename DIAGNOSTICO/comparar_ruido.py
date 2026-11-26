"""comparar_ruido.py — ¿Mejora la fatiga si no se usan las repeticiones ruidosas?

Corre sobre grabaciones ya guardadas (CSV o carpetas de grabar.py) el método de la ESP
(M2 + "MDF o MNF") dos veces: SIN el criterio de reps ruidosas (como antes) y CON él
(RMS de la rep >= VALIDA_SNR_MIN × reposo, como ahora). Muestra lado a lado qué cambia.

Uso:
    python comparar_ruido.py "sesiones/csv/*.csv"
    python comparar_ruido.py sesiones/csv/fatiga.csv sesiones/20261126_113623
    python comparar_ruido.py "sesiones/csv/*.csv" --snr 2.5     (probar otro umbral)

Deja por cada grabación, en <grabación>/analisis/ :  criterio_ruido.png  y  criterio_ruido.csv
y una tabla con todas en  sesiones/comparacion/criterio_ruido.csv
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

import analizar as A
import config as C
import metodos as M

METODO = "M2"                 # el que usa la ESP: Theil-Sen + referencia de la recta
SENALES = ["MDF o MNF", "MDF"]


def reps_de(s, env, reposo):
    """Repeticiones de la serie más larga (de la ESP, o contadas en la PC si es un CSV crudo)."""
    esp = A.reps_esp(s)
    estados = A.estados_esp(s)
    if esp.empty:
        picos = A.contar_picos(s, env, reposo, estados)
        esp = pd.DataFrame({"t": picos, "serie": 1, "reps": np.arange(1, len(picos) + 1),
                            "mdf": np.nan, "rms": np.nan, "estado": "", "fatiga": np.nan, "ref": np.nan})
    reps = A.analizar_reps(s, esp)
    if reps.empty:
        return reps, None
    reps = reps[reps.serie == reps.serie.value_counts().idxmax()]
    t_marca = float(s["marcas"].t_s.iloc[0] - s["t0"]) if not s["marcas"].empty else None
    if t_marca is not None and A.EXCLUIR_POST_MARCA and A.reps_esp(s).empty:
        reps = reps[reps.t_s <= t_marca]
    return reps.reset_index(drop=True), t_marca


def correr(car, t_marca, snr_min):
    M.VALIDA_SNR_MIN = snr_min
    tabla, curvas = M.comparar(car, t_marca)
    return tabla[tabla.metodo == METODO].set_index("senal"), curvas


def tendencia(valores, valida):
    """Pendiente y p de la MDF de las reps válidas (para ver si la tendencia se aclara)."""
    from scipy import stats
    y = valores[valida & np.isfinite(valores)]
    if len(y) < 4:
        return np.nan, np.nan
    r = stats.linregress(np.arange(len(y)), y)
    return r.slope, r.pvalue


def graficar(car, curvas_sin, curvas_con, t_marca, snr_min, salida, nombre):
    import matplotlib
    if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    C.aplicar_estilo()
    rep = np.arange(1, len(car) + 1)
    val_sin = curvas_sin[(METODO, "MDF")]["valida"]
    val_con = curvas_con[(METODO, "MDF")]["valida"]
    ruidosa = car.snr.values < snr_min
    fig, (a, b) = plt.subplots(1, 2, figsize=(16, 5.5))
    ref = np.nanmedian(car.mdf.values[val_con][:3]) if val_con.any() else np.nanmedian(car.mdf.values[:3])
    a.plot(rep[val_con], 100 * car.mdf[val_con] / ref, "o", color=C.LIMA, ms=8, label="usada")
    a.plot(rep[ruidosa], 100 * car.mdf[ruidosa] / ref, "x", color=C.NARANJA, ms=9, mew=2,
           label=f"ruidosa (RMS < {snr_min:g} × reposo)")
    otras = ~val_con & ~ruidosa
    a.plot(rep[otras], 100 * car.mdf[otras] / ref, "x", color=C.GRIS_OSCURO, ms=8, label="incompleta")
    for valida, color, ls, nombre_c in ((val_sin, C.GRIS, "-", "recta sin criterio"),
                                        (val_con, C.LIMA, "--", "recta con criterio")):
        y = car.mdf.values[valida]
        if len(y) >= M.REPS_MIN_RECTA:
            x = np.arange(len(y), dtype=float)
            p, o = M.ajuste(x, y, True)
            a.plot(rep[valida], 100 * (o + p * x) / ref, ls, color=color, lw=2, label=nombre_c)
    a.axhline(100, color=C.GRIS_OSCURO, lw=0.8)
    a.set_title("MDF por repetición (% de la referencia)")
    a.set_xlabel("repetición")
    a.set_ylabel("%")
    a.legend(fontsize=8)
    for curvas, color, ls, nombre_c in ((curvas_sin, C.GRIS, "-", "sin criterio (antes)"),
                                        (curvas_con, C.LIMA, "-", "con criterio (ahora)")):
        b.plot(rep, curvas[(METODO, "MDF o MNF")]["caida"], ls, color=color, lw=2.2, label=nombre_c)
    for nivel, color in (("leve", C.LIMA), ("moderada", C.AMARILLO), ("alta", C.NARANJA)):
        b.axhline(C.NIVELES_PCT[nivel], color=color, lw=0.8, ls=":")
    if t_marca is not None:
        antes = np.where(car.t_s.values <= t_marca)[0]
        if len(antes):
            b.axvline(antes[-1] + 1.5, color=C.CLARO, lw=1.2, ls=":")
    b.axhline(0, color=C.GRIS_OSCURO, lw=0.8)
    b.set_title("Caída que vería la ESP (M2, MDF o MNF)")
    b.set_xlabel("repetición")
    b.set_ylabel("caída % (escala de la MDF)")
    b.legend(fontsize=8)
    fig.suptitle(f"Criterio de reps ruidosas · {nombre}", fontsize=14, color=C.CLARO, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "criterio_ruido.png"), dpi=100)
    plt.close(fig)


def analizar_uno(ruta, snr_min):
    s = A.cargar(ruta)
    env = A.envolvente_rms(s["y"])
    ruido = A.analizar_ruido(s, env)
    reposo = ruido["reposo_esp_mV"] or ruido["ruido_base_mV"]
    reps, t_marca = reps_de(s, env, reposo)
    if len(reps) < M.REPS_OBSERVAR:
        print(f"\n=== {s['nombre']} ===\n   pocas repeticiones ({len(reps)}): se saltea")
        return None
    car, _ = M.caracteristicas(s, reps, env, reposo, A.BANDA)
    sin, curvas_sin = correr(car, t_marca, 0.0)
    con, curvas_con = correr(car, t_marca, snr_min)
    M.VALIDA_SNR_MIN = snr_min
    filas = []
    for senal in SENALES:
        for etiqueta, t, curvas in (("sin criterio", sin, curvas_sin), ("con criterio", con, curvas_con)):
            r = t.loc[senal]
            valida = curvas[(METODO, "MDF")]["valida"]
            pend, p = tendencia(car.mdf.values.astype(float), valida)
            filas.append({"grabacion": s["nombre"], "senal": senal, "criterio": etiqueta,
                          "reps": int(r.reps_total), "usadas": int(r.reps_usadas),
                          "ruidosas": int((car.snr < snr_min).sum()),
                          "ruido_base_mV": ruido["ruido_base_mV"], "snr_dB": ruido["snr_dB"],
                          "dispersion_pct": r.dispersion_pct, "pendiente_mdf_hz_rep": round(pend, 3),
                          "p_mdf": float(f"{p:.3g}") if np.isfinite(p) else np.nan,
                          "caida_final_pct": r.caida_final_pct, "nivel_final": r.nivel_final,
                          "rep_alta": r.rep_alta,
                          "antes_marca_alta_s": r.get("antes_marca_alta_s", np.nan)})
    tabla = pd.DataFrame(filas)
    salida = os.path.join(s["carpeta"], "analisis")
    os.makedirs(salida, exist_ok=True)
    tabla.to_csv(os.path.join(salida, "criterio_ruido.csv"), index=False)
    graficar(car, curvas_sin, curvas_con, t_marca, snr_min, salida, s["nombre"])

    print(f"\n=== {s['nombre']} ===   ruido de base {ruido['ruido_base_mV']} mV · SNR {ruido['snr_dB']} dB · "
          f"{int((car.snr < snr_min).sum())} de {len(car)} reps ruidosas")
    cols = ["senal", "criterio", "usadas", "dispersion_pct", "p_mdf", "caida_final_pct", "nivel_final", "rep_alta"]
    print(tabla[cols].to_string(index=False))
    print(f"   Gráfico: {os.path.join(salida, 'criterio_ruido.png')}")
    return tabla


def main():
    args = sys.argv[1:]
    snr_min = M.VALIDA_SNR_MIN
    if "--snr" in args:
        i = args.index("--snr")
        snr_min = float(args[i + 1])
        args = args[:i] + args[i + 2:]
    rutas = []
    for arg in args or ["ultima"]:
        rutas += sorted(glob.glob(arg)) if any(c in arg for c in "*?") else [arg]
    tablas = [t for t in (analizar_uno(A.resolver_carpeta(r), snr_min) for r in rutas) if t is not None]
    if not tablas:
        return
    todo = pd.concat(tablas, ignore_index=True)
    carpeta = os.path.join(C.CARPETA_SESIONES, "comparacion")
    os.makedirs(carpeta, exist_ok=True)
    todo.to_csv(os.path.join(carpeta, "criterio_ruido.csv"), index=False)
    print("\nCómo leerlo: 'con criterio' es mejor si baja la dispersión, la tendencia de la MDF se vuelve más")
    print("clara (p más chico) y en las grabaciones con fatiga sigue llegando a alta (y en los controles no).")
    print(f"Tabla de todas: {os.path.join(carpeta, 'criterio_ruido.csv')}")


if __name__ == "__main__":
    main()
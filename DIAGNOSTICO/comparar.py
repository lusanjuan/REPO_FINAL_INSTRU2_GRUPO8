"""comparar.py — Junta todas las sesiones analizadas en una tabla y un gráfico.

Uso:
    python comparar.py                       (todas las sesiones con analisis/)
    python comparar.py isometrica curl_4kg   (solo las que tengan esas palabras en el nombre)
Deja en  sesiones/comparacion/ :  comparacion.csv  y  comparacion.png
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

import config as C

COLORES = [C.LIMA, C.NARANJA, C.CLARO, C.AMARILLO, C.VERDE, C.GRIS, "#5AC8FA", "#FF6B9A"]


def cargar(filtros):
    filas, curvas = [], []
    for resumen in sorted(glob.glob(os.path.join(C.CARPETA_SESIONES, "*", "analisis", "resumen.json"))):
        carpeta = os.path.dirname(os.path.dirname(resumen))
        nombre = os.path.basename(carpeta)
        if filtros and not any(f in nombre for f in filtros):
            continue
        r = json.load(open(resumen, encoding="utf-8"))
        m = r.get("meta", {})
        serie = max(r["series"], key=lambda s: s["reps"]) if r["series"] else {}
        mdf, mnf, rms = serie.get("mdf") or {}, serie.get("mnf") or {}, serie.get("rms") or {}
        iso = r.get("isometrica") or {}
        filas.append({
            "sesion": nombre, "persona": m.get("persona", ""), "peso_kg": m.get("peso_kg", ""),
            "duracion_s": r["enlace"]["duracion_s"], "senal_recibida_pct": r["enlace"]["muestras_recibidas_pct"],
            "ruido_base_mV": r["ruido"]["ruido_base_mV"], "snr_dB": r["ruido"]["snr_dB"],
            "recortado_pct": r["ruido"]["recortado_pct"],
            "reps_esp": r["conteo"]["esp"], "reps_pc": r["conteo"]["pc"], "reps_coinciden": r["conteo"]["coinciden"],
            "mdf_esp_vs_pc_hz": (r.get("validacion") or {}).get("mae_hz"),
            "mdf_ref_hz": mdf.get("ref"), "mdf_pendiente_hz_rep": mdf.get("pendiente_por_rep"),
            "mdf_r": mdf.get("r"), "mdf_p": mdf.get("p"), "mdf_caida_pct": mdf.get("caida_pct"),
            "mnf_caida_pct": mnf.get("caida_pct"), "rms_cambio_pct": rms.get("cambio_pct"),
            "estado_final_esp": serie.get("estado_final_esp"), "caida_final_esp": serie.get("caida_final_esp"),
            "iso_mdf_caida_pct": iso.get("caida_pct"), "iso_pendiente_hz_s": iso.get("pendiente_hz_por_s"),
        })
        reps = os.path.join(carpeta, "analisis", "reps.csv")
        if mdf and os.path.exists(reps):
            d = pd.read_csv(reps)
            d = d[d.serie == serie["serie"]]
            curvas.append((nombre, 100 * d.mdf.values / mdf["ref"]))
    return pd.DataFrame(filas), curvas


def graficar(tabla, curvas, salida):
    import matplotlib
    if not os.environ.get("DISPLAY") and sys.platform.startswith("linux"):
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    C.aplicar_estilo()
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(16, 6), gridspec_kw={"width_ratios": [1.6, 1]})
    for k, (nombre, pct) in enumerate(curvas):
        a1.plot(np.arange(1, len(pct) + 1), pct, "o-", ms=4, lw=1.5, color=COLORES[k % len(COLORES)], label=nombre)
    for nivel, color in (("leve", C.LIMA), ("moderada", C.AMARILLO), ("alta", C.NARANJA)):
        a1.axhline(100 - C.NIVELES_PCT[nivel], color=color, lw=0.8, ls=":")
    a1.axhline(100, color=C.GRIS_OSCURO, lw=1)
    a1.set_title("MDF por repetición (% de la referencia de cada sesión)")
    a1.xaxis.set_major_locator(MaxNLocator(integer=True))
    a1.set_xlabel("repetición")
    a1.set_ylabel("%")
    a1.legend(fontsize=8)
    t = tabla.dropna(subset=["mdf_caida_pct"])
    colores = [COLORES[k % len(COLORES)] for k in range(len(t))]
    a2.barh(t.sesion, t.mdf_caida_pct, color=colores)
    for y, (c, p) in enumerate(zip(t.mdf_caida_pct, t.mdf_p)):
        a2.text(c, y, f"  {c:.1f} %  (p = {p:.2g})", va="center", fontsize=9, color=C.CLARO)
    a2.axvline(0, color=C.GRIS_OSCURO, lw=1)
    a2.set_title("Caída de la MDF (recta)")
    a2.invert_yaxis()
    a2.set_axisbelow(True)
    a2.grid(axis="y", visible=False)
    fig.tight_layout()
    fig.savefig(os.path.join(salida, "comparacion.png"), dpi=110)


def main():
    tabla, curvas = cargar(sys.argv[1:])
    if tabla.empty:
        sys.exit("No hay sesiones analizadas. Corré primero analizar.py.")
    salida = os.path.join(C.CARPETA_SESIONES, "comparacion")
    os.makedirs(salida, exist_ok=True)
    tabla.to_csv(os.path.join(salida, "comparacion.csv"), index=False)
    columnas = ["sesion", "reps_esp", "reps_pc", "snr_dB", "mdf_esp_vs_pc_hz", "mdf_caida_pct", "mdf_p",
                "mnf_caida_pct", "rms_cambio_pct", "estado_final_esp", "iso_mdf_caida_pct"]
    with pd.option_context("display.width", 200, "display.max_columns", 20):
        print(tabla[columnas].to_string(index=False))
    graficar(tabla, curvas, salida)
    print(f"\nArchivos en {salida}")


if __name__ == "__main__":
    main()

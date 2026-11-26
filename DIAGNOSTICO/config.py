"""Configuración compartida por los scripts de diagnóstico.

Todo lo que tiene que coincidir con el firmware (UUIDs, frecuencia de muestreo, formato
del paquete de diagnóstico) y la estética de los gráficos (colores de la app) está acá.
"""
import os
import sys

import matplotlib as mpl

# Que la consola de Windows no se trabe con tildes o flechas
for _flujo in (sys.stdout, sys.stderr):
    try:
        _flujo.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

# =============================================================================
# Firmware (FATIGOMETRO_ESP32.ino + fatiga_dsp.h)
# =============================================================================
NOMBRE_BLE = "MyoSignals"
UUID_DIAGNOSTICO = "8d1f0002-6c3a-4b8e-9a2e-3f1b2c4d5e60"   # señal procesada (ENVIAR_DIAGNOSTICO = true)
UUID_FATIGA = "8d1f0003-6c3a-4b8e-9a2e-3f1b2c4d5e60"        # JSON de resultados
UUID_COMANDO = "8d1f0004-6c3a-4b8e-9a2e-3f1b2c4d5e60"       # 'I' = START, 'S' = STOP

FS = 2000                    # muestras por segundo
MUESTRAS_BLOQUE = 100        # muestras por paquete de diagnóstico (50 ms)
FORMATO_DIAG = "<IfffffffBB100h"   # PaqueteDiagnostico (234 bytes)
CAMPOS_DIAG = ["bloque", "envolvente", "limite_recorte", "reposo", "maximo", "minimo",
               "umbral_pico", "umbral_valle", "etapa", "buscando_pico"]
ESCALA_MUESTRA = 10.0        # la ESP manda décimas de mV
ENV_VENTANA_S = 0.5          # envolvente RMS de la ESP (centrada)
ETAPAS = ["ESPERANDO", "RELAJA", "CALIBRANDO", "MIDIENDO", "SIN REFERENCIA"]

# =============================================================================
# Análisis offline (mismo método que la ESP, para poder compararlos)
# =============================================================================
BANDA_HZ = (20, 450)         # banda para MDF y MNF
NFFT = 512                   # tramos de Welch
VENTANA_REP_S = 1.5          # ventana del espectro, centrada en el pico de cada repetición
VENTANA_DESLIZANTE_S = 1.0   # MDF/MNF continuas a lo largo del registro
PASO_DESLIZANTE_S = 0.25
NIVELES_PCT = {"leve": 7, "moderada": 12, "alta": 20}   # caída de la MDF (fatiga_dsp.h)
REPS_REFERENCIA = 3          # la referencia offline = mediana de las primeras 3 repeticiones

CARPETA = os.path.dirname(os.path.abspath(__file__))
CARPETA_SESIONES = os.path.join(CARPETA, "sesiones")

# =============================================================================
# Estética (la de la app y la presentación)
# =============================================================================
NEGRO = "#0E0E0E"
PANEL = "#161616"
GRILLA = "#2A2A2A"
LIMA = "#C6F800"
CLARO = "#F4F4EF"
GRIS = "#A9A9A2"
NARANJA = "#F28C28"
AMARILLO = "#F2B705"
VERDE = "#8FD14F"
GRIS_OSCURO = "#6E6E68"
COLOR_ESTADO = {"esperando": GRIS_OSCURO, "relaja": GRIS_OSCURO, "calibrando": GRIS,
                "sin": VERDE, "leve": LIMA, "moderada": AMARILLO, "alta": NARANJA,
                "sinref": CLARO}
TEXTO_ESTADO = {"esperando": "ESPERANDO START", "relaja": "RELAJÁ EL BRAZO",
                "calibrando": "CALIBRANDO", "sin": "SIN FATIGA", "leve": "FATIGA LEVE",
                "moderada": "FATIGA MODERADA", "alta": "FATIGA ALTA", "sinref": "SIN REFERENCIA"}


def aplicar_estilo():
    """Fondo negro, texto claro, acentos lima. Usa Barlow Condensed / Inter si están instaladas."""
    from matplotlib import font_manager
    disponibles = {f.name for f in font_manager.fontManager.ttflist}
    familia = [f for f in ("Inter", "Barlow Condensed", "DejaVu Sans") if f in disponibles]
    mpl.rcParams.update({
        "figure.facecolor": NEGRO, "axes.facecolor": PANEL, "savefig.facecolor": NEGRO,
        "axes.edgecolor": GRILLA, "axes.labelcolor": GRIS, "axes.titlecolor": CLARO,
        "axes.titleweight": "bold", "axes.titlesize": 13, "axes.titlelocation": "left",
        "axes.grid": True, "grid.color": GRILLA, "grid.linewidth": 0.8,
        "xtick.color": GRIS, "ytick.color": GRIS, "text.color": CLARO,
        "legend.facecolor": PANEL, "legend.edgecolor": GRILLA, "legend.labelcolor": CLARO,
        "legend.fontsize": 9, "font.family": familia, "font.size": 11,
        "axes.spines.top": False, "axes.spines.right": False,
    })


def titulo_fuerte():
    """Fuente para títulos grandes (Barlow Condensed si está, si no la de siempre)."""
    from matplotlib import font_manager
    nombres = {f.name for f in font_manager.fontManager.ttflist}
    return {"family": "Barlow Condensed", "weight": "black", "style": "italic"} \
        if "Barlow Condensed" in nombres else {"weight": "bold"}

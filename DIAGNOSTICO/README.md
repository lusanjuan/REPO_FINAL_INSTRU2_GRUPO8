# Diagnóstico MyoSignals

Tres scripts para la jornada de pruebas:

| Script | Qué hace | Cuándo |
|---|---|---|
| `grabar.py` | Se conecta a la ESP, muestra la señal en vivo (liviana) y **guarda todo en CSV** | Durante cada prueba |
| `analizar.py` | Recalcula en la PC, con la señal completa: MDF, MNF, RMS, tendencias, conteo independiente, comparación con la ESP. Hace los gráficos y un `diapo.png` | Se corre solo al cerrar `grabar.py` (o a mano) |
| `comparar.py` | Junta todas las sesiones en una tabla y un gráfico | Al final del día |

`config.py` tiene lo que tiene que coincidir con el firmware (UUIDs, 2000 Hz, formato del paquete) y los colores.

## Antes de empezar

```
pip install -r requirements.txt
```

En la ESP: `ENVIAR_DIAGNOSTICO = true` (en el .ino). **Cerrá la app web** (si está conectada, la PC no encuentra la ESP).

## Grabar una prueba

```
python grabar.py --etiqueta iso_lu --persona Lu --peso 4 --ejercicio "isometrico 90°"
```

Al conectarse manda START solo (usá `--sin-start` si no querés). En la ventana:

- **S** = START  ·  **X** = STOP  ·  **M** = marca ("acá sentí fatiga")  ·  **Q** = terminar

En la terminal aparece una línea por repetición y, cada 5 s, el estado del enlace (tienen que llegar ~20 paquetes/s y 0 perdidos).

Todo queda en `sesiones/<fecha>_<etiqueta>/`, y al cerrar se corre `analizar.py` sobre esa carpeta.

## Analizar / comparar

```
python analizar.py ultima
python analizar.py sesiones/20261127_101500_iso_lu
python comparar.py              (todas)
python comparar.py iso          (solo las que tengan "iso" en el nombre)
```

Resultados en `<sesion>/analisis/`: `resumen.txt` (las conclusiones con números), `reps.csv`, `ventanas.csv`, `senal.png`, `mdf_reps.png`, `espectros.png`, `validacion_esp.png`, `diapo.png` (1920×1080).

## Protocolo sugerido

Una sesión por prueba, con `--etiqueta` clara. Entre pruebas de fatiga, descanso de ≥ 5 min.

| # | Prueba | Etiqueta | Qué valida | Qué mirar |
|---|---|---|---|---|
| 1 | Reposo 20 s, brazo apoyado (`--sin-start`) | `reposo` | Ruido del sistema | `ruido_base_mV` bajo (antes veíamos 6–8 mV) |
| 2 | Reposo con OLED apagada (`PANTALLA_ACTIVA = false`, o desconectada) | `reposo_sin_oled` | ¿La OLED mete ruido? | Comparar ruido y espectro con la 1 |
| 3 | 5 contracciones fuertes y cortas | `conteo` | Conteo de reps | ESP = PC = 5 |
| 4 | Isométrica sostenida hasta no poder (codo 90°) | `iso` | **La MDF cae con la fatiga** (la prueba clásica) | Ventanas de 1 s: caída % y p |
| 5 | Serie liviana, ~10 reps cómodas | `liviana` | Control: sin fatiga no debería caer | Caída chica, p alto |
| 6 | Serie pesada al fallo, ritmo constante | `pesada` | Detección en dinámico | Caída, nivel final, marca M vs estado ESP |
| 7 | (opcional) repetir 4 y 6 con otra persona | `iso_franco`, ... | Reproducibilidad | `comparar.py` |

Para el pitch: `diapo.png` de la prueba 6 (o la 4) y `comparacion.png`.

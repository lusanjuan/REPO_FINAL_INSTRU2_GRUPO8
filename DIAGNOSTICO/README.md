# Diagnóstico MyoSignals

Scripts para probar la ESP, analizar las grabaciones en la PC y comparar métodos de fatiga.

## Qué hay en esta carpeta

| Archivo | Para qué |
|---|---|
| `config.py` | lo que tiene que coincidir con la ESP (nombre BLE, UUIDs, 2000 Hz, banda 20–450 Hz) y los colores |
| `grabar.py` | se conecta a la ESP por Bluetooth, muestra la señal en vivo y la guarda en `sesiones/` |
| `analizar.py` | analiza una grabación o un CSV: MDF, MNF, RMS, repeticiones, gráficos, y compara los métodos |
| `metodos.py` | los 6 métodos de fatiga que compara `analizar.py` (lo usa solo, no se corre) |
| `comparar.py` | junta todas las grabaciones analizadas en una tabla y gráficos |
| `requirements.txt` | las librerías |
| `sesiones/` | acá quedan las grabaciones y los resultados (se crea solo) |

## Una sola vez

```
pip install -r requirements.txt
```

## Uso de todos los días (desde la carpeta del repo)

### A) Ya tengo CSV grabados (formato `usuario,sexo,modo,muestra,t_s,mV,fatiga`)

1. Poné los CSV en `DIAGNOSTICO\sesiones\csv\`. Que el nombre diga qué es, por ejemplo
   `fatiga_lu.csv`, `fatiga_franco.csv`, `control_lu.csv` (control = sin llegar a la fatiga).
2. Analizalos todos:
   ```
   .\.venv\Scripts\python.exe .\DIAGNOSTICO\analizar.py ".\DIAGNOSTICO\sesiones\csv\*.csv"
   ```
3. Compará, diciendo qué palabra tienen los controles en el nombre:
   ```
   .\.venv\Scripts\python.exe .\DIAGNOSTICO\comparar.py --control control
   ```

### B) Grabar con la ESP en vivo

En la ESP: `ENVIAR_DIAGNOSTICO = true`. Cerrá la app web (si está conectada, la PC no ve la ESP).
```
.\.venv\Scripts\python.exe .\DIAGNOSTICO\grabar.py --etiqueta fatiga_lu --persona Lu --peso 4
```
Teclas: **S** START · **X** STOP · **M** marca ("acá sentí fatiga") · **Q** terminar.
Al cerrar se analiza sola. Después, `comparar.py` como arriba.

## Dónde mirar los resultados

Por cada grabación, en `<nombre>\analisis\`:
- `resumen.txt`: las conclusiones con números (empezá por acá)
- `senal.png`: señal, envolvente y repeticiones
- `mdf_reps.png`: MDF, MNF y RMS por repetición
- `metodos.png`: los 6 métodos lado a lado
- `diapo.png`: figura lista para la presentación

Entre todas, en `sesiones\comparacion\`:
- `metodos_comparacion.png` y `.csv`: qué método avisa a tiempo y cuál da falsas alarmas

## Los métodos que se comparan

Cada uno simula la ESP repetición por repetición y agrega un cambio al anterior:

| | Método | Qué cambia |
|---|---|---|
| M0 | actual (ESP) | lo que hace hoy la ESP |
| M1 | + Theil-Sen | recta robusta: una rep rara casi no la mueve |
| M2 | + ref de la recta | la referencia sale de la recta, no de 3 reps sueltas |
| M3 | + calidad | descarta reps saturadas, con movimiento o con poca señal |
| M4 | + espectro limpio | resta el ruido de reposo y rellena el pozo del notch de 50 Hz |
| M5 | + JASA | solo sube de nivel si la RMS no bajó |

Y cada uno con 4 señales: MDF · MNF (umbrales × 0,7) · MDF+MNF (caen las dos) · MDF o MNF (cae una).

El mejor: llega a ALTA antes de la marca del usuario, en todas las sesiones con fatiga, y sin
falsas alarmas en los controles. Los umbrales están al principio de `metodos.py`.

## Protocolo para grabar

| Prueba | Nombre sugerido | Para qué |
|---|---|---|
| 20 s quieto, brazo apoyado | `reposo_...` | ruido del sistema |
| 6–8 reps con el mismo peso, lejos del fallo | `control_...` | ver que NO avise (falsas alarmas) |
| Serie hasta el fallo, ritmo parejo, marcando la fatiga | `fatiga_...` | ver que avise a tiempo |
| Isométrica sostenida hasta no poder | `iso_...` | la prueba clásica de que la MDF baja |

Hacen falta varios controles: sin ellos no se puede saber si un método da falsas alarmas.

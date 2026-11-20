"""Prueba sin interfaz: camara simulada -> vision -> PIC simulado -> planta.
Uso:  python pruebas/prueba_lazo_simulado.py   (desde la raiz del repo)
Imprime el error de medicion y la respuesta a escalones de referencia."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import numpy as np
from vision import MedidorAltura
from simulador import Planta, PICSimulado, CamaraSimulada

FPS = 30
reloj = {"t": 0.0}
planta = Planta(semilla=3)
pic = PICSimulado(reloj_ms=lambda: int(reloj["t"] * 1000))
cam = CamaraSimulada(planta, reloj_s=lambda: reloj["t"])
med = MedidorAltura()

def enviar(s):
    pic.escribir(s + "\n")

enviar("R")
plan = [(0, 400), (12, 700), (24, 1000), (34, 100), (46, None)]
errores_med, registro = [], []
sp_actual, i_plan = None, 0
while reloj["t"] < plan[-1][0]:
    if i_plan < len(plan) - 1 and reloj["t"] >= plan[i_plan][0]:
        sp_actual = plan[i_plan][1]; enviar(f"S{sp_actual}"); i_plan += 1
    reloj["t"] += 1 / FPS
    ok, frame = cam.read()
    r = med.procesar(frame)
    if r.pelota_ok:
        enviar(f"M{int(round(r.altura_mm))}")
        errores_med.append(r.altura_cruda_mm - planta.h * 1000)
    else:
        enviar("N")
    planta.u = pic.u_actual
    registro.append((reloj["t"], planta.h * 1000, sp_actual, pic.u_actual, pic.estado))

e = np.array(errores_med)
print(f"Medicion: {len(e)} cuadros con pelota, error medio {e.mean():.1f} mm, desv {e.std():.1f} mm, max {abs(e).max():.1f} mm")
reg = np.array([(t, h, sp, u, st) for t, h, sp, u, st in registro])
for (t0, sp), (t1, _) in zip(plan[:-1], plan[1:]):
    seg = reg[(reg[:, 0] >= t0) & (reg[:, 0] < t1)]
    final = seg[seg[:, 0] > t1 - 3]
    sobre = seg[:, 1].max() - sp if sp > (seg[0, 1] if len(seg) else 0) else sp - seg[:, 1].min()
    dentro = np.where(abs(seg[:, 1] - sp) > 0.05 * max(sp, 100))[0]
    t_est = seg[dentro[-1], 0] - t0 if len(dentro) else 0
    print(f"SP {sp:4d} mm | ultimos 3 s: media {final[:,1].mean():6.1f} mm, desv {final[:,1].std():4.1f} mm, "
          f"u {final[:,3].mean():5.0f} | sobrepaso {sobre:5.1f} mm | t asentamiento(5%) {t_est:4.1f} s")
print("Estados vistos:", sorted(set(int(s) for s in reg[:, 4])))

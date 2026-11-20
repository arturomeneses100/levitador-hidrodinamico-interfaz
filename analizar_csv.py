"""
analizar_csv.py - Analisis de los registros CSV de la interfaz.

  python analizar_csv.py metricas    registros/levitador_XXXX.csv
      Para cada cambio de referencia: sobrepaso, tiempo de asentamiento (5 %),
      error en estado estacionario e IAE.

  python analizar_csv.py feedforward registros/levitador_XXXX.csv
      Usa los tramos en MODO MANUAL (lazo abierto) donde el PWM se mantuvo
      fijo y la pelota se estabilizo, y ajusta la recta  u = B + W * h.
      Los valores B y W se escriben en la interfaz (Feedforward B y W).

  Agrega --graficar para ver la grafica (requiere matplotlib).
"""
import csv
import sys

import numpy as np


def leer(ruta):
    with open(ruta, encoding="utf-8") as f:
        filas = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in filas])
    return {k: col(k) for k in ("t_s", "altura_mm", "referencia_mm", "u_pwm", "modo", "estado")}


def metricas(d):
    t, h, sp, est = d["t_s"], d["altura_mm"], d["referencia_mm"], d["estado"]
    cambios = [0] + [i for i in range(1, len(sp)) if abs(sp[i] - sp[i - 1]) > 5] + [len(sp)]
    print(f"{'t0 [s]':>7} {'de':>6} {'a':>6} {'sobrepaso':>10} {'t_asent':>8} {'e_ss':>7} {'IAE':>8}")
    for a, b in zip(cambios[:-1], cambios[1:]):
        if b - a < 30 or np.all(d["modo"][a:b] == 0):
            continue                       # tramos cortos o en lazo abierto
        ref = sp[a]
        ini = sp[a - 1] if a > 0 else h[a]
        tramo_h, tramo_t = h[a:b], t[a:b]
        if np.any(est[a:b] != 2):
            nota = "  (hubo perdida/caida de la pelota)"
        else:
            nota = ""
        sube = ref >= ini
        sob = (tramo_h.max() - ref) if sube else (ref - tramo_h.min())
        sob_pct = 100 * max(0.0, sob) / max(1.0, abs(ref - ini))
        banda = 0.05 * max(abs(ref - ini), 20)
        fuera = np.where(np.abs(tramo_h - ref) > banda)[0]
        ts = (tramo_t[fuera[-1]] - tramo_t[0]) if len(fuera) else 0.0
        n_fin = max(1, len(tramo_h) // 5)
        ess = ref - tramo_h[-n_fin:].mean()
        integrar = getattr(np, "trapezoid", None) or np.trapz
        iae = integrar(np.abs(ref - tramo_h), tramo_t)
        print(f"{tramo_t[0]:7.1f} {ini:6.0f} {ref:6.0f} {max(0, sob):6.0f} mm ({sob_pct:3.0f}%) "
              f"{ts:6.1f} s {ess:6.1f} {iae:8.0f}{nota}")


def feedforward(d):
    t, h, u, modo = d["t_s"], d["altura_mm"], d["u_pwm"], d["modo"]
    puntos = []
    i = 0
    while i < len(u):
        j = i
        while j < len(u) and u[j] == u[i] and modo[j] == 0:
            j += 1
        if modo[i] == 0 and j - i > 15:
            # ultimo 40 % del tramo (ya estable)
            k = i + int(0.6 * (j - i))
            hh = h[k:j]
            if hh.mean() > 30 and hh.std() < 25:
                puntos.append((hh.mean(), u[i]))
        i = max(j, i + 1)
    if len(puntos) < 3:
        print("Se necesitan al menos 3 tramos estables en modo MANUAL con PWM fijo "
              "(por ejemplo 400, 450, 500, ... esperando ~5 s en cada uno).")
        return None
    P = np.array(puntos)
    W, B = np.polyfit(P[:, 0], P[:, 1], 1)
    print("Tramos estables (altura media -> PWM):")
    for hm, uu in P:
        print(f"   {hm:6.0f} mm  ->  {uu:5.0f}")
    res = P[:, 1] - (B + W * P[:, 0])
    print(f"\nFeedforward ajustado:  u = {B:.0f} + {W:.3f} * h     (residuo max {np.abs(res).max():.0f} cuentas)")
    print(f"Feedforward ESTATICO completo:  B = {B:.0f}   W = {W:.3f}")
    # Feedforward parcial: la recta completa da una "patada" fuerte en cada
    # escalon de referencia. Se recomienda usar una fraccion de la pendiente y
    # ajustar B para que coincida a media altura (500 mm); la integral hace el resto.
    f, hm = 0.35, 500.0
    Wp = f * W
    Bp = B + (W - Wp) * hm
    print(f"Recomendado (35 % de la pendiente, igual a 500 mm):  B = {Bp:.0f}   W = {Wp:.3f}")
    return P, B, W


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("metricas", "feedforward"):
        print(__doc__)
        return
    d = leer(sys.argv[2])
    graficar = "--graficar" in sys.argv
    if sys.argv[1] == "metricas":
        metricas(d)
        if graficar:
            import matplotlib.pyplot as plt
            fig, (a1, a2) = plt.subplots(2, 1, sharex=True)
            a1.plot(d["t_s"], d["referencia_mm"], "--", label="referencia")
            a1.plot(d["t_s"], d["altura_mm"], label="altura")
            a1.set_ylabel("mm")
            a1.legend()
            a2.plot(d["t_s"], d["u_pwm"])
            a2.set_ylabel("PWM")
            a2.set_xlabel("t [s]")
            plt.show()
    else:
        r = feedforward(d)
        if r and graficar:
            import matplotlib.pyplot as plt
            P, B, W = r
            x = np.linspace(0, 1000, 50)
            plt.plot(P[:, 0], P[:, 1], "o", label="ensayo")
            plt.plot(x, B + W * x, label=f"u = {B:.0f} + {W:.3f} h")
            plt.xlabel("altura [mm]")
            plt.ylabel("PWM")
            plt.legend()
            plt.show()


if __name__ == "__main__":
    main()

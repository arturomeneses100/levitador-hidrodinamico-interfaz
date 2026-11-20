"""
simulador.py - Modo simulacion (sin hardware).

Contiene:
  * Planta       : modelo fisico de bomba QWORK 12V 800 L/h + chorro + pelota.
  * PICSimulado  : replica en Python de la logica de firmware/levitador.c
                   (mismo protocolo, mismos modos y estados).
  * CamaraSimulada: genera imagenes sinteticas (fondo negro, marcadores ArUco,
                   pelota naranja, chorro y gotas) para probar la vision.

Sirve para practicar la interfaz y la sintonizacion antes de mojar nada.
El modelo es aproximado: los valores reales de bias y ganancias se ajustan
con el equipo fisico.
"""
from __future__ import annotations

import math
import random
import threading
import time
from collections import deque

import cv2
import numpy as np

from vision import generar_marcador

G = 9.81
RHO = 1000.0


# ===========================================================================
#  Planta
# ===========================================================================
class Planta:
    """Bomba brushless + chorro vertical libre + pelota de unicel.

    - La bomba no gira por debajo de ~35 % de PWM (driver interno) y arranca
      ya con cierta velocidad: por eso existe una altura minima controlable.
    - Velocidad del chorro en la boquilla v0 ~ proporcional a la velocidad de
      la bomba (5.8 m/s a 100 % con boquilla de 5 mm).
    - Fuerza sobre la pelota F = rho * A0 * v0 * v_rel (cantidad de movimiento
      del chorro), con v_rel la velocidad del chorro a esa altura menos la de
      la pelota.
    """

    def __init__(self, diam_boquilla_mm=5.0, masa_g=10.0, v0_max=5.8,
                 umbral_arranque=0.35, vel_min=0.40, tau_bomba=0.30,
                 retardo_s=0.08, ruido=0.08, semilla=None):
        self.A0 = math.pi * (diam_boquilla_mm / 2000.0) ** 2
        self.m = masa_g / 1000.0
        self.v0_max = v0_max
        self.umbral = umbral_arranque
        self.vel_min = vel_min
        self.tau = tau_bomba
        self.retardo = retardo_s
        self.ruido = ruido
        self.rng = random.Random(semilla)
        self.reiniciar()

    def reiniciar(self):
        self.u = 0                 # PWM 0..1023 aplicado
        self.n = 0.0               # velocidad relativa de la bomba 0..1
        self.h = 0.0               # altura base de la pelota [m]
        self.v = 0.0               # velocidad de la pelota [m/s]
        self.lat = 0.0             # desviacion lateral [m] (solo visual)
        self.f_ruido = 0.0
        self._cola = deque()       # retardo de transporte
        self._t = 0.0

    def _n_objetivo(self, u):
        d = max(0.0, min(1.0, u / 1023.0))
        if d < self.umbral:
            return 0.0
        return self.vel_min + (1 - self.vel_min) * (d - self.umbral) / (1 - self.umbral)

    def avanzar(self, dt_total: float, paso: float = 0.001):
        k = max(1, int(round(dt_total / paso)))
        dt = dt_total / k
        for _ in range(k):
            self._t += dt
            # bomba (primer orden)
            self.n += (self._n_objetivo(self.u) - self.n) * dt / self.tau
            self._cola.append((self._t, self.n))
            n_ret = 0.0
            while self._cola and self._cola[0][0] <= self._t - self.retardo:
                n_ret = self._cola.popleft()[1]
                self._n_ret = n_ret
            n_ret = getattr(self, "_n_ret", 0.0)
            v0 = self.v0_max * n_ret
            # chorro a la altura de la pelota
            vh2 = v0 * v0 - 2 * G * self.h
            vh = math.sqrt(vh2) if vh2 > 0 else 0.0
            vrel = max(0.0, vh - self.v)
            f = RHO * self.A0 * v0 * vrel if vh > 0 else 0.0
            # turbulencia (ruido filtrado proporcional al peso)
            self.f_ruido += (self.rng.gauss(0, 1) - self.f_ruido) * dt / 0.05
            f += self.ruido * self.m * G * self.f_ruido * (1.0 if f > 0 else 0.0)
            arrastre = 0.5 * 1.2 * 0.47 * 0.00196 * self.v * abs(self.v)
            a = (f - self.m * G - arrastre) / self.m
            self.v += a * dt
            self.h += self.v * dt
            if self.h <= 0:
                self.h, self.v = 0.0, max(0.0, self.v)
            if self.h > 1.6:
                self.h, self.v = 1.6, min(0.0, self.v)
            self.lat = 0.002 * math.sin(self._t * 7.0) + 0.001 * self.f_ruido


# ===========================================================================
#  PIC simulado (misma logica que firmware/levitador.c)
# ===========================================================================
MODO_MANUAL, MODO_ONOFF, MODO_P, MODO_PI, MODO_PID = range(5)
EST_PARO, EST_SIN_ENLACE, EST_CONTROL, EST_PERDIDA, EST_CAIDA = range(5)


class PICSimulado:
    T_ENLACE_MS = 300
    T_PERDIDA_MS = 600
    VISTAS_REANUDA = 5
    T_TELEM_MS = 200
    V_ARRANQUE = 300.0

    def __init__(self, reloj_ms=None):
        self.reloj_ms = reloj_ms or (lambda: int(time.monotonic() * 1000))
        self.modo = MODO_PID
        self.sp_mm = 400
        self.kp, self.ki, self.kd, self.alfa_d = 0.25, 0.50, 0.04, 0.30
        self.bias, self.amp_onoff, self.hist_mm = 467, 120, 20
        self.ff_pend = 0.15
        self.v_ref = 0
        self.u_manual, self.u_max = 0, 1023
        self.estado = EST_PARO
        self.sp_ef = 0.0
        self._reiniciar()
        self.h_mm = 0
        self.u_actual = 0
        self.dt_ms = 0
        self.vistas = 0
        ahora = self.reloj_ms()
        self.t_ultimo_msg = ahora
        self.t_ultima_med = ahora
        self.t_ultima_vista = ahora
        self.t_ult_telem = ahora
        self.salida: list[str] = []
        self._buf = ""
        self._telem = False
        self.salida.append("E,INICIO Levitador hidrodinamico v1.0 (SIMULADO)")
        self._config()

    # -- utilidades
    def _reiniciar(self):
        self.integral = 0.0
        self.deriv_f = 0.0
        self.h_prev = 0
        self.h_prev_valida = False
        self.onoff_alto = False
        self.arrancando = False

    def _pwm(self, u):
        u = min(float(self.u_max), max(0.0, u))
        self.u_actual = int(u)

    def _config(self):
        self.salida.append(
            f"C,{self.modo},{self.sp_mm},{self.kp:.4f},{self.ki:.4f},{self.kd:.4f},"
            f"{self.alfa_d:.3f},{self.bias},{self.amp_onoff},{self.hist_mm},"
            f"{self.u_manual},{self.u_max},{self.ff_pend:.3f},{self.v_ref}")

    def _telemetria(self):
        sp_rep = int(round(self.sp_ef)) if self.estado in (EST_CONTROL, EST_PERDIDA) else self.sp_mm
        self.salida.append(f"T,{self.h_mm},{sp_rep},{self.u_actual},"
                           f"{self.modo},{self.estado},{self.dt_ms}")
        self.t_ult_telem = self.reloj_ms()

    def _paro(self):
        self.estado = EST_PARO
        self._pwm(0)
        self._reiniciar()

    def _armar(self):
        self._reiniciar()
        self.vistas = 0
        if self.modo == MODO_MANUAL:
            self.estado = EST_CONTROL
            self._pwm(self.u_manual)
        else:
            self.estado = EST_CAIDA
            self._pwm(0)

    def _cambiar_modo(self, nuevo):
        self.modo = nuevo
        self.deriv_f = 0.0
        self.onoff_alto = False
        self.integral = float(self.u_actual) - (self.bias + self.ff_pend * self.sp_ef)
        if self.modo == MODO_MANUAL and self.estado not in (EST_PARO, EST_SIN_ENLACE):
            self.estado = EST_CONTROL
            self._pwm(self.u_manual)

    def _controlar(self, h, ahora):
        self.dt_ms = (ahora - self.t_ultima_med) if self.h_prev_valida else 33
        dt = min(0.25, max(0.005, self.dt_ms / 1000.0))
        # Rampa de la referencia: al arrancar siempre (V_ARRANQUE); despues,
        # solo si v_ref > 0 (v_ref = 0 -> escalones puros para las practicas)
        v = self.V_ARRANQUE if self.arrancando else float(self.v_ref)
        if v > 0:
            paso = v * dt
            if self.sp_ef < self.sp_mm - paso:
                self.sp_ef += paso
            elif self.sp_ef > self.sp_mm + paso:
                self.sp_ef -= paso
            else:
                self.sp_ef = float(self.sp_mm)
        else:
            self.sp_ef = float(self.sp_mm)
        if self.arrancando and self.sp_ef == self.sp_mm:
            self.arrancando = False
        e = self.sp_ef - h
        u_ff = self.bias + self.ff_pend * self.sp_ef
        m = self.modo
        if m == MODO_MANUAL:
            u = self.u_manual
        elif m == MODO_ONOFF:
            if e > self.hist_mm:
                self.onoff_alto = True
            elif e < -self.hist_mm:
                self.onoff_alto = False
            u = u_ff + (self.amp_onoff if self.onoff_alto else -self.amp_onoff)
        elif m == MODO_P:
            u = u_ff + self.kp * e
        else:
            u = u_ff + self.kp * e + self.integral
            if m == MODO_PID:
                if self.h_prev_valida:
                    d = -(h - self.h_prev) / dt
                    self.deriv_f += self.alfa_d * (d - self.deriv_f)
                u += self.kd * self.deriv_f
            if not self.arrancando and not ((u >= self.u_max and e > 0) or (u <= 0 and e < 0)):
                self.integral += self.ki * e * dt
                self.integral = max(-1023.0, min(1023.0, self.integral))
        self._pwm(u)

    def _medicion(self, h, ahora):
        self.h_mm = h
        self.t_ultima_vista = ahora
        self._telem = True
        if self.estado in (EST_PARO, EST_SIN_ENLACE):
            return
        if self.estado == EST_CAIDA:
            self.vistas = min(255, self.vistas + 1)
            if self.vistas < self.VISTAS_REANUDA:
                return
            self._reiniciar()
            self.estado = EST_CONTROL
            self.sp_ef = float(h)
            self.arrancando = True
        if self.estado == EST_PERDIDA:
            self.estado = EST_CONTROL
        self._controlar(h, ahora)
        self.h_prev = h
        self.h_prev_valida = True
        self.t_ultima_med = ahora

    def _sin_pelota(self):
        self._telem = True
        self.vistas = 0
        if self.modo == MODO_MANUAL:
            return
        if self.estado == EST_CONTROL:
            self.estado = EST_PERDIDA

    @staticmethod
    def _entero(s, vmin, vmax):
        try:
            v = int(float(s))
        except ValueError:
            v = 0
        return max(vmin, min(vmax, v))

    @staticmethod
    def _real(s):
        try:
            return float(s)
        except ValueError:
            return 0.0

    def _linea(self, s):
        ahora = self.reloj_ms()
        self.t_ultimo_msg = ahora
        if self.estado == EST_SIN_ENLACE:
            self._armar()
        cmd, arg = s[0], s[1:]
        if cmd == "M":
            self._medicion(self._entero(arg, -200, 1500), ahora)
        elif cmd == "N":
            self._sin_pelota()
        elif cmd == "S":
            self.sp_mm = self._entero(arg, 0, 1000)
        elif cmd == "P":
            self.kp = self._real(arg)
        elif cmd == "I":
            self.ki = self._real(arg)
        elif cmd == "D":
            self.kd = self._real(arg)
        elif cmd == "F":
            self.alfa_d = min(1.0, max(0.01, self._real(arg)))
        elif cmd == "B":
            self.bias = self._entero(arg, 0, 1023)
        elif cmd == "V":
            self.v_ref = self._entero(arg, 0, 5000)
        elif cmd == "W":
            self.ff_pend = min(2.0, max(0.0, self._real(arg)))
        elif cmd == "A":
            self.amp_onoff = self._entero(arg, 0, 1023)
        elif cmd == "H":
            self.hist_mm = self._entero(arg, 0, 300)
        elif cmd == "U":
            self.u_manual = self._entero(arg, 0, 1023)
            if self.modo == MODO_MANUAL and self.estado == EST_CONTROL:
                self._pwm(self.u_manual)
        elif cmd == "L":
            self.u_max = self._entero(arg, 0, 1023)
        elif cmd == "O":
            self._cambiar_modo(self._entero(arg, 0, 4))
        elif cmd == "Z":
            self.integral = 0.0
        elif cmd == "R":
            self._armar()
            self.salida.append("E,ARMADO")
        elif cmd == "X":
            self._paro()
            self.salida.append("E,PARO_PC")
        elif cmd == "?":
            self._config()

    # -- interfaz publica (igual que un puerto serie)
    def escribir(self, texto: str):
        self._buf += texto
        while True:
            pos = min([p for p in (self._buf.find("\n"), self._buf.find("\r")) if p >= 0], default=-1)
            if pos < 0:
                break
            linea, self._buf = self._buf[:pos].strip(), self._buf[pos + 1:]
            if linea:
                self._linea(linea)
        self.tick()

    def tick(self):
        ahora = self.reloj_ms()
        if self.estado not in (EST_PARO, EST_SIN_ENLACE) and ahora - self.t_ultimo_msg > self.T_ENLACE_MS:
            self.estado = EST_SIN_ENLACE
            self._pwm(0)
            self._reiniciar()
            self.salida.append("E,SIN_ENLACE")
        if self.estado == EST_PERDIDA and ahora - self.t_ultima_vista > self.T_PERDIDA_MS:
            self.estado = EST_CAIDA
            self.vistas = 0
            self._pwm(0)
            self._reiniciar()
        if self._telem or ahora - self.t_ult_telem >= self.T_TELEM_MS:
            self._telem = False
            self._telemetria()

    def leer_lineas(self) -> list[str]:
        out, self.salida = self.salida, []
        return out


# ===========================================================================
#  Camara simulada
# ===========================================================================
class CamaraSimulada:
    """Genera cuadros de 480x640 (vertical) como si la webcam estuviera girada
    y la imagen ya corregida (rotacion = 0 en la configuracion)."""

    ANCHO, ALTO = 480, 640

    def __init__(self, planta: Planta, reloj_s=None, px_por_mm=0.55):
        self.planta = planta
        self.reloj_s = reloj_s or time.monotonic
        self.k = px_por_mm
        self.y0 = 590                 # y del nivel de la boquilla (px)
        self.x_chorro = 260
        self.x_poste = 120
        self._t = self.reloj_s()
        self.rng = np.random.default_rng(1)
        self.lock = threading.Lock()
        self._fondo = self._crear_fondo()

    def _crear_fondo(self):
        img = np.full((self.ALTO, self.ANCHO, 3), 18, np.uint8)
        # poste
        cv2.rectangle(img, (self.x_poste - 6, 20), (self.x_poste + 6, self.ALTO), (45, 45, 45), -1)
        lado = int(80 * self.k)                       # marcador de 80 mm
        borde = int(12 * self.k)
        for id_m, alt_mm in ((0, 0), (1, 1000)):
            cy = int(self.y0 - alt_mm * self.k)
            cx = self.x_poste
            m = generar_marcador(id_m, lado)
            m = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            x0, y0 = cx - lado // 2, cy - lado // 2
            cv2.rectangle(img, (x0 - borde, y0 - borde), (x0 + lado + borde, y0 + lado + borde),
                          (235, 235, 235), -1)
            img[y0:y0 + lado, x0:x0 + lado] = m
        # boquilla (tubo de cobre)
        cv2.rectangle(img, (self.x_chorro - 4, self.y0), (self.x_chorro + 4, self.ALTO),
                      (60, 110, 170), -1)
        # charola/embudo
        cv2.line(img, (self.x_chorro - 95, self.y0 + 10), (self.x_chorro - 6, self.y0 + 40), (70, 70, 70), 3)
        cv2.line(img, (self.x_chorro + 95, self.y0 + 10), (self.x_chorro + 6, self.y0 + 40), (70, 70, 70), 3)
        return img

    def isOpened(self):
        return True

    def release(self):
        pass

    def set(self, *_):
        return True

    def read(self):
        ahora = self.reloj_s()
        with self.lock:
            dt = ahora - self._t
            self._t = ahora
            if dt > 0:
                self.planta.avanzar(min(dt, 0.2))
            h, lat, n = self.planta.h, self.planta.lat, self.planta.n
        img = self._fondo.copy()
        r = 25 * self.k
        cx = self.x_chorro + lat * 1000 * self.k
        cy = self.y0 - (h * 1000 + 25) * self.k
        # chorro
        if n > 0.05:
            capa = img.copy()
            alto_chorro = (self.planta.v0_max * n) ** 2 / (2 * G) * 1000 * self.k
            y_fin = int(max(cy, self.y0 - alto_chorro))
            cv2.line(capa, (self.x_chorro, self.y0), (int(cx), y_fin), (230, 200, 170), 4)
            img = cv2.addWeighted(capa, 0.5, img, 0.5, 0)
            # gotas alrededor de la pelota
            for _ in range(25):
                gx = int(cx + self.rng.normal(0, 18))
                gy = int(cy + self.rng.normal(0, 14))
                cv2.circle(img, (gx, gy), int(self.rng.integers(1, 3)), (235, 235, 240), -1)
        # pelota naranja con sombreado
        cv2.circle(img, (int(cx), int(cy)), int(r), (0, 110, 255), -1)
        cv2.circle(img, (int(cx - r / 3), int(cy - r / 3)), int(r / 3), (60, 160, 255), -1)
        ruido = self.rng.normal(0, 4, img.shape).astype(np.int16)
        img = np.clip(img.astype(np.int16) + ruido, 0, 255).astype(np.uint8)
        return True, img


class SimulacionCompleta:
    """Une planta + PIC + camara con el reloj real, para la interfaz."""

    def __init__(self):
        self.planta = Planta()
        self.pic = PICSimulado()
        self.camara = CamaraSimulada(self.planta)

    def sincronizar(self):
        """Pasa la u del PIC simulado a la planta."""
        self.planta.u = self.pic.u_actual

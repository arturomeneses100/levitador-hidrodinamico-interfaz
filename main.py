"""
Levitador hidrodinamico - Interfaz de supervision y medicion por camara.

    python main.py

Flujo:
    webcam -> vision (ArUco + color) -> "M<mm>" por serial -> PIC (PID + PWM)
    PIC -> "T,h,sp,u,modo,estado,dt" -> graficas, registro CSV
"""
from __future__ import annotations

import csv
import json
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QKeySequence, QPixmap, QShortcut
from PySide6.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout,
    QGroupBox, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton,
    QSlider, QSpinBox, QSplitter, QVBoxLayout, QWidget,
)
import pyqtgraph as pg

from comunicacion import (
    NOMBRES_ESTADO, NOMBRES_MODO, EnlaceSerial, EnlaceSimulado, Telemetria,
    config_de_linea, listar_puertos,
)
from simulador import SimulacionCompleta
from vision import ConfigVision, MedidorAltura, rotar

DIR = os.path.dirname(os.path.abspath(__file__))
ARCHIVO_CONFIG = os.path.join(DIR, "config.json")
DIR_REGISTROS = os.path.join(DIR, "registros")
VENTANA_S = 30.0

CONFIG_DEFECTO = {
    "camara": 0, "ancho": 640, "alto": 480, "fps": 30,
    "exposicion_auto": False, "exposicion": -6,
    "puerto": "", "simulacion": False,
    "control": {"modo": 4, "sp": 400, "kp": 0.25, "ki": 0.50, "kd": 0.04, "alfa": 0.30,
                "bias": 467, "ff_pend": 0.15, "amp": 120, "hist": 20, "u_manual": 0,
                "u_max": 1023, "v_ref": 0},
    "vision": ConfigVision().a_dict(),
}

COLORES_ESTADO = ["#c62828", "#c62828", "#2e7d32", "#ef6c00", "#6d4c41"]


def cargar_config() -> dict:
    cfg = json.loads(json.dumps(CONFIG_DEFECTO))
    try:
        with open(ARCHIVO_CONFIG, encoding="utf-8") as f:
            guardada = json.load(f)
        for k, v in guardada.items():
            if isinstance(v, dict) and isinstance(cfg.get(k), dict):
                cfg[k].update(v)
            else:
                cfg[k] = v
    except (OSError, json.JSONDecodeError):
        pass
    return cfg


# ===========================================================================
#  Hilo de vision: captura, mide, envia y dibuja
# ===========================================================================
class HiloVision(QThread):
    cuadro = Signal(QImage, object)       # imagen anotada, Resultado
    error = Signal(str)

    def __init__(self, medidor: MedidorAltura):
        super().__init__()
        self.medidor = medidor
        self.fuente = None                 # cv2.VideoCapture o CamaraSimulada
        self.enlace = None                 # EnlaceSerial / EnlaceSimulado
        self.setpoint = 400
        self.ver_mascara = False
        self._vivo = True
        self._lock = threading.Lock()
        self._pendientes = []              # ajustes de camara a aplicar en este hilo
        self.ultimo_frame = None
        self.fps = 0.0

    def poner_fuente(self, fuente):
        with self._lock:
            if self.fuente is not None and self.fuente is not fuente:
                try:
                    self.fuente.release()
                except Exception:
                    pass
            self.fuente = fuente
            self.medidor.reiniciar_filtro()

    def ajustar_camara(self, prop, valor):
        with self._lock:
            self._pendientes.append((prop, valor))

    def detener(self):
        self._vivo = False
        self.wait(1500)
        if self.fuente is not None:
            try:
                self.fuente.release()
            except Exception:
                pass

    def run(self):
        t_prev = time.perf_counter()
        while self._vivo:
            with self._lock:
                fuente = self.fuente
                pendientes, self._pendientes = self._pendientes, []
            if fuente is None:
                self.msleep(50)
                continue
            for prop, valor in pendientes:
                try:
                    fuente.set(prop, valor)
                except Exception:
                    pass
            ok, frame = fuente.read()
            if not ok or frame is None:
                self.error.emit("No se pudo leer la camara")
                self.msleep(200)
                continue
            if isinstance(fuente, cv2.VideoCapture):
                frame = rotar(frame, self.medidor.cfg.rotacion)
            else:
                self.msleep(30)            # la camara simulada no limita los fps

            r = self.medidor.procesar(frame, quiere_mascara=self.ver_mascara)

            enlace = self.enlace
            if enlace is not None and enlace.conectado:
                if r.pelota_ok:
                    enlace.enviar(f"M{int(round(r.altura_mm))}")
                else:
                    enlace.enviar("N")

            self.ultimo_frame = frame.copy()
            if self.ver_mascara and r.mascara is not None:
                vista = cv2.cvtColor(r.mascara, cv2.COLOR_GRAY2BGR)
            else:
                vista = frame
            vista = self.medidor.dibujar(vista, r, self.setpoint)

            ahora = time.perf_counter()
            dt = ahora - t_prev
            t_prev = ahora
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

            rgb = cv2.cvtColor(vista, cv2.COLOR_BGR2RGB)
            h, w = rgb.shape[:2]
            img = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
            self.cuadro.emit(img, r)


# ===========================================================================
#  Etiqueta de video que reporta clics en coordenadas de la imagen
# ===========================================================================
class VistaVideo(QLabel):
    clic = Signal(int, int)

    def __init__(self):
        super().__init__("Sin camara")
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(360, 480)
        self.setStyleSheet("background:#111; color:#aaa;")
        self._tam_img = None

    def mostrar(self, img: QImage):
        self._tam_img = (img.width(), img.height())
        self.setPixmap(QPixmap.fromImage(img).scaled(
            self.width(), self.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def mousePressEvent(self, ev):
        if not self._tam_img or self.pixmap() is None:
            return
        pw, ph = self.pixmap().width(), self.pixmap().height()
        ox, oy = (self.width() - pw) / 2, (self.height() - ph) / 2
        p = ev.position()
        x, y = p.x() - ox, p.y() - oy
        if 0 <= x < pw and 0 <= y < ph:
            iw, ih = self._tam_img
            self.clic.emit(int(x * iw / pw), int(y * ih / ph))


# ===========================================================================
#  Ventana principal
# ===========================================================================
class Ventana(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Levitador hidrodinamico - DUAL ARD / UTHH")
        self.resize(1400, 860)
        self.cfg = cargar_config()
        self.medidor = MedidorAltura(ConfigVision.de_dict(self.cfg["vision"]))
        self.enlace = None
        self.sim = None
        self.ultima_tel: Telemetria | None = None
        self.ultimo_res = None
        self.csv_archivo = None
        self.csv_escritor = None
        self.t0 = time.monotonic()
        n = int(VENTANA_S * 40)
        self.b_t, self.b_h, self.b_sp, self.b_u = (deque(maxlen=n) for _ in range(4))

        self.hilo = HiloVision(self.medidor)
        self.hilo.cuadro.connect(self._nuevo_cuadro)
        self.hilo.error.connect(lambda m: self.statusBar().showMessage(m, 2000))

        self._construir()
        self._cargar_widgets()

        self.timer_serial = QTimer(self)
        self.timer_serial.timeout.connect(self._leer_serial)
        self.timer_serial.start(20)
        self.timer_graf = QTimer(self)
        self.timer_graf.timeout.connect(self._actualizar_graficas)
        self.timer_graf.start(100)

        atajo = QShortcut(QKeySequence(Qt.Key_Space), self)
        atajo.activated.connect(self._paro)
        self.hilo.start()
        if self.cfg.get("simulacion"):
            self.chk_sim.setChecked(True)

    # ------------------------------------------------------------ interfaz
    def _construir(self):
        central = QWidget()
        self.setCentralWidget(central)
        raiz = QHBoxLayout(central)
        div = QSplitter(Qt.Horizontal)
        raiz.addWidget(div)

        # --- columna izquierda: video
        izq = QWidget()
        li = QVBoxLayout(izq)
        self.video = VistaVideo()
        self.video.clic.connect(self._clic_video)
        li.addWidget(self.video, 1)
        fila = QHBoxLayout()
        self.chk_mascara = QCheckBox("Ver mascara")
        self.chk_mascara.toggled.connect(lambda v: setattr(self.hilo, "ver_mascara", v))
        self.chk_calibrar = QCheckBox("Calibrar color: clic en la pelota")
        fila.addWidget(self.chk_mascara)
        fila.addWidget(self.chk_calibrar)
        fila.addStretch()
        li.addLayout(fila)
        div.addWidget(izq)

        # --- columna central: graficas
        centro = QWidget()
        lc = QVBoxLayout(centro)
        pg.setConfigOptions(antialias=True)
        self.g_h = pg.PlotWidget(title="Altura de la pelota")
        self.g_h.setLabel("left", "altura", units="mm")
        self.g_h.setLabel("bottom", "tiempo", units="s")
        self.g_h.setYRange(0, 1050)
        self.g_h.addLegend()
        self.g_h.showGrid(x=True, y=True, alpha=0.3)
        self.c_sp = self.g_h.plot(pen=pg.mkPen("#43a047", width=2, style=Qt.DashLine), name="referencia")
        self.c_h = self.g_h.plot(pen=pg.mkPen("#ff9800", width=2), name="altura")
        self.g_u = pg.PlotWidget(title="Senal de control (PWM bomba)")
        self.g_u.setLabel("left", "u", units="cuentas")
        self.g_u.setLabel("bottom", "tiempo", units="s")
        self.g_u.setYRange(0, 1050)
        self.g_u.showGrid(x=True, y=True, alpha=0.3)
        self.c_u = self.g_u.plot(pen=pg.mkPen("#1e88e5", width=2))
        self.g_u.setXLink(self.g_h)
        lc.addWidget(self.g_h, 3)
        lc.addWidget(self.g_u, 2)
        div.addWidget(centro)

        # --- columna derecha: controles
        der = QWidget()
        der.setMaximumWidth(380)
        ld = QVBoxLayout(der)

        # estado grande
        self.lbl_estado = QLabel("DESCONECTADO")
        self.lbl_estado.setAlignment(Qt.AlignCenter)
        self.lbl_estado.setStyleSheet("font-size:18px; font-weight:bold; padding:6px; color:white; background:#555;")
        ld.addWidget(self.lbl_estado)
        self.lbl_valores = QLabel("h: -   sp: -   u: -")
        self.lbl_valores.setStyleSheet("font-size:15px; font-family:Consolas, monospace;")
        ld.addWidget(self.lbl_valores)

        # conexion
        gc = QGroupBox("Conexion")
        fc = QGridLayout(gc)
        self.cmb_puerto = QComboBox()
        btn_ref = QPushButton("Actualizar")
        btn_ref.clicked.connect(self._refrescar_puertos)
        self.btn_conectar = QPushButton("Conectar")
        self.btn_conectar.clicked.connect(self._conectar)
        self.chk_sim = QCheckBox("Modo simulacion (sin hardware)")
        self.chk_sim.toggled.connect(self._cambio_simulacion)
        fc.addWidget(QLabel("Puerto PIC:"), 0, 0)
        fc.addWidget(self.cmb_puerto, 0, 1)
        fc.addWidget(btn_ref, 0, 2)
        self.spn_cam = QSpinBox()
        self.spn_cam.setRange(0, 9)
        self.cmb_rot = QComboBox()
        self.cmb_rot.addItems(["0", "90", "180", "270"])
        self.cmb_rot.currentTextChanged.connect(self._cambio_rotacion)
        self.btn_cam = QPushButton("Abrir camara")
        self.btn_cam.clicked.connect(self._abrir_camara)
        fc.addWidget(QLabel("Camara:"), 1, 0)
        fc.addWidget(self.spn_cam, 1, 1)
        fc.addWidget(self.btn_cam, 1, 2)
        fc.addWidget(QLabel("Rotacion:"), 2, 0)
        fc.addWidget(self.cmb_rot, 2, 1)
        btn_drv = QPushButton("Ajustes driver")
        btn_drv.setToolTip("Abre el dialogo de la camara (Windows/DirectShow)")
        btn_drv.clicked.connect(lambda: self.hilo.ajustar_camara(cv2.CAP_PROP_SETTINGS, 1))
        fc.addWidget(btn_drv, 2, 2)
        self.chk_expo_auto = QCheckBox("Exposicion auto")
        self.chk_expo_auto.toggled.connect(self._aplicar_exposicion)
        self.spn_expo = QSpinBox()
        self.spn_expo.setRange(-13, 0)
        self.spn_expo.valueChanged.connect(self._aplicar_exposicion)
        fc.addWidget(self.chk_expo_auto, 3, 0)
        fc.addWidget(QLabel("Exposicion:"), 3, 1)
        fc.addWidget(self.spn_expo, 3, 2)
        fc.addWidget(self.chk_sim, 4, 0, 1, 2)
        fc.addWidget(self.btn_conectar, 4, 2)
        ld.addWidget(gc)

        # armar / paro
        fb = QHBoxLayout()
        self.btn_armar = QPushButton("ARMAR")
        self.btn_armar.setStyleSheet("background:#2e7d32; color:white; font-weight:bold; padding:10px;")
        self.btn_armar.clicked.connect(lambda: self._enviar("R"))
        self.btn_paro = QPushButton("PARO (Espacio)")
        self.btn_paro.setStyleSheet("background:#c62828; color:white; font-weight:bold; padding:10px;")
        self.btn_paro.clicked.connect(self._paro)
        fb.addWidget(self.btn_armar)
        fb.addWidget(self.btn_paro)
        ld.addLayout(fb)

        # control
        gk = QGroupBox("Control")
        fk = QFormLayout(gk)
        self.cmb_modo = QComboBox()
        self.cmb_modo.addItems(NOMBRES_MODO)
        self.cmb_modo.currentIndexChanged.connect(self._cambio_modo)
        fk.addRow("Modo:", self.cmb_modo)

        fsp = QHBoxLayout()
        self.sld_sp = QSlider(Qt.Horizontal)
        self.sld_sp.setRange(0, 1000)
        self.spn_sp = QSpinBox()
        self.spn_sp.setRange(0, 1000)
        self.spn_sp.setSuffix(" mm")
        self.sld_sp.valueChanged.connect(self.spn_sp.setValue)
        self.spn_sp.valueChanged.connect(self.sld_sp.setValue)
        self.spn_sp.valueChanged.connect(self._cambio_sp)
        fsp.addWidget(self.sld_sp)
        fsp.addWidget(self.spn_sp)
        fk.addRow("Referencia:", fsp)

        def dspin(mn, mx, dec, paso, cmd):
            s = QDoubleSpinBox()
            s.setRange(mn, mx)
            s.setDecimals(dec)
            s.setSingleStep(paso)
            s.setKeyboardTracking(False)
            s.valueChanged.connect(lambda v: self._enviar(f"{cmd}{v:.{dec}f}"))
            return s

        def ispin(mn, mx, cmd, suf=""):
            s = QSpinBox()
            s.setRange(mn, mx)
            s.setKeyboardTracking(False)
            if suf:
                s.setSuffix(suf)
            s.valueChanged.connect(lambda v: self._enviar(f"{cmd}{v}"))
            return s

        self.spn_kp = dspin(0, 20, 3, 0.05, "P")
        self.spn_ki = dspin(0, 20, 3, 0.05, "I")
        self.spn_kd = dspin(0, 5, 3, 0.01, "D")
        self.spn_alfa = dspin(0.01, 1, 2, 0.05, "F")
        self.spn_bias = ispin(0, 1023, "B")
        self.spn_ff = dspin(0, 2, 3, 0.01, "W")
        self.spn_amp = ispin(0, 1023, "A")
        self.spn_hist = ispin(0, 300, "H", " mm")
        self.spn_umax = ispin(0, 1023, "L")
        self.spn_vref = ispin(0, 5000, "V", " mm/s")
        self.spn_vref.setToolTip("Rampa de la referencia. 0 = escalon puro")
        fk.addRow("Kp:", self.spn_kp)
        fk.addRow("Ki:", self.spn_ki)
        fk.addRow("Kd:", self.spn_kd)
        fk.addRow("Filtro derivada (alfa):", self.spn_alfa)
        fk.addRow("Feedforward B (PWM a 0 mm):", self.spn_bias)
        fk.addRow("Feedforward W (PWM/mm):", self.spn_ff)
        fk.addRow("On-Off amplitud:", self.spn_amp)
        fk.addRow("On-Off histeresis:", self.spn_hist)
        fk.addRow("PWM maximo:", self.spn_umax)
        fk.addRow("Rampa referencia (0=escalon):", self.spn_vref)

        fu = QHBoxLayout()
        self.sld_u = QSlider(Qt.Horizontal)
        self.sld_u.setRange(0, 1023)
        self.spn_u = QSpinBox()
        self.spn_u.setRange(0, 1023)
        self.sld_u.valueChanged.connect(self.spn_u.setValue)
        self.spn_u.valueChanged.connect(self.sld_u.setValue)
        self.spn_u.valueChanged.connect(lambda v: self._enviar(f"U{v}"))
        fu.addWidget(self.sld_u)
        fu.addWidget(self.spn_u)
        fk.addRow("PWM manual:", fu)

        fz = QHBoxLayout()
        btn_z = QPushButton("Borrar integral")
        btn_z.clicked.connect(lambda: self._enviar("Z"))
        btn_env = QPushButton("Enviar todo")
        btn_env.clicked.connect(self._enviar_parametros)
        fz.addWidget(btn_z)
        fz.addWidget(btn_env)
        fk.addRow(fz)
        ld.addWidget(gk)

        # registro
        gr = QGroupBox("Registro")
        fr = QHBoxLayout(gr)
        self.btn_csv = QPushButton("Grabar CSV")
        self.btn_csv.setCheckable(True)
        self.btn_csv.toggled.connect(self._grabar_csv)
        btn_limpiar = QPushButton("Limpiar graficas")
        btn_limpiar.clicked.connect(self._limpiar_graficas)
        fr.addWidget(self.btn_csv)
        fr.addWidget(btn_limpiar)
        ld.addWidget(gr)
        ld.addStretch()
        div.addWidget(der)
        div.setSizes([420, 650, 360])

        self.lbl_fps = QLabel()
        self.statusBar().addPermanentWidget(self.lbl_fps)

    def _cargar_widgets(self):
        c = self.cfg
        k = c["control"]
        self._refrescar_puertos()
        idx = self.cmb_puerto.findData(c.get("puerto", ""))
        if idx >= 0:
            self.cmb_puerto.setCurrentIndex(idx)
        self.spn_cam.setValue(int(c["camara"]))
        self.cmb_rot.setCurrentText(str(self.medidor.cfg.rotacion))
        self.chk_expo_auto.setChecked(bool(c["exposicion_auto"]))
        self.spn_expo.setValue(int(c["exposicion"]))
        self._poner_control(k)

    def _poner_control(self, k: dict):
        """Pone valores en los widgets SIN enviarlos al PIC."""
        pares = [(self.cmb_modo, "modo"), (self.spn_sp, "sp"), (self.sld_sp, "sp"),
                 (self.spn_kp, "kp"), (self.spn_ki, "ki"), (self.spn_kd, "kd"),
                 (self.spn_alfa, "alfa"), (self.spn_bias, "bias"), (self.spn_ff, "ff_pend"),
                 (self.spn_amp, "amp"),
                 (self.spn_hist, "hist"), (self.spn_umax, "u_max"), (self.spn_vref, "v_ref"),
                 (self.spn_u, "u_manual"), (self.sld_u, "u_manual")]
        for w, clave in pares:
            w.blockSignals(True)
            if isinstance(w, QComboBox):
                w.setCurrentIndex(int(k[clave]))
            else:
                w.setValue(k[clave])
            w.blockSignals(False)
        self.hilo.setpoint = k["sp"]

    def _leer_control(self) -> dict:
        return {"modo": self.cmb_modo.currentIndex(), "sp": self.spn_sp.value(),
                "kp": self.spn_kp.value(), "ki": self.spn_ki.value(), "kd": self.spn_kd.value(),
                "alfa": self.spn_alfa.value(), "bias": self.spn_bias.value(),
                "ff_pend": self.spn_ff.value(),
                "amp": self.spn_amp.value(), "hist": self.spn_hist.value(),
                "u_manual": self.spn_u.value(), "u_max": self.spn_umax.value(),
                "v_ref": self.spn_vref.value()}

    # ------------------------------------------------------------ camara
    def _abrir_camara(self):
        if self.chk_sim.isChecked():
            return
        i = self.spn_cam.value()
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW) if os.name == "nt" else cv2.VideoCapture(i)
        if not cap.isOpened():
            QMessageBox.warning(self, "Camara", f"No se pudo abrir la camara {i}")
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg["ancho"])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg["alto"])
        cap.set(cv2.CAP_PROP_FPS, self.cfg["fps"])
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.hilo.poner_fuente(cap)
        self._aplicar_exposicion()
        self.statusBar().showMessage(f"Camara {i} abierta", 3000)

    def _aplicar_exposicion(self, *_):
        if self.chk_sim.isChecked():
            return
        if self.chk_expo_auto.isChecked():
            # DirectShow: 0.75 = automatica, 0.25 = manual (V4L2 usa 3 / 1)
            self.hilo.ajustar_camara(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if os.name == "nt" else 3)
        else:
            self.hilo.ajustar_camara(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25 if os.name == "nt" else 1)
            self.hilo.ajustar_camara(cv2.CAP_PROP_EXPOSURE, self.spn_expo.value())

    def _cambio_rotacion(self, txt):
        self.medidor.cfg.rotacion = int(txt)
        self.medidor.reiniciar_filtro()

    def _clic_video(self, x, y):
        if not self.chk_calibrar.isChecked() or self.hilo.ultimo_frame is None:
            return
        hm, sm, vm = self.medidor.calibrar_color(self.hilo.ultimo_frame, x, y)
        c = self.medidor.cfg
        self.statusBar().showMessage(
            f"Color calibrado: H={hm} S={sm} V={vm} -> H[{c.h_min},{c.h_max}] S>={c.s_min} V>={c.v_min}; "
            f"eje del chorro a {c.offset_chorro_mm:.0f} mm del poste", 6000)
        self.chk_calibrar.setChecked(False)

    # ------------------------------------------------------------ enlace
    def _refrescar_puertos(self):
        actual = self.cmb_puerto.currentData()
        self.cmb_puerto.clear()
        for dev, desc in listar_puertos():
            self.cmb_puerto.addItem(desc, dev)
        i = self.cmb_puerto.findData(actual)
        if i >= 0:
            self.cmb_puerto.setCurrentIndex(i)

    def _cambio_simulacion(self, activo):
        self._desconectar()
        if activo:
            # la simulacion usa su propia configuracion de vision (la real se guarda)
            self._cfg_vision_real = self.medidor.cfg
            self.medidor.cfg = ConfigVision()
            self.medidor.reiniciar_filtro()
            self.sim = SimulacionCompleta()
            self.hilo.poner_fuente(self.sim.camara)
        else:
            if getattr(self, "_cfg_vision_real", None) is not None:
                self.medidor.cfg = self._cfg_vision_real
                self._cfg_vision_real = None
                self.medidor.reiniciar_filtro()
            self.sim = None
            self.hilo.poner_fuente(None)
        self.cmb_rot.blockSignals(True)
        self.cmb_rot.setCurrentText(str(self.medidor.cfg.rotacion))
        self.cmb_rot.blockSignals(False)
        self.btn_cam.setEnabled(not activo)
        self.cmb_puerto.setEnabled(not activo)

    def _conectar(self):
        if self.enlace is not None:
            self._desconectar()
            return
        try:
            if self.chk_sim.isChecked():
                enlace = EnlaceSimulado(self.sim)
            else:
                puerto = self.cmb_puerto.currentData()
                if not puerto:
                    QMessageBox.warning(self, "Conexion", "Selecciona un puerto serie")
                    return
                enlace = EnlaceSerial(puerto)
            enlace.abrir()
        except Exception as e:
            QMessageBox.critical(self, "Conexion", f"No se pudo conectar:\n{e}")
            return
        self.enlace = enlace
        self.hilo.enlace = enlace
        self.btn_conectar.setText("Desconectar")
        self._enviar_parametros()
        self._enviar("?")

    def _desconectar(self):
        if self.enlace is not None:
            self.hilo.enlace = None
            self.enlace.cerrar()
            self.enlace = None
        self.btn_conectar.setText("Conectar")
        self.lbl_estado.setText("DESCONECTADO")
        self.lbl_estado.setStyleSheet("font-size:18px; font-weight:bold; padding:6px; color:white; background:#555;")

    def _enviar(self, cmd: str):
        if self.enlace is not None and self.enlace.conectado:
            self.enlace.enviar(cmd)

    def _enviar_parametros(self):
        k = self._leer_control()
        for cmd in (f"P{k['kp']:.3f}", f"I{k['ki']:.3f}", f"D{k['kd']:.3f}", f"F{k['alfa']:.2f}",
                    f"B{k['bias']}", f"W{k['ff_pend']:.3f}", f"A{k['amp']}", f"H{k['hist']}", f"L{k['u_max']}", f"V{k['v_ref']}",
                    f"U{k['u_manual']}", f"S{k['sp']}", f"O{k['modo']}"):
            self._enviar(cmd)

    def _paro(self):
        self._enviar("X")

    def _cambio_sp(self, v):
        self.hilo.setpoint = v
        self._enviar(f"S{v}")

    def _cambio_modo(self, m):
        # transferencia sin salto hacia manual: el PWM manual toma la u actual
        if m == 0 and self.ultima_tel is not None:
            self.spn_u.blockSignals(True)
            self.sld_u.blockSignals(True)
            self.spn_u.setValue(self.ultima_tel.u)
            self.sld_u.setValue(self.ultima_tel.u)
            self.spn_u.blockSignals(False)
            self.sld_u.blockSignals(False)
            self._enviar(f"U{self.ultima_tel.u}")
        self._enviar(f"O{m}")

    def _leer_serial(self):
        if self.enlace is None:
            return
        ahora = time.monotonic() - self.t0
        for linea in self.enlace.leer_lineas():
            if linea.startswith("T,"):
                t = Telemetria.de_linea(linea, ahora)
                if t is None:
                    continue
                self.ultima_tel = t
                self.b_t.append(t.t)
                self.b_h.append(t.h)
                self.b_sp.append(t.sp)
                self.b_u.append(t.u)
                if self.csv_escritor:
                    r = self.ultimo_res
                    self.csv_escritor.writerow([
                        f"{t.t:.3f}", t.h, t.sp, t.u, t.modo, t.estado, t.dt,
                        f"{r.lateral_mm:.1f}" if (r and r.lateral_mm is not None) else ""])
            elif linea.startswith("C,"):
                k = config_de_linea(linea)
                if k:
                    self._poner_control(k)
            elif linea.startswith("E,"):
                self.statusBar().showMessage("PIC: " + linea[2:], 4000)

    # ------------------------------------------------------------ actualizacion
    def _nuevo_cuadro(self, img, r):
        self.ultimo_res = r
        self.video.mostrar(img)
        self.lbl_fps.setText(f"{self.hilo.fps:4.1f} fps | vision {r.t_proceso_ms:4.1f} ms | "
                             f"{'marcadores OK' if r.marcadores_vistos else 'SIN marcadores'}")

    def _actualizar_graficas(self):
        if self.b_t:
            t = np.fromiter(self.b_t, float)
            self.c_h.setData(t, np.fromiter(self.b_h, float))
            self.c_sp.setData(t, np.fromiter(self.b_sp, float))
            self.c_u.setData(t, np.fromiter(self.b_u, float))
            self.g_h.setXRange(max(0.0, t[-1] - VENTANA_S), max(VENTANA_S, t[-1]), padding=0)
        tel = self.ultima_tel
        if tel is not None and self.enlace is not None:
            est = NOMBRES_ESTADO[tel.estado] if 0 <= tel.estado < len(NOMBRES_ESTADO) else "?"
            modo = NOMBRES_MODO[tel.modo] if 0 <= tel.modo < len(NOMBRES_MODO) else "?"
            color = COLORES_ESTADO[tel.estado] if 0 <= tel.estado < len(COLORES_ESTADO) else "#555"
            self.lbl_estado.setText(f"{est}  |  {modo}")
            self.lbl_estado.setStyleSheet(
                f"font-size:18px; font-weight:bold; padding:6px; color:white; background:{color};")
            self.lbl_valores.setText(f"h: {tel.h:5d} mm   sp: {tel.sp:5d} mm   u: {tel.u:4d}   dt: {tel.dt} ms")

    def _limpiar_graficas(self):
        for b in (self.b_t, self.b_h, self.b_sp, self.b_u):
            b.clear()

    def _grabar_csv(self, activo):
        if activo:
            os.makedirs(DIR_REGISTROS, exist_ok=True)
            nombre = os.path.join(DIR_REGISTROS, datetime.now().strftime("levitador_%Y%m%d_%H%M%S.csv"))
            self.csv_archivo = open(nombre, "w", newline="", encoding="utf-8")
            self.csv_escritor = csv.writer(self.csv_archivo)
            self.csv_escritor.writerow(["t_s", "altura_mm", "referencia_mm", "u_pwm", "modo",
                                        "estado", "dt_ms", "lateral_mm"])
            self.btn_csv.setText("Detener CSV")
            self.statusBar().showMessage(f"Grabando en {nombre}", 5000)
        else:
            self.csv_escritor = None
            if self.csv_archivo:
                self.csv_archivo.close()
                self.csv_archivo = None
            self.btn_csv.setText("Grabar CSV")

    # ------------------------------------------------------------ cierre
    def closeEvent(self, ev):
        self.cfg["camara"] = self.spn_cam.value()
        self.cfg["puerto"] = self.cmb_puerto.currentData() or ""
        self.cfg["simulacion"] = self.chk_sim.isChecked()
        self.cfg["exposicion_auto"] = self.chk_expo_auto.isChecked()
        self.cfg["exposicion"] = self.spn_expo.value()
        self.cfg["control"] = self._leer_control()
        cfg_v = getattr(self, "_cfg_vision_real", None) or self.medidor.cfg
        self.cfg["vision"] = cfg_v.a_dict()
        try:
            with open(ARCHIVO_CONFIG, "w", encoding="utf-8") as f:
                json.dump(self.cfg, f, indent=2, ensure_ascii=False)
        except OSError:
            pass
        self._grabar_csv(False)
        self._desconectar()
        self.hilo.detener()
        super().closeEvent(ev)


def main():
    app = QApplication(sys.argv)
    v = Ventana()
    v.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

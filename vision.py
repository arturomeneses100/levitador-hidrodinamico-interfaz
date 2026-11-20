"""
vision.py - Medicion de la altura de la pelota con una webcam.

Referencia: dos marcadores ArUco (diccionario 4x4_50) montados en un poste
en el MISMO PLANO que el chorro:
    ID 0 -> centro a la altura de la salida de la boquilla (0 mm)
    ID 1 -> centro a 1000 mm por encima del ID 0 (configurable)

Con los dos centros se obtiene el eje vertical y la escala mm/px en cada
cuadro, de modo que si la camara se mueve o se inclina un poco la medicion
sigue siendo correcta.

La pelota (unicel pintada de naranja fluorescente) se detecta por color en
HSV dentro de una franja (ROI) alrededor del eje de los marcadores.
La altura reportada es la de la PARTE INFERIOR de la pelota respecto a la
boquilla: pelota descansando en la boquilla ~ 0 mm.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass, field, asdict

import cv2
import numpy as np

DICCIONARIO_ARUCO = cv2.aruco.DICT_4X4_50
ID_BASE = 0
ID_SUPERIOR = 1


# ---------------------------------------------------------------------------
#  Configuracion
# ---------------------------------------------------------------------------
@dataclass
class ConfigVision:
    # Color de la pelota en HSV de OpenCV (H 0-179, S 0-255, V 0-255).
    # Valores iniciales para naranja fluorescente; se recalibran con un clic.
    h_min: int = 5
    h_max: int = 22
    s_min: int = 120
    v_min: int = 110
    s_max: int = 255
    v_max: int = 255

    distancia_marcadores_mm: float = 1000.0  # entre centros de ID0 e ID1
    altura_marcador_base_mm: float = 0.0     # altura del centro de ID0 respecto a la boquilla
    diametro_pelota_mm: float = 50.0         # se usa si el radio medido es dudoso
    offset_chorro_mm: float = 250.0          # distancia lateral del poste al chorro (+ derecha)
    semiancho_roi_mm: float = 130.0          # franja de busqueda a cada lado del chorro
    area_min_px: int = 60
    circularidad_min: float = 0.55
    salto_max_mm: float = 200.0              # salto maximo creible entre cuadros
    mediana_n: int = 3                       # filtro de mediana (1 = sin filtro)
    rotacion: int = 0                        # 0, 90, 180, 270 (camara girada)

    def a_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def de_dict(cls, d: dict) -> "ConfigVision":
        campos = {k: v for k, v in d.items() if k in cls.__dataclass_fields__}
        return cls(**campos)


@dataclass
class Resultado:
    marcadores_ok: bool = False          # hay referencia (vista o recordada)
    marcadores_vistos: bool = False      # se vieron en ESTE cuadro
    pelota_ok: bool = False
    altura_mm: float | None = None       # filtrada, base de la pelota
    altura_cruda_mm: float | None = None
    lateral_mm: float | None = None      # desviacion respecto al chorro (+ derecha)
    centro_px: tuple | None = None
    radio_px: float | None = None
    mm_por_px: float | None = None
    base_px: tuple | None = None
    sup_px: tuple | None = None
    esquinas: list = field(default_factory=list)
    ids: list = field(default_factory=list)
    roi_poly: np.ndarray | None = None
    mascara: np.ndarray | None = None
    t_proceso_ms: float = 0.0


def rotar(frame: np.ndarray, grados: int) -> np.ndarray:
    if grados == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if grados == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if grados == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return frame


# ---------------------------------------------------------------------------
#  Detector ArUco compatible con OpenCV 4.6 y 4.7+
# ---------------------------------------------------------------------------
class _DetectorAruco:
    def __init__(self):
        self.dic = cv2.aruco.getPredefinedDictionary(DICCIONARIO_ARUCO)
        if hasattr(cv2.aruco, "ArucoDetector"):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._det = cv2.aruco.ArucoDetector(self.dic, params)
            self._nuevo = True
        else:  # OpenCV < 4.7
            self._params = cv2.aruco.DetectorParameters_create()
            self._nuevo = False

    def detectar(self, gris):
        if self._nuevo:
            esquinas, ids, _ = self._det.detectMarkers(gris)
        else:
            esquinas, ids, _ = cv2.aruco.detectMarkers(gris, self.dic, parameters=self._params)
        return esquinas, ids


def generar_marcador(id_marcador: int, lado_px: int) -> np.ndarray:
    dic = cv2.aruco.getPredefinedDictionary(DICCIONARIO_ARUCO)
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dic, id_marcador, lado_px)
    return cv2.aruco.drawMarker(dic, id_marcador, lado_px)


# ---------------------------------------------------------------------------
#  Medidor principal
# ---------------------------------------------------------------------------
class MedidorAltura:
    def __init__(self, cfg: ConfigVision | None = None):
        self.cfg = cfg or ConfigVision()
        self._aruco = _DetectorAruco()
        self._base = None          # centro ID0 (px), recordado
        self._sup = None           # centro ID1 (px), recordado
        self._t_marcadores = 0.0
        self._hist = deque(maxlen=max(1, self.cfg.mediana_n))
        self._ultima = None        # ultima altura cruda aceptada
        self._rechazos = 0
        self._ult_centro = None
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # ----------------------------------------------------------------- color
    def calibrar_color(self, frame_bgr: np.ndarray, x: int, y: int, r: int = 6) -> tuple:
        """Toma una muestra de color alrededor de (x, y) del cuadro YA rotado
        y ajusta los rangos HSV. Regresa (h, s, v) medianos."""
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        h, w = hsv.shape[:2]
        x0, x1 = max(0, x - r), min(w, x + r + 1)
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        parche = hsv[y0:y1, x0:x1].reshape(-1, 3)
        hm, sm, vm = (int(v) for v in np.median(parche, axis=0))
        c = self.cfg
        c.h_min = max(0, hm - 9)
        c.h_max = min(179, hm + 9)
        c.s_min = int(max(60, sm - 70))
        c.v_min = int(max(50, vm - 90))
        c.s_max = 255
        c.v_max = 255
        # el clic tambien fija la posicion lateral del chorro
        if self._base is not None:
            eje = self._sup - self._base
            largo = np.linalg.norm(eje)
            u = eje / largo
            n = np.array([-u[1], u[0]])
            if n[0] < 0:
                n = -n
            mmpx = self.cfg.distancia_marcadores_mm / largo
            c.offset_chorro_mm = float((np.array([x, y]) - self._base) @ n * mmpx)
        self.reiniciar_filtro()
        return hm, sm, vm

    def reiniciar_filtro(self):
        self._hist = deque(maxlen=max(1, self.cfg.mediana_n))
        self._ultima = None
        self._rechazos = 0

    # ------------------------------------------------------------- geometria
    def _px_de_altura(self, altura_mm: float, lateral_mm: float = 0.0):
        """Convierte (altura, lateral) en mm a coordenadas de pixel."""
        b, s = self._base, self._sup
        eje = s - b
        largo = np.linalg.norm(eje)
        u = eje / largo
        n = np.array([-u[1], u[0]])            # perpendicular
        if n[0] < 0:
            n = -n                             # que apunte a la derecha
        mmpx = self.cfg.distancia_marcadores_mm / largo
        t = (altura_mm - self.cfg.altura_marcador_base_mm) / mmpx
        p = b + u * t + n * (lateral_mm / mmpx)
        return float(p[0]), float(p[1])

    def linea_altura(self, altura_mm: float, semiancho_mm: float = 80.0, en_poste: bool = False):
        """Extremos (px) de un segmento horizontal a cierta altura; para dibujar."""
        if self._base is None:
            return None
        c0 = 0.0 if en_poste else self.cfg.offset_chorro_mm
        p0 = self._px_de_altura(altura_mm, c0 - semiancho_mm)
        p1 = self._px_de_altura(altura_mm, c0 + semiancho_mm)
        return (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1]))

    def _roi(self, forma):
        sa = self.cfg.semiancho_roi_mm
        oc = self.cfg.offset_chorro_mm
        top = self.cfg.distancia_marcadores_mm * 1.15
        bot = -12.0                     # justo debajo de la boquilla (la excluye)
        pts = [self._px_de_altura(bot, oc - sa), self._px_de_altura(top, oc - sa),
               self._px_de_altura(top, oc + sa), self._px_de_altura(bot, oc + sa)]
        poly = np.array(pts, dtype=np.int32)
        mask = np.zeros(forma[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, poly, 255)
        return poly, mask

    # ------------------------------------------------------------- proceso
    def procesar(self, frame_bgr: np.ndarray, quiere_mascara: bool = False) -> Resultado:
        """frame_bgr ya debe venir rotado (ver rotar())."""
        t0 = time.perf_counter()
        c = self.cfg
        r = Resultado()

        # 1) Marcadores
        gris = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        esquinas, ids = self._aruco.detectar(gris)
        vistos = {}
        if ids is not None:
            for esq, i in zip(esquinas, ids.flatten()):
                vistos[int(i)] = esq.reshape(4, 2).mean(axis=0)
                r.esquinas.append(esq)
                r.ids.append(int(i))
        if ID_BASE in vistos and ID_SUPERIOR in vistos:
            # suavizado ligero para que la referencia no "tiemble"
            if self._base is None:
                self._base, self._sup = vistos[ID_BASE], vistos[ID_SUPERIOR]
            else:
                self._base = 0.7 * self._base + 0.3 * vistos[ID_BASE]
                self._sup = 0.7 * self._sup + 0.3 * vistos[ID_SUPERIOR]
            self._t_marcadores = time.monotonic()
            r.marcadores_vistos = True
        r.marcadores_ok = self._base is not None and np.linalg.norm(self._sup - self._base) > 20
        if not r.marcadores_ok:
            r.t_proceso_ms = (time.perf_counter() - t0) * 1000
            return r

        eje = self._sup - self._base
        largo = float(np.linalg.norm(eje))
        u = eje / largo
        mmpx = c.distancia_marcadores_mm / largo
        r.mm_por_px = mmpx
        r.base_px = (float(self._base[0]), float(self._base[1]))
        r.sup_px = (float(self._sup[0]), float(self._sup[1]))

        # 2) Mascara de color dentro de la ROI
        poly, mroi = self._roi(frame_bgr.shape)
        r.roi_poly = poly
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame_bgr, (5, 5), 0), cv2.COLOR_BGR2HSV)
        if c.h_min <= c.h_max:
            m = cv2.inRange(hsv, (c.h_min, c.s_min, c.v_min), (c.h_max, c.s_max, c.v_max))
        else:  # rango que cruza el rojo (H=179 -> 0)
            m = cv2.inRange(hsv, (c.h_min, c.s_min, c.v_min), (179, c.s_max, c.v_max)) | \
                cv2.inRange(hsv, (0, c.s_min, c.v_min), (c.h_max, c.s_max, c.v_max))
        m = cv2.bitwise_and(m, mroi)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self._kernel)
        if quiere_mascara:
            r.mascara = m

        # 3) Mejor candidato: grande, redondo y cerca de la ultima posicion
        contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        radio_esperado_px = c.diametro_pelota_mm / 2 / mmpx
        mejor, mejor_pts = None, -1e9
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area < c.area_min_px:
                continue
            per = cv2.arcLength(cnt, True)
            circ = 4 * np.pi * area / (per * per) if per > 0 else 0
            if circ < c.circularidad_min:
                continue
            (x, y), rad = cv2.minEnclosingCircle(cnt)
            if rad < 0.4 * radio_esperado_px or rad > 2.5 * radio_esperado_px:
                continue
            pts = area * circ
            if self._ult_centro is not None:
                dist = np.hypot(x - self._ult_centro[0], y - self._ult_centro[1])
                pts -= dist * 2.0
            if pts > mejor_pts:
                mejor_pts, mejor = pts, (x, y, rad)

        if mejor is None:
            r.t_proceso_ms = (time.perf_counter() - t0) * 1000
            return r

        x, y, rad = mejor
        d = np.array([x, y]) - self._base
        t_eje = float(d @ u)                       # px a lo largo del eje
        n = np.array([-u[1], u[0]])                # perpendicular al eje
        if n[0] < 0:
            n = -n                                 # que apunte a la derecha
        lat = float(d @ n)                         # px perpendicular
        radio_mm = rad * mmpx
        # si el radio medido es raro (gotas encima, parcialmente tapada) usar el nominal
        if not (0.6 * c.diametro_pelota_mm / 2 < radio_mm < 1.5 * c.diametro_pelota_mm / 2):
            radio_mm = c.diametro_pelota_mm / 2
        h_centro = c.altura_marcador_base_mm + t_eje * mmpx
        h_cruda = h_centro - radio_mm

        # 4) Rechazo de saltos imposibles (gota naranja, reflejo, etc.)
        if self._ultima is not None and abs(h_cruda - self._ultima) > c.salto_max_mm:
            self._rechazos += 1
            if self._rechazos < 4:
                r.t_proceso_ms = (time.perf_counter() - t0) * 1000
                return r                           # se ignora este cuadro
            self._hist.clear()                     # salto persistente: es real
        self._rechazos = 0
        self._ultima = h_cruda
        self._ult_centro = (x, y)

        # 5) Mediana corta
        self._hist.append(h_cruda)
        h_filtrada = float(np.median(self._hist))

        r.pelota_ok = True
        r.altura_cruda_mm = h_cruda
        r.altura_mm = h_filtrada
        r.lateral_mm = lat * mmpx - c.offset_chorro_mm
        r.centro_px = (float(x), float(y))
        r.radio_px = float(rad)
        r.t_proceso_ms = (time.perf_counter() - t0) * 1000
        return r

    # ------------------------------------------------------------- dibujo
    def dibujar(self, frame: np.ndarray, r: Resultado, setpoint_mm: float | None = None) -> np.ndarray:
        """Dibuja la superposicion sobre el cuadro (in place) y lo regresa."""
        if r.esquinas:
            ids = np.array(r.ids, dtype=np.int32).reshape(-1, 1)
            cv2.aruco.drawDetectedMarkers(frame, r.esquinas, ids)
        if not r.marcadores_ok:
            cv2.putText(frame, "Sin marcadores ArUco 0 y 1", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return frame
        b = tuple(int(v) for v in r.base_px)
        s = tuple(int(v) for v in r.sup_px)
        cv2.line(frame, b, s, (255, 255, 0), 1)
        eb = self.linea_altura(0, 0)
        es = self.linea_altura(self.cfg.distancia_marcadores_mm, 0)
        if eb and es:
            cv2.line(frame, eb[0], es[0], (120, 120, 60), 1)   # eje del chorro
        if r.roi_poly is not None:
            cv2.polylines(frame, [r.roi_poly], True, (90, 90, 90), 1)
        # marcas cada 100 mm
        for mm in range(0, int(self.cfg.distancia_marcadores_mm) + 1, 100):
            seg = self.linea_altura(mm, 15, en_poste=True)
            if seg:
                cv2.line(frame, seg[0], seg[1], (200, 200, 200), 1)
                cv2.putText(frame, f"{mm // 10}", (seg[1][0] + 4, seg[1][1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        if setpoint_mm is not None:
            seg = self.linea_altura(setpoint_mm, 90)
            if seg:
                cv2.line(frame, seg[0], seg[1], (0, 255, 0), 2)
        if r.pelota_ok:
            cx, cy = int(r.centro_px[0]), int(r.centro_px[1])
            cv2.circle(frame, (cx, cy), int(r.radio_px), (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 2, (0, 255, 255), -1)
            cv2.putText(frame, f"{r.altura_mm / 10:.1f} cm", (cx + int(r.radio_px) + 6, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        else:
            cv2.putText(frame, "Pelota no detectada", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 140, 255), 2)
        if not r.marcadores_vistos:
            cv2.putText(frame, "(referencia recordada)", (10, frame.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        return frame

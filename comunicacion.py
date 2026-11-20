"""
comunicacion.py - Enlace con el PIC (puerto serie real o PIC simulado).

Ambas clases tienen la misma interfaz:
    abrir(), cerrar(), enviar(texto), leer_lineas() -> list[str], conectado
"""
from __future__ import annotations

import queue
import threading
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # permite usar el modo simulacion sin pyserial
    serial = None

NOMBRES_MODO = ["Manual (lazo abierto)", "On-Off", "P", "PI", "PID"]
NOMBRES_ESTADO = ["PARO", "SIN ENLACE", "CONTROLANDO", "PELOTA PERDIDA", "PELOTA CAIDA"]


def listar_puertos() -> list[tuple[str, str]]:
    """[(dispositivo, descripcion)]"""
    if serial is None:
        return []
    return [(p.device, f"{p.device} - {p.description}") for p in serial.tools.list_ports.comports()]


class Telemetria:
    __slots__ = ("t", "h", "sp", "u", "modo", "estado", "dt")

    def __init__(self, t, h, sp, u, modo, estado, dt):
        self.t, self.h, self.sp, self.u = t, h, sp, u
        self.modo, self.estado, self.dt = modo, estado, dt

    @classmethod
    def de_linea(cls, linea: str, t: float) -> "Telemetria | None":
        try:
            _, h, sp, u, modo, est, dt = linea.split(",")
            return cls(t, int(h), int(sp), int(u), int(modo), int(est), int(dt))
        except ValueError:
            return None


def config_de_linea(linea: str) -> dict | None:
    """'C,modo,sp,kp,ki,kd,alfa,bias,amp,hist,u_manual,u_max,ff_pend,v_ref' -> dict"""
    try:
        p = [v.strip() for v in linea.split(",")[1:]]
        return {
            "modo": int(p[0]), "sp": int(p[1]),
            "kp": float(p[2]), "ki": float(p[3]), "kd": float(p[4]), "alfa": float(p[5]),
            "bias": int(p[6]), "amp": int(p[7]), "hist": int(p[8]),
            "u_manual": int(p[9]), "u_max": int(p[10]),
            "ff_pend": float(p[11]) if len(p) > 11 else 0.0,
            "v_ref": int(p[12]) if len(p) > 12 else 0,
        }
    except (ValueError, IndexError):
        return None


class EnlaceSerial:
    def __init__(self, puerto: str, baudios: int = 115200):
        self.puerto = puerto
        self.baudios = baudios
        self._ser = None
        self._hilo = None
        self._cola: queue.Queue[str] = queue.Queue()
        self._vivo = False
        self._lock = threading.Lock()

    @property
    def conectado(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def abrir(self):
        if serial is None:
            raise RuntimeError("Falta pyserial: pip install pyserial")
        self._ser = serial.Serial(self.puerto, self.baudios, timeout=0.05, write_timeout=0.2)
        time.sleep(0.1)
        self._ser.reset_input_buffer()
        self._vivo = True
        self._hilo = threading.Thread(target=self._leer, daemon=True)
        self._hilo.start()

    def _leer(self):
        buf = b""
        while self._vivo:
            try:
                datos = self._ser.read(256)
            except Exception as e:  # puerto desconectado
                self._cola.put(f"E,ERROR_SERIAL {e}")
                self._vivo = False
                break
            if not datos:
                continue
            buf += datos
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                txt = linea.decode("ascii", errors="ignore").strip()
                if txt:
                    self._cola.put(txt)

    def enviar(self, texto: str):
        if not self.conectado:
            return
        with self._lock:
            try:
                self._ser.write((texto + "\n").encode("ascii"))
            except Exception as e:
                self._cola.put(f"E,ERROR_SERIAL {e}")

    def leer_lineas(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self._cola.get_nowait())
            except queue.Empty:
                return out

    def cerrar(self):
        self._vivo = False
        if self._ser is not None:
            try:
                self._ser.write(b"X\n")      # deja la bomba apagada
                time.sleep(0.05)
                self._ser.close()
            except Exception:
                pass
        self._ser = None


class EnlaceSimulado:
    """Envuelve un PICSimulado + Planta; un hilo mantiene el reloj."""

    def __init__(self, sim):
        self.sim = sim
        self._lock = threading.Lock()
        self._vivo = False
        self._hilo = None

    @property
    def conectado(self) -> bool:
        return self._vivo

    def abrir(self):
        self._vivo = True
        self._hilo = threading.Thread(target=self._latido, daemon=True)
        self._hilo.start()

    def _latido(self):
        while self._vivo:
            with self._lock:
                self.sim.pic.tick()
                self.sim.sincronizar()
            time.sleep(0.01)

    def enviar(self, texto: str):
        with self._lock:
            self.sim.pic.escribir(texto + "\n")
            self.sim.sincronizar()

    def leer_lineas(self) -> list[str]:
        with self._lock:
            return self.sim.pic.leer_lineas()

    def cerrar(self):
        self.enviar("X")
        self._vivo = False

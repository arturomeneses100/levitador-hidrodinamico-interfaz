"""
generar_marcadores.py - Crea las hojas para imprimir los marcadores ArUco.

    python generar_marcadores.py            # marcadores de 80 mm (recomendado)
    python generar_marcadores.py 100        # marcadores de 100 mm

Genera marcadores/aruco_id0.pdf y marcadores/aruco_id1.pdf (tamano carta) y
sus PNG de vista previa. Imprimir el PDF al 100 % ("tamano real", sin
"ajustar a pagina").

El tamano exacto del marcador NO afecta la medicion: lo unico que importa es
la distancia entre los CENTROS de los dos marcadores (1000 mm por defecto).
Las marcas en cruz alrededor de cada marcador senalan su centro para poder
medirlo con flexometro.
"""
import os
import sys
import zlib

import cv2
import numpy as np

from vision import generar_marcador

DPI = 300
ANCHO_HOJA, ALTO_HOJA = int(8.5 * DPI), int(11 * DPI)   # carta


def mm_a_px(mm: float) -> int:
    return int(round(mm / 25.4 * DPI))


def hoja(id_marcador: int, lado_mm: float, leyenda: str) -> np.ndarray:
    img = np.full((ALTO_HOJA, ANCHO_HOJA), 255, np.uint8)
    lado = mm_a_px(lado_mm)
    m = generar_marcador(id_marcador, lado)
    cx, cy = ANCHO_HOJA // 2, ALTO_HOJA // 2
    x0, y0 = cx - lado // 2, cy - lado // 2
    img[y0:y0 + lado, x0:x0 + lado] = m
    # marcas de centro (fuera del margen blanco obligatorio)
    sep = lado // 2 + mm_a_px(20)
    largo = mm_a_px(12)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        p0 = (cx + dx * sep, cy + dy * sep)
        p1 = (cx + dx * (sep + largo), cy + dy * (sep + largo))
        cv2.line(img, p0, p1, 0, 3)
    # recuadro de corte con margen blanco de 15 mm
    margen = mm_a_px(15)
    cv2.rectangle(img, (x0 - margen, y0 - margen), (x0 + lado + margen, y0 + lado + margen), 160, 2)
    f = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, f"ArUco DICT_4X4_50  -  ID {id_marcador}  -  lado {lado_mm:.0f} mm",
                (mm_a_px(15), mm_a_px(20)), f, 1.6, 0, 3)
    cv2.putText(img, leyenda, (mm_a_px(15), mm_a_px(32)), f, 1.3, 0, 2)
    cv2.putText(img, "Imprimir al 100% (tamano real). Recortar por la linea gris dejando el margen blanco.",
                (mm_a_px(15), ALTO_HOJA - mm_a_px(25)), f, 1.0, 0, 2)
    cv2.putText(img, "Enmicar en MATE (no brillante) para protegerlo del agua.",
                (mm_a_px(15), ALTO_HOJA - mm_a_px(15)), f, 1.0, 0, 2)
    return img


def guardar_pdf(img: np.ndarray, ruta: str):
    """PDF de una pagina tamano carta con la imagen en escala de grises
    ocupando toda la hoja (sin dependencias externas)."""
    h, w = img.shape
    datos = zlib.compress(img.tobytes(), 9)
    contenido = b"q 612 0 0 792 0 0 cm /Im0 Do Q"
    objetos = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 612 792] "
        b"/Resources << /XObject << /Im0 4 0 R >> >> /Contents 5 0 R >>",
        (f"<< /Type /XObject /Subtype /Image /Width {w} /Height {h} /ColorSpace /DeviceGray "
         f"/BitsPerComponent 8 /Filter /FlateDecode /Length {len(datos)} >>\nstream\n").encode()
        + datos + b"\nendstream",
        f"<< /Length {len(contenido)} >>\nstream\n".encode() + contenido + b"\nendstream",
    ]
    salida = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, obj in enumerate(objetos, 1):
        offsets.append(len(salida))
        salida += f"{i} 0 obj\n".encode() + obj + b"\nendobj\n"
    xref = len(salida)
    salida += f"xref\n0 {len(objetos) + 1}\n0000000000 65535 f \n".encode()
    for o in offsets:
        salida += f"{o:010d} 00000 n \n".encode()
    salida += f"trailer\n<< /Size {len(objetos) + 1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    with open(ruta, "wb") as f:
        f.write(salida)


def main():
    lado_mm = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0
    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "marcadores")
    os.makedirs(carpeta, exist_ok=True)
    datos = [
        (0, "BASE: su centro va a la altura de la salida de la boquilla (0 mm)"),
        (1, "SUPERIOR: su centro va 1000 mm arriba del centro del ID 0"),
    ]
    for id_m, leyenda in datos:
        img = hoja(id_m, lado_mm, leyenda)
        base = os.path.join(carpeta, f"aruco_id{id_m}")
        guardar_pdf(img, base + ".pdf")
        cv2.imwrite(base + ".png", cv2.resize(img, None, fx=0.25, fy=0.25, interpolation=cv2.INTER_AREA))
        print("Generado:", base + ".pdf  (imprimir este)")


if __name__ == "__main__":
    main()

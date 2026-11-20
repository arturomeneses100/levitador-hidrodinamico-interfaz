# Levitador hidrodinámico — Interfaz y visión por computadora

Control de la **altura de una pelota de unicel sostenida por un chorro de agua vertical** (0 a 100 cm). Una **webcam genérica** mide la altura con **OpenCV**, un **PIC18F4550** ejecuta el controlador (Manual, On-Off, P, PI y PID) y mueve una **bomba de 12 V** con PWM. Esta aplicación en **Python** es la interfaz de supervisión: video en vivo, gráficas, ajuste de parámetros y registro de datos.

> Este repositorio contiene la **parte de PC (Python)**.
> El firmware del microcontrolador está en el repositorio **levitador-hidrodinamico-pic**.

---

## Contenido

1. [Contexto de la práctica](#1-contexto-de-la-práctica)
2. [Cómo funciona](#2-cómo-funciona)
3. [Materiales](#3-materiales)
4. [Construcción del montaje](#4-construcción-del-montaje)
5. [Instalación del software](#5-instalación-del-software)
6. [Uso paso a paso](#6-uso-paso-a-paso)
7. [Prácticas sugeridas](#7-prácticas-sugeridas)
8. [Parámetros del controlador](#8-parámetros-del-controlador)
9. [Seguridad](#9-seguridad)
10. [Solución de problemas](#10-solución-de-problemas)
11. [Estructura del repositorio](#11-estructura-del-repositorio)
12. [Protocolo serie](#12-protocolo-serie)
13. [Licencia](#13-licencia)

---

## 1. Contexto de la práctica

**Asignatura / unidad:** Desarrollo de aplicaciones de control de procesos utilizando sistemas embebidos.

**Objetivo general:** que el estudiante diseñe, implemente y compare controladores en lazo abierto y lazo cerrado (On-Off, P, PI y PID) sobre un proceso físico real, no lineal y con retardo, usando un microcontrolador PIC y una interfaz de supervisión en Python.

**Qué se aprende:**

- La diferencia entre **lazo abierto y lazo cerrado** frente a perturbaciones.
- **Identificación** experimental de una planta: curva estática (PWM contra altura), zona muerta y respuesta al escalón.
- El comportamiento característico de cada controlador:
  - **On-Off:** oscilación sostenida, efecto de la histéresis.
  - **P:** error en estado estacionario.
  - **PI:** elimina ese error; aparece la saturación y el *windup*.
  - **PID:** amortiguamiento y efecto del ruido sobre la derivada.
- **Feedforward** (prealimentación) a partir de la curva estática.
- **Medición con visión por computadora**: calibración con marcadores ArUco, segmentación por color y filtrado.
- **Diseño seguro de sistemas embebidos**: *watchdog* de comunicación, estados de falla y paro de emergencia.
- Evaluación con **métricas**: sobrepaso, tiempo de asentamiento, error en estado estacionario e IAE.

**Por qué es un buen proceso para la clase:** es visual (la pelota sube y baja frente al grupo), tiene una dinámica lenta (se ve a simple vista y basta con muestrear a 30 Hz), es no lineal (la bomba tiene zona muerta y la fuerza del chorro cambia con la altura) y tiene retardo (bomba, agua y cámara). Por eso las diferencias entre controladores se notan con claridad.

---

## 2. Cómo funciona

```
 ┌──────────┐  640x480  ┌───────────────────────────────┐   "M523\n"   ┌──────────────────────┐
 │  Webcam  │──────────▶│  PC – Python (este repo)      │─────────────▶│  PIC18F4550          │
 └──────────┘   30 fps  │  • ArUco ID0/ID1 → escala mm  │  115200 bps  │  • PID / PI / P /    │
      ▲                 │  • pelota naranja (HSV)       │◀─────────────│    On-Off / Manual   │
      │                 │  • altura = base de la pelota │ "T,h,sp,u,…" │  • PWM 1.22 kHz      │
      │                 │  • gráficas, CSV, parámetros  │              │  • watchdog y paro   │
      │                 └───────────────────────────────┘              └──────────┬───────────┘
      │                                                                           │ MOSFET
      │        ●  pelota de unicel Ø50 mm, ~10 g                                  ▼
      │        ┆  chorro vertical (boquilla Ø5 mm)                     ┌──────────────────────┐
      └────────┆──────────────────────────────────────────────────────│ Bomba QWORK 12 V     │
               ╧  boquilla                                            │ 800 L/h, 5 m (DC40)  │
                                                                       └──────────────────────┘
```

**¿Por qué el PID está en el PIC y no en Python?**

- El tema de la unidad es el **control embebido**: el algoritmo vive en el microcontrolador.
- El PIC es quien maneja la bomba. Si la PC se congela, el PIC lo detecta en 300 ms y **apaga la bomba**.
- Python hace lo que el PIC no puede: procesar video, graficar y registrar.
- La cámara (unos 33 ms por cuadro más el procesamiento) marca el ritmo. El PIC ejecuta el control **cada vez que llega una medición** y mide el `dt` real con su propio temporizador.

**Cómo se mide la altura:**

1. Dos marcadores **ArUco** (ID 0 e ID 1) en un poste vertical, en el mismo plano que el chorro. El centro del ID 0 queda a la altura de la salida de la boquilla y el centro del ID 1, 1000 mm arriba.
2. En cada cuadro se detectan los marcadores. Con ellos se obtienen el **eje vertical** y la **escala mm/px**, así que, si alguien golpea la cámara, la medición se corrige sola.
3. La pelota naranja se segmenta en HSV **solo dentro de una franja alrededor del chorro**.
4. Se elige el contorno más grande y redondo, cercano a la posición anterior.
5. Se reporta la altura de la **parte inferior** de la pelota respecto a la boquilla: pelota apoyada ≈ 0 mm.
6. Se rechazan saltos imposibles (gotas, reflejos) y se aplica una mediana de 3 cuadros.

En la simulación el error de medición es de unos ±2 mm.

---

## 3. Materiales

### Pelota (tamaño ideal teórico)

| Parámetro | Valor | Por qué |
|---|---|---|
| **Diámetro** | **50 mm** (rango útil de 40 a 60 mm) | Unas 10 veces el diámetro del chorro: el efecto Coanda la mantiene centrada. Ocupa unos 27 px en la cámara (se necesitan más de 20). |
| **Masa total** | **≈ 10 g** (unicel + lastre) | Una pelota de unicel sola pesa unos 3 g y "flota" en la punta del chorro (menos de 1 cm por debajo), donde el chorro ya se abre y es inestable. Con 10 g se apoya unos 6 cm abajo, en chorro compacto, y la altura mínima controlable baja de unos 25 cm a unos 4 cm. |
| **Color** | **Naranja fluorescente** | Contrasta con el fondo negro y con el agua, que se ve blanca o transparente. |

Cálculo con la bomba QWORK (5 m de altura máxima, 800 L/h) y boquilla de 5 mm:

| Masa | Distancia bajo la punta del chorro a 1 m | PWM necesario a 1 m | Altura mínima controlable |
|---|---|---|---|
| 3 g (solo unicel) | 0.6 cm | 75 % | ≈ 25 cm |
| 8 g | 4.0 cm | 76 % | ≈ 12 cm |
| **10 g** | **6.1 cm** | **77 %** | **≈ 4 cm** |
| 15 g | 12.9 cm | 80 % | 0 cm |

**Preparación de la pelota:**

1. Esfera de unicel de **5 cm** (papelerías).
2. Con un palillo o una broca fina, abre un canal hasta el **centro** e introduce el lastre: balines, perdigones o una plomada de pesca. El total debe quedar en **≈ 10 g**; pésala en una báscula de cocina o de joyería. El lastre **debe quedar centrado**, o la pelota girará y bailará.
3. Sella el canal con silicón o resistol.
4. Pinta con **pintura acrílica base agua** naranja fluorescente, en 2 o 3 capas. ⚠️ **No uses pintura en aerosol**: sus solventes derriten el unicel.
5. Termina con barniz acrílico base agua (impermeabiliza; si no, el unicel absorbe agua y cambia de peso).

### Fondo y marcas

| Elemento | Recomendación |
|---|---|
| **Fondo** | **Foamy (EVA) negro mate** detrás del chorro. No absorbe agua, no brilla y es barato. Puede ser lona o triplay pintado de negro mate. |
| **Boquilla** | Fórrala con **termoencogible negro** o píntala de negro mate. El cobre es naranja y se confundiría con la pelota. |
| **Marcadores** | `python generar_marcadores.py` → imprime `marcadores/aruco_id0.pdf` y `aruco_id1.pdf` al **100 %**, recórtalos con su margen blanco y **enmícalos en mate** (el brillante refleja la luz). |
| **Poste** | Tubo de PVC o perfil de aluminio vertical, **a un lado del chorro (20 a 30 cm) y a la misma distancia de la cámara**. ID 0 con su centro a la altura de la salida de la boquilla; ID 1 con su centro **exactamente 1000 mm** arriba (usa flexómetro y nivel). |
| **Iluminación** | Lámpara LED difusa, de frente o de lado. Evita el sol directo y los LEDs baratos que parpadean. |

### Electrónica e hidráulica

Bomba **QWORK DC 12 V 800 L/h 5 m** (brushless), fuente de 12 V a 3 A, PIC18F4550, MOSFET IRLZ44N, adaptador USB-serial y webcam USB genérica (640×480 a 30 fps). La lista completa y el diagrama de conexiones están en el repo **levitador-hidrodinamico-pic**.

Parte hidráulica:

| Pieza | Detalle |
|---|---|
| Depósito | Cubeta de 20 L; la bomba va sumergida, con una esponja en la entrada como filtro. |
| Conducción | Salida de la bomba de 1/2" → manguera de 1/2" → reducción → **tubo de cobre de 1/4"** (diámetro interior ≈ 4.8 mm), de 15 a 20 cm, **recto y vertical**: esa es la boquilla. |
| Embudo de retorno | Charola o embudo inclinado alrededor de la boquilla para que la pelota caída **regrese sola** a la boquilla. La boquilla sobresale 1 o 2 cm del centro. |
| Bypass (opcional, recomendado) | Una T con llave de paso que regresa agua al depósito. Desplaza el rango de trabajo y sirve para aplicar **perturbaciones** en las prácticas. |
| Contención | Paneles laterales y trasero de al menos 1.4 m de alto; el frente queda abierto hacia la cámara. |

---

## 4. Construcción del montaje

```
   vista frontal (lo que ve la cámara, ya girada)

        ┌───────┐
        │ ID 1  │ ← centro a 1000 mm
        └───┬───┘
            │                      ●  pelota
            │                      ┆
            │ poste                ┆  chorro
            │                      ┆
        ┌───┴───┐                  ┆
        │ ID 0  │ ← centro ────────╧── salida de la boquilla (0 mm)
        └───────┘           ╲_____/ ← embudo de retorno
   ░░░░░░░░░░░░░░░░░ fondo foamy negro mate ░░░░░░░░░░░░░░░░░
```

- **Cámara:** en un tripié fijo, a **1.6–2.0 m** de distancia y a media altura (unos 55 cm). Conviene **girarla 90°**, en vertical, para aprovechar los 640 px a lo alto; en la interfaz se pone **Rotación = 90** (o 270).
- En el cuadro deben verse completos el ID 0, el ID 1 y el chorro, con margen.
- El poste y el chorro deben quedar a la **misma distancia de la cámara**. Si no, hay error por paralaje.

---

## 5. Instalación del software

Requiere **Python 3.10 o más reciente** (Windows, Linux o macOS).

```powershell
git clone https://github.com/arturomeneses100/levitador-hidrodinamico-interfaz.git
cd levitador-hidrodinamico-interfaz
python -m venv .venv
.venv\Scripts\activate            # en Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

Dependencias: `opencv-python`, `numpy`, `PySide6`, `pyqtgraph`, `pyserial`. `matplotlib` es opcional y solo se usa con `analizar_csv.py --graficar`.

---

## 6. Uso paso a paso

### A. Primero en simulación (sin hardware)

1. `python main.py`
2. Marca **Modo simulación** → **Conectar** → **ARMAR**.
3. Mueve la **Referencia** y cambia de **Modo**. La cámara, la bomba, el chorro y el PIC son simulados, con la misma lógica del firmware.

Es ideal para que los alumnos se familiaricen con la interfaz antes de mojar nada. También puedes correr la prueba automática del lazo:

```powershell
python pruebas\prueba_lazo_simulado.py
```

### B. Con el equipo real

1. **Conecta** el adaptador USB-serial del PIC y la webcam. Enciende la fuente de 12 V; el PIC arranca en **PARO**.
2. En la interfaz:
   - Elige **Cámara** (0, 1…), pon la **Rotación** y pulsa **Abrir cámara**.
   - Quita la **Exposición auto** y ajusta la **Exposición** (de −4 a −7 suele funcionar) hasta que la pelota se vea naranja intenso y el fondo negro. Si tu webcam no responde, usa **Ajustes driver** (diálogo de Windows).
3. **Verifica los marcadores.** En el video debe aparecer el eje con marcas cada 10 cm, y en la barra inferior, `marcadores OK`.
4. **Calibra el color.** Pon la pelota sobre la boquilla, marca **Calibrar color** y haz **clic sobre la pelota**. Ese clic fija el rango HSV y también la posición del **eje del chorro**. Con **Ver máscara** compruebas que solo la pelota quede en blanco.
5. Elige el **Puerto PIC** → **Conectar**. La interfaz envía todos los parámetros y lee la configuración del PIC.
6. Pulsa **ARMAR**. En modo cerrado el PIC espera a ver la pelota en la boquilla, arranca la bomba y sube la referencia en rampa (300 mm/s) hasta la referencia elegida.
7. **PARO** (o la barra **Espacio**) apaga la bomba de inmediato.
8. Pulsa **Grabar CSV** para guardar los datos en `registros/`.

La configuración (cámara, color, parámetros) se guarda sola en `config.json` al cerrar.

### Analizar los registros

```powershell
python analizar_csv.py metricas    registros\levitador_20261120_101500.csv
python analizar_csv.py feedforward registros\levitador_20261120_100000.csv --graficar
```

---

## 7. Prácticas sugeridas

> Duración aproximada: 2 sesiones de 2 h. Cada equipo entrega su CSV, sus gráficas y una tabla de métricas.

### Práctica 1 — Lazo abierto e identificación

1. Modo **Manual**. Sube el **PWM manual** en pasos de 60 (420, 480, 540… 780) y espera unos 5 s en cada uno, **grabando el CSV**.
2. Anota a partir de qué PWM empieza a girar la bomba (**zona muerta**).
3. `python analizar_csv.py feedforward archivo.csv --graficar` → obtén la **curva estática** y la recta `u = B + W·h`.
4. Con un escalón de PWM (por ejemplo, de 540 a 660), estima en la gráfica un **modelo de primer orden con retardo**: ganancia K, constante de tiempo τ y retardo θ.
5. Aplica una perturbación abriendo el **bypass** y observa que **en lazo abierto la altura no se recupera**.

### Práctica 2 — On-Off con histéresis

1. Modo **On-Off**, referencia de 500 mm, **amplitud** de 120 e **histéresis** de 20 mm.
2. Observa la **oscilación sostenida** y mide su amplitud y su periodo.
3. Repite con histéresis de 0, 20 y 50 mm y con amplitud de 60, 120 y 200. ¿Qué pasa con la frecuencia de conmutación y la amplitud?

### Práctica 3 — Control P y el error en estado estacionario

1. Pon **W = 0** y **B = 527** (el PWM de flotación a 400 mm, sacado de la práctica 1).
2. Modo **P** con Kp de 0.1, 0.25 y 0.5. Aplica escalones de referencia de 400 a 700 y de vuelta a 400.
3. Mide el **error en estado estacionario**. ¿Por qué es cero solo a 400 mm?
4. Ahora pon el feedforward completo de la práctica 1. ¿Cómo cambia el error?

### Práctica 4 — PI y windup

1. Modo **PI**, con Kp de 0.25 y Ki de 0.5. Confirma que el error en estado estacionario es ≈ 0.
2. Varía Ki (0.2, 0.5, 1.0) y compara el sobrepaso y el tiempo de asentamiento.
3. **Windup:** limita **PWM máximo** a 700 y pide 1000 mm durante 10 s, luego regresa a 500. El firmware tiene *anti-windup*. Para comparar, discute qué pasaría sin él.

### Práctica 5 — PID, ruido y perturbaciones

1. Modo **PID** con Kd de 0, 0.04 y 0.1, y filtro α de 1.0, 0.3 y 0.1.
2. Observa el ruido en la gráfica de **u** cuando Kd es grande y α = 1 (sin filtro).
3. Abre el **bypass** de golpe con la pelota a 600 mm: compara la recuperación con P, PI y PID.
4. Prueba la **Rampa de referencia** (V = 200 mm/s) contra el escalón puro (V = 0).

### Práctica 6 — Comparación final

Llena una tabla con `analizar_csv.py metricas` para un escalón de 400 a 700 mm:

| Controlador | Sobrepaso | t asentamiento (5 %) | e_ss | IAE |
|---|---|---|---|---|
| On-Off | | | | |
| P | | | | |
| PI | | | | |
| PID | | | | |

Preguntas de cierre:

- ¿Qué controlador elegirías para este proceso y por qué?
- ¿Qué limita la velocidad de respuesta: la bomba, el agua o la cámara?

---

## 8. Parámetros del controlador

Ley de control en el PIC (cada medición, con `dt` real):

```
u_ff = B + W · sp                                (feedforward)
u    = u_ff + Kp·e + Σ Ki·e·dt − Kd·d(h)/dt      (PID, derivada sobre la medición filtrada)
e    = sp − h          u saturada a [0, PWM máximo]
```

| Parámetro | Por defecto | Unidades | Comentario |
|---|---|---|---|
| Kp | 0.25 | PWM/mm | |
| Ki | 0.50 | PWM/(mm·s) | anti-windup por integración condicional |
| Kd | 0.04 | PWM/(mm/s) | sobre la medición, no sobre el error |
| α (filtro derivada) | 0.30 | — | 1 = sin filtro |
| **B** | 467 | PWM | feedforward: ordenada |
| **W** | 0.15 | PWM/mm | feedforward: pendiente (35 % de la estática) |
| Amplitud On-Off | 120 | PWM | u = u_ff ± amplitud |
| Histéresis On-Off | 20 | mm | |
| PWM máximo | 1023 | PWM | bájalo si salpica demasiado |
| Rampa referencia V | 0 | mm/s | 0 = escalón puro |

Los valores por defecto se obtuvieron con el **simulador** (bomba QWORK, boquilla de 5 mm, pelota de 50 mm y 10 g). En simulación dan un sobrepaso de 6 a 15 mm y un asentamiento menor a 2 s; con la variación de masa de ±30 % no pierde la pelota. **En el equipo real hay que re-identificar B y W (práctica 1) y reajustar.** La planta real será más ruidosa y oscilatoria que el modelo.

**Usa una fracción del feedforward estático (W).** Con la pendiente completa, cada escalón da una "patada" a la bomba y la pelota se pasa. `analizar_csv.py feedforward` ya sugiere el 35 % de la pendiente con B ajustado a media altura.

---

## 9. Seguridad

- ⚠️ **Agua y electricidad.** Solo **12 V DC** cerca del agua. La fuente de 12 V y la PC deben quedar **lejos y más arriba** que la cubeta, con los cables formando un "bucle de goteo".
- **Paro físico:** un botón tipo hongo (normalmente cerrado) en serie con el +12 V de la bomba, independiente del software.
- **Paro lógico:** botón en RB0 del PIC, botón **PARO** o **Espacio** en la interfaz.
- El PIC **apaga la bomba solo** si:
  - deja de recibir datos de la PC durante 300 ms;
  - la pelota no se ve durante 600 ms (se declara caída);
  - se desconecta la interfaz (envía `X` al cerrar).
- El PIC **arranca en PARO**: hay que **ARMAR** a propósito.
- Limita **PWM máximo** si el chorro libre salpica fuera de la contención.

---

## 10. Solución de problemas

| Síntoma | Causa probable / solución |
|---|---|
| "Sin marcadores ArUco 0 y 1" | Están fuera del cuadro, borrosos o con reflejo. Usa enmicado mate, mejor luz y verifica que se impriman al 100 % con su margen blanco. |
| La pelota no se detecta | Recalibra el color con un clic sobre la pelota. Revisa con **Ver máscara**. Baja la exposición si la pelota se ve blanca (saturada). |
| La boquilla o una gota se detectan como pelota | Forra la boquilla de negro. Las gotas aisladas se rechazan solas. |
| Los fps bajan a 15 | Hay poca luz y la exposición automática está activa: quítala y agrega luz. |
| La bomba zumba o no arranca a PWM bajo | Es normal: el driver brushless tiene zona muerta (≈ 35 %). Si se comporta raro, prueba otra frecuencia de PWM en el firmware (`PWM_DIVISOR`). |
| La pelota no baja de cierta altura | Es la velocidad mínima de la bomba. Agrega lastre a la pelota o abre un poco el bypass. |
| La pelota baila mucho o se cae de lado | La boquilla no está vertical, el lastre no está centrado o la pelota pesa muy poco. |
| "SIN ENLACE" en el PIC | Revisa el puerto y el cable USB-serial; la cámara debe seguir enviando datos. |
| La altura sale desfasada | Revisa que el centro del ID 0 esté a la altura de la boquilla y que entre centros haya 1000 mm. |

---

## 11. Estructura del repositorio

```
levitador-hidrodinamico-interfaz/
├── main.py                  Interfaz gráfica (PySide6 + pyqtgraph)
├── vision.py                Medición: ArUco + color HSV + filtros
├── comunicacion.py          Enlace serie con el PIC (y enlace simulado)
├── simulador.py             Planta física + PIC simulado + cámara sintética
├── generar_marcadores.py    PDF imprimibles de los marcadores ArUco
├── analizar_csv.py          Métricas de respuesta y ajuste de feedforward
├── marcadores/              aruco_id0.pdf / aruco_id1.pdf (listos para imprimir)
├── pruebas/
│   └── prueba_lazo_simulado.py
├── docs/
│   └── protocolo.md         Protocolo serie PC ↔ PIC
├── requirements.txt
├── LICENSE                  MIT
└── README.md
```

---

## 12. Protocolo serie

Es ASCII, a 115200 bps, 8N1, una línea por mensaje. El detalle completo está en [`docs/protocolo.md`](docs/protocolo.md).

| PC → PIC | Significado | PIC → PC | Significado |
|---|---|---|---|
| `M<mm>` | altura medida | `T,h,sp,u,modo,estado,dt` | telemetría |
| `N` | pelota no vista | `C,…` | configuración actual |
| `S<mm>` | referencia | `E,<texto>` | eventos |
| `O<0-4>` | modo | | |
| `P` `I` `D` `F` `B` `W` `A` `H` `U` `L` `V` | parámetros | | |
| `R` / `X` / `Z` / `?` | armar / paro / borrar integral / leer config | | |

---

## 13. Licencia

[MIT](LICENSE) © 2026 Arturo Elfego Meneses Flores — DUAL ARD / UTHH.

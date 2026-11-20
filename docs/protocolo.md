# Protocolo serie PC ↔ PIC — Levitador hidrodinámico

- **Físico:** UART del PIC18F4550 (RC6 = TX, RC7 = RX) ↔ adaptador USB-serial.
- **Formato:** 115200 bps, 8 bits, sin paridad, 1 bit de paro.
- **Mensajes:** texto ASCII, **una línea por mensaje**, terminada en `\n` (se acepta también `\r`). Máximo 31 caracteres.
- Los números van en decimal; los reales usan punto (`0.25`).

## PC → PIC

| Comando | Ejemplo | Rango | Descripción |
|---|---|---|---|
| `M<mm>` | `M523` | −200 … 1500 | Altura medida de la pelota (base de la pelota sobre la boquilla). **Dispara un ciclo de control.** Se envía en cada cuadro de la cámara (~30 Hz). |
| `N` | `N` | — | La cámara no vio la pelota en este cuadro. |
| `S<mm>` | `S600` | 0 … 1000 | Referencia de altura. |
| `O<n>` | `O4` | 0 … 4 | Modo: 0 Manual, 1 On-Off, 2 P, 3 PI, 4 PID. El cambio es "sin salto" (precarga la integral). |
| `P<real>` | `P0.25` | | Kp [PWM/mm] |
| `I<real>` | `I0.5` | | Ki [PWM/(mm·s)] |
| `D<real>` | `D0.04` | | Kd [PWM/(mm/s)] |
| `F<real>` | `F0.3` | 0.01 … 1 | α del filtro de la derivada (1 = sin filtro) |
| `B<n>` | `B467` | 0 … 1023 | Feedforward: PWM base (ordenada) |
| `W<real>` | `W0.15` | 0 … 2 | Feedforward: pendiente [PWM/mm]; `u_ff = B + W·sp` |
| `A<n>` | `A120` | 0 … 1023 | On-Off: amplitud, `u = u_ff ± A` |
| `H<mm>` | `H20` | 0 … 300 | On-Off: histéresis |
| `U<n>` | `U600` | 0 … 1023 | PWM en modo Manual |
| `L<n>` | `L900` | 0 … 1023 | PWM máximo (saturación) |
| `V<mm/s>` | `V200` | 0 … 5000 | Rampa de la referencia; 0 = escalón puro |
| `R` | `R` | — | **Armar**. En Manual aplica U; en lazo cerrado pasa a *CAÍDA* y espera ver la pelota para arrancar con rampa. |
| `X` | `X` | — | **Paro**: bomba a 0. |
| `Z` | `Z` | — | Borra la integral. |
| `?` | `?` | — | Pide la configuración (`C,…`). |
| `K` | `K` | — | Latido sin efecto (solo refresca el *watchdog*). |

**Cualquier** línea recibida refresca el *watchdog* de enlace.

## PIC → PC

| Mensaje | Campos |
|---|---|
| `T,h,sp,u,modo,estado,dt` | Telemetría tras cada `M`/`N` y cada 200 ms. `h` altura [mm], `sp` referencia efectiva [mm] (incluye la rampa), `u` PWM 0–1023, `modo` 0–4, `estado` (tabla abajo), `dt` periodo de control medido [ms]. |
| `C,modo,sp,kp,ki,kd,alfa,B,A,H,U,L,W,V` | Configuración actual (al arrancar y con `?`). |
| `E,<texto>` | Eventos: `INICIO…`, `ARMADO`, `PARO_PC`, `PARO_BOTON`, `SIN_ENLACE`. |

## Estados

| N° | Estado | Bomba | Sale de ahí cuando… |
|---|---|---|---|
| 0 | PARO | apagada | llega `R` |
| 1 | SIN ENLACE | apagada | llega cualquier mensaje → se re-arma (CAÍDA o Manual) |
| 2 | CONTROLANDO | controlada | `N` → PERDIDA; `X`/botón → PARO; 300 ms sin mensajes → SIN ENLACE |
| 3 | PELOTA PERDIDA | sostiene la última u | llega `M` → CONTROLANDO; 600 ms sin verla → CAÍDA |
| 4 | PELOTA CAÍDA | apagada | 5 mediciones `M` seguidas → CONTROLANDO (con rampa de arranque) |

## Ejemplo de sesión

```
PIC → E,INICIO Levitador hidrodinamico v1.0
PIC → C,4,400, 0.2500, 0.5000, 0.0400,0.300,467,120,20,0,1023, 0.150,0
PC  → S500
PC  → R
PIC → E,ARMADO
PC  → M2            (x5: la pelota está en la boquilla)
PIC → T,2,2,467,4,2,33
PC  → M40
PIC → T,40,12,470,4,2,33     (la referencia sube en rampa a 300 mm/s)
...
PC  → X
PIC → E,PARO_PC
```

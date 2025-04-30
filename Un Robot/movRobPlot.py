"""
viconMovePlot.py
Visualiza posición, orientación y ejes locales de ROBO_sebas
mientras ejecuta una rutina de movimiento automática.
Requiere:
  - vicon_dssdk
  - robomaster-sdk
  - matplotlib, numpy
Ajusta las IP y el nombre del sujeto en Vicon según tu setup.
"""

import time, threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds
from robomaster import robot, config

# ──────────────────── CONFIGURACIÓN ──────────────────────────
# --- Red / RoboMaster ---------------------------------------
config.LOCAL_IP_STR = "192.168.2.11"   #  ⇦ IP de tu laptop en la red S1
ROBOT_IP            = "192.168.2.19"   #  ⇦ IP del S1 (ROBO_sebas)

# --- Vicon ---------------------------------------------------
HOST, PORT = "localhost", 801
SUBJECT    = "ROBO_sebas"              #  ⇦ nombre exacto en Vicon

# --- Parámetros gráficos ------------------------------------
LEN_AXIS  = 0.30   # m (flechas locales)
TRAIL_LEN = 70

# ────────────────── CONEXIONES INICIALES ─────────────────────
# Vicon
cli = vds.RetimingClient()
cli.Connect(f"{HOST}:{PORT}")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                   vds.Client.AxisMapping.ELeft,
                   vds.Client.AxisMapping.EUp)

print("⏳ Esperando primer frame Vicon…")
while True:
    try:
        cli.UpdateFrame()
        break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame":
            time.sleep(0.05)

# RoboMaster
config.ROBOT_IP_STR = ROBOT_IP
ep = robot.Robot()
print("⏳ Conectando a RoboMaster…")
ep.initialize(conn_type="sta")
chs = ep.chassis
print("✅ Conectado a ROBO_sebas")

# ─────────────────── FIGURA MATPLOTLIB ───────────────────────
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect("equal")
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.grid(True)
ax.set_xlabel("X global (m)")
ax.set_ylabel("Y global (m)")
plt.title("Rastro, ejes locales y rutina de movimiento — ROBO_sebas")

pt,    = ax.plot([], [], "o", color="tab:blue", markersize=10)
trail, = ax.plot([], [], "-", color="tab:blue", alpha=0.6)
arrow_x = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1,
                    color="red", width=0.008, label="+X local")
arrow_y = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1,
                    color="green", width=0.008, label="+Y local")
info  = ax.text(-1.9, -1.85, "", fontsize=9, va="bottom")
ax.legend(loc="upper right")

hist = []

# ──────────────────── FUNCIÓN DE UPDATE ─────────────────────
def update(_):
    try:
        cli.UpdateFrame()
        # Posición global
        px, py, _ = cli.GetSegmentGlobalTranslation(SUBJECT, SUBJECT)[0]
        x, y = px / 1000, py / 1000

        # Yaw en grados 0-360
        yaw_raw = cli.GetSegmentGlobalRotationEulerXYZ(SUBJECT, SUBJECT)[0][2]
        yaw_raw=np.rad2deg(yaw_raw)
        yaw_deg = (yaw_raw + 360) % 360
        yaw_rad = np.deg2rad(yaw_deg)

        # Punto y rastro
        pt.set_data([x], [y])
        hist.append((x, y))
        if len(hist) > TRAIL_LEN:
            hist.pop(0)
        hx, hy = zip(*hist)
        trail.set_data(hx, hy)

        # Ejes locales
        dx_x, dy_x = LEN_AXIS * np.cos(yaw_rad), LEN_AXIS * np.sin(yaw_rad)
        dx_y, dy_y = LEN_AXIS * np.cos(yaw_rad + np.pi/2), LEN_AXIS * np.sin(yaw_rad + np.pi/2)
        arrow_x.set_offsets([x, y]); arrow_x.set_UVC(dx_x, dy_x)
        arrow_y.set_offsets([x, y]); arrow_y.set_UVC(dx_y, dy_y)

        info.set_text(f"X = {x:+.2f} m\nY = {y:+.2f} m\nYaw = {yaw_deg:6.1f}°")
        return [pt, trail, arrow_x, arrow_y, info]

    except vds.DataStreamException:
        return []

# ────────────────── RUTINA DE MOVIMIENTO (drive_speed) ──────────────────
def move_seq():
    """
    Rutina:
        1. Avanza   0.5 m
        2. Retrocede 0.5 m
        3. Desplaza  0.5 m a la izquierda
        4. Desplaza  0.5 m a la derecha

    Se usa drive_wheels(lf, lb, rf, rb, timeout).
    - speed_pct: 0–100   (≈ 1 m/s cuando es 100 %)
    - dist:       m      (por segmento)
    - dur:        s      (dist / velocidad_lineal)
    """
    # ----- Parámetros que puedes tocar fácilmente -----
    dist      = 0.5          # m por segmento
    speed_pct = 40           # % de la velocidad nominal
    speed_mps = 1.0 * speed_pct / 100    # ≈ 0.40 m/s
    dur       = dist / speed_mps         # tiempo p/ recorrer dist
    pause     = 0.3           # s de pausa entre segmentos
    # --------------------------------------------------

    patterns = [
        ("forward",  +speed_pct, +speed_pct, +speed_pct, +speed_pct),
        ("backward", -speed_pct, -speed_pct, -speed_pct, -speed_pct),
        ("left",     -speed_pct, +speed_pct, +speed_pct, -speed_pct),
        ("right",    +speed_pct, -speed_pct, -speed_pct, +speed_pct)
    ]

    try:
        for name, lf, lb, rf, rb in patterns:
            print(f"🚀 {name:8s} {dist} m  —  {dur:.2f}s  @ {speed_mps:.2f} m/s")
            chs.drive_wheels(lf, lb, rf, rb, timeout=dur)
            time.sleep(dur)            # pequeña pausa
        print("🏁 Rutina terminada")
    finally:
        chs.drive_wheels(0, 0, 0, 0, timeout=0.1)  # frena
        ep.close()



# ───────────────────── MAIN / ANIMACIÓN ─────────────────────
anim = FuncAnimation(fig, update, interval=60, blit=True)
threading.Thread(target=move_seq, daemon=True).start()

plt.tight_layout()
plt.show()

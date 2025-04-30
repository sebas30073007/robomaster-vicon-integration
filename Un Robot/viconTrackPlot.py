import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds

# ── Conexión Vicon ───────────────────────────────────────────
HOST, PORT = "localhost", 801
cli = vds.RetimingClient()
cli.Connect(f"{HOST}:{PORT}")

# Eje X = Forward, Y = Left, Z = Up   (convención Vicon RHS)
cli.SetAxisMapping(
    vds.Client.AxisMapping.EForward,
    vds.Client.AxisMapping.ELeft,
    vds.Client.AxisMapping.EUp)

print("Esperando primer frame…")
while True:
    try:
        cli.UpdateFrame()
        break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame":
            time.sleep(0.05)

# ── Parámetros del robot ────────────────────────────────────
ROBOT      = "ROBO_sebas"
COLOR      = "tab:blue"
TRAIL_LEN  = 60
ARROW_LEN  = 0.25           # m
hist       = []

# ── Figura ──────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect("equal")
ax.grid(True)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

pt,   = ax.plot([], [], "o", color=COLOR, markersize=10, label=ROBOT)
trail, = ax.plot([], [], "-", color=COLOR, alpha=0.6)
arrow = ax.quiver(0, 0, 0, 0, angles='xy', scale_units='xy', scale=1,
                  color=COLOR, width=0.006, alpha=0.9)
info  = ax.text(-1.9, -1.8, "", fontsize=9, va="bottom")
ax.legend(loc="upper right")
plt.title("Posición + orientación (yaw) — ROBO_sebas")

# ── Función de actualización ─────────────────────────────────
def update(_):
    try:
        cli.UpdateFrame()

        # ① POSICIÓN (mm → m)
        px, py, _ = cli.GetSegmentGlobalTranslation(ROBOT, ROBOT)[0]
        x, y = px / 1000, py / 1000

        # ② ORIENTACIÓN (Euler XYZ → yaw = rotación sobre Z)
        euler_xyz = cli.GetSegmentGlobalRotationEulerXYZ(ROBOT, ROBOT)[0]  # [X°, Y°, Z°]
        yaw_rad   = euler_xyz[2]            # rotación sobre Z
        yaw_deg   = np.rad2deg(yaw_rad)
        yaw_deg = (yaw_deg + 360) % 360 
        # — Punto y rastro —
        pt.set_data([x], [y])
        hist.append((x, y))
        if len(hist) > TRAIL_LEN:
            hist.pop(0)
        hx, hy = zip(*hist)
        trail.set_data(hx, hy)

        # — Flecha: desde (x,y) apuntando a la dirección “frontal” —
        dx, dy = ARROW_LEN * np.cos(yaw_rad), ARROW_LEN * np.sin(yaw_rad)
        arrow.set_offsets([x, y])
        arrow.set_UVC(dx, dy)

        # — Texto —
        info.set_text(f"X = {x:+.2f} m\nY = {y:+.2f} m\nYaw = {yaw_deg:+.1f}°")

        return [pt, trail, arrow, info]

    except vds.DataStreamException:
        return []

# ── Lanzar animación ─────────────────────────────────────────
anim = FuncAnimation(fig, update, interval=60, blit=True)
plt.tight_layout()
plt.show()
# -*- coding:utf-8 -*-
"""
Control de formación para 2 robots (ROBO_dany y ROBO_manolo) con Vicon.
Usa retroalimentación para mantener una posición relativa deseada entre ambos robots.
"""

import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds
from robomaster import robot, config

# IP de tu laptop
LAPTOP_IP = "192.168.2.11"

# Datos de los robots
robots_info = [
    {"name": "ROBO_dany", "ip": "192.168.2.16", "color": "tab:red"},
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"}
]

Zd = np.array([[0.0, 0.0], [1.0, 1.0]])  # posiciones relativas deseadas

K_FORM = 0.6
V_MAX = 0.4
DT = 0.1

# ────── Conexión Vicon ──────
cli = vds.RetimingClient()
cli.Connect("localhost:801")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                   vds.Client.AxisMapping.ELeft,
                   vds.Client.AxisMapping.EUp)
print("Esperando primer frame Vicon…")
while True:
    try:
        cli.UpdateFrame()
        break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame":
            time.sleep(0.05)

# ────── Conexión a los robots ──────
robots = []
for info in robots_info:
    config.LOCAL_IP_STR = LAPTOP_IP
    config.ROBOT_IP_STR = info["ip"]
    try:
        robot_ep = robot.Robot()
        robot_ep.initialize(conn_type="sta")
        info["ep"] = robot_ep
        info["chs"] = robot_ep.chassis
        robots.append(info)
        print(f"✅ Conectado a {info['name']}")
    except Exception as err:
        print(f"❌ Error conectando a {info['name']}: {err}")
        sys.exit(1)

# ────── Control de formación ──────
def formation_control(Z, Zd, k=K_FORM):
    """
    Cálculo del control de formación para 2 robots.
    Si fueran N robots, se definirían las posiciones relativas Cij = Zdj - Zdi,
    y se aplicaría una ley como:
        ui = -k * sum_j ( (Zi - Zj) - (Zdi - Zdj) )
    """
    C12 = Zd[1] - Zd[0]
    C21 = Zd[0] - Zd[1]

    Z1, Z2 = Z[0], Z[1]
    C1 = Z2 + C21
    C2 = Z1 + C12

    u1 = -k * (Z1 - C1)
    u2 = -k * (Z2 - C2)

    return np.array([u1, u2])

# ────── Loop de control ──────
STOP = False

last_positions = np.array([[0.0, 0.0], [0.0, 0.0]])

def control_loop():
    global STOP, last_positions
    while not STOP:
        cli.UpdateFrame()
        Z = []
        for r in robots_info:
            pos = cli.GetSegmentGlobalTranslation(r["name"], r["name"])[0]
            Z.append([pos[0] / 1000, pos[1] / 1000])
        Z = np.array(Z)
        last_positions = Z.copy()

        U = formation_control(Z, Zd)
        for i, r in enumerate(robots):
            vx, vy = U[i]
            norm = np.linalg.norm([vx, vy])
            if norm > V_MAX:
                vx, vy = vx / norm * V_MAX, vy / norm * V_MAX
            r["chs"].drive_speed(x=vx, y=-vy, z=0)

        time.sleep(DT)

    for r in robots:
        r["chs"].drive_speed(0, 0, 0)
        r["ep"].close()

# ────── Visualización ──────
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_aspect('equal'); ax.grid(True)
ax.set_xlim(-2, 2); ax.set_ylim(-2, 2)

plots = []
trails = []
history = [[] for _ in robots_info]

for info in robots_info:
    pt, = ax.plot([], [], 'o', color=info["color"], markersize=10, label=info["name"])
    tr, = ax.plot([], [], '-', color=info["color"], alpha=0.6)
    plots.append(pt); trails.append(tr)

ax.legend()

def update(_):
    cli.UpdateFrame()
    for i, info in enumerate(robots_info):
        try:
            pos = cli.GetSegmentGlobalTranslation(info["name"], info["name"])[0]
            x, y = pos[0] / 1000, pos[1] / 1000
            plots[i].set_data([x], [y])
            history[i].append((x, y))
            if len(history[i]) > 100: history[i].pop(0)
            hx, hy = zip(*history[i])
            trails[i].set_data(hx, hy)
        except:
            continue
    return plots + trails

# ────── Lanzamiento ──────
if __name__ == "__main__":
    threading.Thread(target=control_loop, daemon=True).start()
    anim = FuncAnimation(fig, update, interval=100, blit=True)
    plt.title("Control de formación: 2 robots")
    plt.tight_layout(); plt.show()
    STOP = True

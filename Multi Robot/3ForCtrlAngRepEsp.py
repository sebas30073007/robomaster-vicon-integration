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
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"},
    {"name": "ROBO_sebas", "ip": "192.168.2.19", "color": "tab:green"}
]

Zd = np.array([[0.0, 0.0], [1.0, 1.0], [-1.0, -1.0]])  # posiciones relativas deseadas

TH_DES    = 0         # rad  (cambia para otro ángulo)
K_YAW     = 1          # ganancia P sobre el error angular
W_MAX     = 2 # rad/s  (saturación)
K_FORM = 3
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
    C13 = Zd[2] - Zd[0]
    C31 = Zd[0] - Zd[2]
    C23 = Zd[2] - Zd[1]
    C32 = Zd[1] - Zd[2]

    Z1, Z2, Z3 = Z[0], Z[1], Z[2]
    C1 = (1/2) * (Z2 + C21 + Z3 + C31)
    C2 = (1/2) * (Z1 + C12 + Z3 + C32)
    C3 = (1/2) * (Z1 + C13 + Z2 + C23)

    u1 = -k * (Z1 - C1)
    u2 = -k * (Z2 - C2)
    u3 = -k * (Z3 - C3)
    
    return np.array([u1, u2, u3])

# ────── Loop de control ──────
STOP = False

last_positions = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])

STOP = False
def control_loop():
    global STOP
    prev_yaw = np.zeros(len(robots))

    while not STOP:
        try:
            cli.UpdateFrame()
        except vds.DataStreamException:
            time.sleep(DT); continue

        # ------------ posiciones actuales ------------
        Z = []
        for r in robots_info:
            pos = cli.GetSegmentGlobalTranslation(r["name"], r["name"])[0]
            Z.append([pos[0]/1000.0, pos[1]/1000.0])
        Z = np.array(Z)

        # ------------ control lineal (formación) ------------
        U = formation_control(Z, Zd)      # m/s en marco global
        for i, r in enumerate(robots):
            vx, vy = U[i]
            v_norm = np.hypot(vx, vy)
            if v_norm > V_MAX:
                vx, vy = vx / v_norm * V_MAX, vy / v_norm * V_MAX

            yaw_rad = cli.GetSegmentGlobalRotationEulerXYZ(r["name"], r["name"])[0][2]
            yaw_deg = np.rad2deg(yaw_rad)

            e_th = TH_DES - yaw_deg
            w_cmd = -K_YAW * e_th
            w_cmd = np.clip(w_cmd, -W_MAX, W_MAX)  
            # y de Vicon apunta a la IZQ; en SDK apunta a FRENTE
            r["chs"].drive_speed(x=vx, y=-vy, z=w_cmd)

        time.sleep(DT)
        print(e_th)

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
    plt.title("Control de formación: 3 robots")
    plt.tight_layout(); plt.show()
    STOP = True

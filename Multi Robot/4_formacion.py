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
    {"name": "ROBO_sebas", "ip": "192.168.2.19", "color": "tab:green"},
    {"name": "ROBO_haili", "ip": "192.168.2.25", "color": "tab:orange"}
]

Zd = np.array([[0.5, 0.5], 
               [0.0, 0.0], 
               [-0.5, -0.5], 
               [-0.5, 0.5]])  # posiciones relativas deseadas

K_FORM = 2.0            # ganancia formación (posición)
V_MAX  = 0.40           # saturación lineal (m/s)
W_MAX  = 1.2            # saturación angular (rad/s)
DT     = 0.10           # periodo de control (s)

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
    C21 = -C12
    C13 = Zd[2] - Zd[0]
    C31 = -C13
    C23 = Zd[2] - Zd[1]
    C32 = -C23
    C14 = Zd[3] - Zd[0]
    C41 = -C14
    C24 = Zd[3] - Zd[1]
    C42 = -C24
    C34 = Zd[3] - Zd[2]
    C43 = -C34

    Z1, Z2, Z3, Z4 = Z[0], Z[1], Z[2], Z[3]
    C1 = (1/3) * (Z2 + C21 + Z3 + C31 + Z4 + C41)
    C2 = (1/3) * (Z1 + C12 + Z3 + C32 + Z4 + C42)
    C3 = (1/3) * (Z1 + C13 + Z2 + C23 + Z4 + C43)
    C4 = (1/3) * (Z1 + C14 + Z2 + C24 + Z3 + C34)
    
    u1 = -k * (Z1 - C1)
    u2 = -k * (Z2 - C2)
    u3 = -k * (Z3 - C3)
    u4 = -k * (Z4 - C4)
    
    return np.array([u1, u2, u3, u4])

# ────── Loop de control ──────
STOP = False

last_positions = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])

def control_loop():
    global STOP, last_positions
    while not STOP:
        try:
            cli.UpdateFrame()
        except vds.DataStreamException:
            time.sleep(DT); continue 
            
        Z = []     # posiciones XY   (m)
        
        for r in robots_info:
            try:
                pos = cli.GetSegmentGlobalTranslation(r["name"], r["name"])[0]
            except vds.DataStreamException:
                break  # si falla un robot, salta este ciclo
            Z.append([pos[0]/1000, pos[1]/1000])
            
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

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

# Zd = np.array([[0.0, 0.0], [1.0, 1.0], [-1.0, -1.0]])  # posiciones relativas deseadas

Ady = [[0, 1, 1], [1, 0, 1], [1, 1, 0]] # adyacencia
n = [2, 2, 2] # colapso matriz A
a = 0.7 # (m)
Cx = np.array([[0, a, 2*a], [-a, 0, a], [-2*a, -a, 0]]) # posiciones rel des en x
Cy = np.array([[0, 2*a, 0], [-2*a, 0, -2*a], [0, 2*a, 0]]) # posiciones rel des en y

TH_DES    = 0         # rad  (cambia para otro ángulo)
K_YAW     = 1          # ganancia P sobre el error angular
W_MAX     = 2 # rad/s  (saturación)
K_FORM = 3
V_MAX = 0.4
DT = 0.1
eta = 15          # Ganancia repulsiva
radiorep = 0.6            # Radio de repulsión

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
def formation_control(Z, Cx=Cx, Cy=Cy, k=K_FORM, eta=eta, r=radiorep, A=Ady):
    Z  = np.asarray(Z,  dtype=float)
    Cx = np.asarray(Cx, dtype=float)
    Cy = np.asarray(Cy, dtype=float)
    A = np.asarray(A, dtype=float)
    U = np.zeros((3, 2))
    for i in range(3):
        for j in range(3):
            U[i, :] = U[i, :] - k * (A[j, i] * (Z[i, :] - Z[j, :] - np.array([Cx[j, i], Cy[j, i]])))
        U[i, :] = U[i, :] / n[i]
    
    R = np.zeros((3, 2))
    for i in range(3):
        for j in range(3):
            d = np.linalg.norm(Z[i, :] - Z[j, :])
            if i != j and d <= r and d > 1e-6:
                #R[i, :] = R[i, :] + eta * (1 - d/r) * (Z[i, :] - Z[j, :])/d # crep
                R[i, :] = R[i, :] + eta * (1 - (r**4) / d) * (Z[i, :] - Z[j, :])/d # espiral
                #R[i, :] = R[i, :] + ((2*eta)/(d**4)) * ((1/(d**2)) - (1/(r**2))) * (Z[i, :] - Z[j, :]) # khatib
    u1 = U[0, :] + R[0, :]
    u2 = U[1, :] + R[1, :]
    u3 = U[2, :] + R[2, :]
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
        U = formation_control(Z)      # m/s en marco global
        
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
def handler(sig, frame):
    global STOP
    print("\nCtrl+C detectado → cerrando…")
    STOP = True
    
    
threading.Thread(target=control_loop, daemon=True).start()
anim = FuncAnimation(fig, update, interval=100, blit=True)
plt.title("Control de formación: 3 robots")
plt.tight_layout(); plt.show()
STOP = True
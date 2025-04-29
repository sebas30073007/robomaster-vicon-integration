# -*- coding:utf-8 -*-
"""
Control de formación y trayectoria lemniscata de Bernoulli con cuatro robots RoboMaster S1,
usando control de consenso basado en matriz de adyacencia para mantener una formación de cuadrado.
Incluye repulsivo simple para evitar colisiones y visualización con rastro semi-histórico.
"""

import sys, time, math, threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robomaster import robot, config
from vicon_dssdk import ViconDataStream as vds

# --- Parámetros de control ---
k_formation = 0.5
eta = 1.5
r = 0.7
V_MAX = 0.30
DT = 0.05
km = 1.0

# --- Configuración de red ---
config.LOCAL_IP_STR = "192.168.2.11"

robots_info = [
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"},
    {"name": "ROBO_peble", "ip": "192.168.2.13", "color": "orange"},
    {"name": "ROBO_dany", "ip": "192.168.2.16", "color": "tab:red"},
    {"name": "ROBO_sebas", "ip": "192.168.2.19", "color": "green"}
]

Zd = np.array([
    [1.0, 0.0],
    [0.0, 1.0],
    [-1.0, 0.0],
    [0.0, -1.0]
])

# Matriz de adyacencia completamente conectada
A = np.array([
    [0, 1, 1, 1],
    [1, 0, 1, 1],
    [1, 1, 0, 1],
    [1, 1, 1, 0]
])

# --- Trayectoria lemniscata de Bernoulli ---
def trajectory_lemniscata(t, a=0.5, omega=0.5):
    theta = omega * t
    denom = 1 + (np.sin(theta))**2
    x = a * np.cos(theta) / denom
    y = a * np.sin(theta) * np.cos(theta) / denom

    dx_dt = a * (-np.sin(theta) * denom - np.cos(theta) * (2*np.sin(theta)*np.cos(theta))) / (denom**2)
    dy_dt = a * (np.cos(theta)**2 - np.sin(theta)**2) / (denom**2)

    pos = np.array([x, y])
    vel = np.array([dx_dt, dy_dt])
    return pos, vel

# --- Conexión a Vicon ---
HOST, PORT = "localhost", 801
cli = vds.RetimingClient()
cli.Connect(f"{HOST}:{PORT}")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                   vds.Client.AxisMapping.ELeft,
                   vds.Client.AxisMapping.EUp)

print("Esperando primer frame...")
while True:
    try:
        cli.UpdateFrame()
        break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame":
            time.sleep(0.05)
        else:
            raise

for robot_info in robots_info:
    config.ROBOT_IP_STR = robot_info["ip"]
    ep = robot.Robot()
    ep.initialize(conn_type="sta")
    robot_info["ep"] = ep
    robot_info["chs"] = ep.chassis

# --- Control de formación con matriz de adyacencia ---
def formation_control(Z, Zd, A, k=k_formation):
    n = Z.shape[0]
    U = np.zeros((n, 2))
    for i in range(n):
        for j in range(n):
            if i != j:
                U[i, :] -= k * A[i, j] * ((Z[i, :] - Z[j, :]) - (Zd[i, :] - Zd[j, :]))
    return U

# --- Campo repulsivo ---
def repulsion_control(Z, eta=eta, r=r):
    n = Z.shape[0]
    R = np.zeros_like(Z)
    for i in range(n):
        for j in range(n):
            if i != j:
                delta = Z[i, :] - Z[j, :]
                d = np.linalg.norm(delta)
                if d <= r and d > 0.0001:
                    delta_dir = delta / d
                    fuerza = eta * (1 - d / r)
                    R[i, :] += fuerza * delta_dir
    return R

last_positions = Zd.copy()
start_time = time.time()

# --- Plot y rastro de posiciones ---
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_aspect("equal")
ax.grid(True)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

plots = []
trails = []
positions_history = [[] for _ in robots_info]

for idx, robot_info in enumerate(robots_info):
    pt, = ax.plot([], [], "o", color=robot_info["color"], markersize=10, label=robot_info["name"])
    trail, = ax.plot([], [], "-", color=robot_info["color"], alpha=0.7)
    plots.append(pt)
    trails.append(trail)

# Dibujar la lemniscata para referencia
theta_ref = np.linspace(0, 2*np.pi, 300)
x_ref = 0.5 * np.cos(theta_ref) / (1 + np.sin(theta_ref)**2)
y_ref = 0.5 * np.sin(theta_ref) * np.cos(theta_ref) / (1 + np.sin(theta_ref)**2)
ax.plot(x_ref, y_ref, "k--", alpha=0.5, label="Trayectoria lemniscata")

legend = ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10)
trail_length = 50

running = True

def control_loop():
    global last_positions, running

    while running:
        t_now = time.time() - start_time
        try:
            cli.UpdateFrame()
        except vds.DataStreamException as e:
            if str(e) == "NoFrame":
                time.sleep(DT)
                continue
            raise

        Z = []
        missing_data = False

        for robot_info in robots_info:
            subject = robot_info["name"]
            segment = subject
            try:
                pos = cli.GetSegmentGlobalTranslation(subject, segment)[0]
                Z.append([pos[0] / 1000, pos[1] / 1000])
            except vds.DataStreamException:
                print(f"No data from {subject}, using last known position.")
                missing_data = True
                break

        if missing_data:
            Z = last_positions.copy()
        else:
            Z = np.array(Z)
            last_positions = Z.copy()

        m, mp = trajectory_lemniscata(t_now)
        U_formation = formation_control(Z, Zd, A)
        R_repulsion = repulsion_control(Z)

        centroid = np.sum(Z, axis=0) / Z.shape[0]
        V_track = -km * (centroid - m) + mp

        U_total = U_formation + R_repulsion
        U_total[0] += -km * (Z[0] - m) + mp
        U_total[1:] += mp

        for idx, robot_info in enumerate(robots_info):
            vx = np.clip(U_total[idx, 0], -V_MAX, V_MAX)
            vy = np.clip(U_total[idx, 1], -V_MAX, V_MAX)
            robot_info["chs"].drive_speed(x=vx, y=-vy, z=0)

        time.sleep(DT)

# --- Actualización del plot ---
def update(_):
    try:
        cli.UpdateFrame()
    except vds.DataStreamException:
        return plots + trails

    artists = []
    for idx, robot_info in enumerate(robots_info):
        try:
            pos = cli.GetSegmentGlobalTranslation(robot_info["name"], robot_info["name"])[0]
            x, y = pos[0] / 1000, pos[1] / 1000
            plots[idx].set_data([x], [y])

            positions_history[idx].append((x, y))
            if len(positions_history[idx]) > trail_length:
                positions_history[idx].pop(0)

            hx, hy = zip(*positions_history[idx])
            trails[idx].set_data(hx, hy)

            artists += [plots[idx], trails[idx]]
        except vds.DataStreamException:
            continue

    return artists

# --- Iniciar el hilo de control ---
control_thread = threading.Thread(target=control_loop, daemon=True)
control_thread.start()

# --- Animación y visualización ---
if __name__ == "__main__":
    anim = FuncAnimation(fig, update, interval=60, blit=True)
    plt.title("Formación y trayectoria lemniscata (4 robots)")
    plt.tight_layout()
    plt.show()

    running = False
    print("Plot cerrado, deteniendo robots y cerrando conexiones...")
    time.sleep(0.1)
    for robot_info in robots_info:
        try:
            robot_info["chs"].drive_speed(0, 0, 0)
            robot_info["ep"].close()
            print(f"{robot_info['name']} detenido y desconectado.")
        except Exception as e:
            print(f"Error al cerrar {robot_info['name']}: {e}")

# -*- coding:utf-8 -*-
"""
Control de formación y trayectoria circular con cuatro robots RoboMaster S1, usando control de consenso para mantener una formación de cuadrado.
Incluye repulsivo simple y tipo espiral para evitar colisiones.
Además, visualización con rastro semi-histórico.
"""

import sys, time, math, threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robomaster import robot, config
from vicon_dssdk import ViconDataStream as vds

# ── Parámetros de control ─────────────────────────────────────
k_formation = 0.5
eta = 1.5
r = 0.7
V_MAX = 0.30
DT = 0.05
km = 1.0

# ── Configuración de red ──────────────────────────────────────
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

# ── Trayectoria circular ─────────────────────────────────────
def trajectory_circle(t, radius=0.3, omega=0.5):
    pos = np.array([radius * np.cos(omega * t), radius * np.sin(omega * t)])
    vel = np.array([-radius * omega * np.sin(omega * t), radius * omega * np.cos(omega * t)])
    return pos, vel

# ── Conexión a Vicon ──────────────────────────────────────────
HOST, PORT = "localhost", 801
cli = vds.RetimingClient()
cli.Connect(f"{HOST}:{PORT}")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
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
        else:
            raise

for robot_info in robots_info:
    config.ROBOT_IP_STR = robot_info["ip"]
    ep = robot.Robot()
    ep.initialize(conn_type="sta")
    robot_info["ep"] = ep
    robot_info["chs"] = ep.chassis

# ── Control de formación ──────────────────────────────────────
def formation_control(Z, Zd, k=k_formation):
    n = Z.shape[0]
    U = np.zeros((n, 2))
    C = Zd.reshape((n, 1, 2)) - Zd.reshape((1, n, 2))
    for i in range(n):
        for j in range(n):
            if i != j:
                U[i, :] -= k * ((Z[i, :] - Z[j, :]) - C[j, i, :])
        U[i, :] /= (n - 1)
    return U

# ── Campo repulsivo simple ────────────────────────────────────
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

# ── Plot y rastro de posiciones ───────────────────────────────
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

legend = ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10)
trail_length = 50

# ── Variable de control para detener el loop ──────────────────
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
                print(f"⚠️ No data from {subject}, using last known position.")
                missing_data = True
                break

        if missing_data:
            Z = last_positions.copy()
        else:
            Z = np.array(Z)
            last_positions = Z.copy()

        m, mp = trajectory_circle(t_now)
        U_formation = formation_control(Z, Zd)
        R_repulsion = repulsion_control(Z)

        centroid = np.sum(Z, axis=0) / Z.shape[0]
        V_track = -km * (centroid - m) + mp

        U_total = U_formation + R_repulsion
        U_total[0] += -km * (Z[0] - m) + mp
        U_total[1] += mp
        U_total[2] += mp
        U_total[3] += mp

        for idx, robot_info in enumerate(robots_info):
            vx = np.clip(U_total[idx, 0], -V_MAX, V_MAX)
            vy = np.clip(U_total[idx, 1], -V_MAX, V_MAX)
            robot_info["chs"].drive_speed(x=vx, y=-vy, z=0)

        time.sleep(DT)

# ── Función de actualización del plot ─────────────────────────
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

# ── Iniciar el hilo de control ────────────────────────────────
control_thread = threading.Thread(target=control_loop, daemon=True)
control_thread.start()

# ── Animación y visualización ────────────────────────────────
if __name__ == "__main__":
    anim = FuncAnimation(fig, update, interval=60, blit=True)
    plt.title("Formación y trayectoria circular (4 robots)")
    plt.tight_layout()
    plt.show()

    # ── Al cerrar la ventana, detener el loop y los robots ─────
    running = False
    print("🛑 Plot cerrado, deteniendo robots y cerrando conexiones...")
    time.sleep(0.1)
    for robot_info in robots_info:
        try:
            robot_info["chs"].drive_speed(0, 0, 0)
            robot_info["ep"].close()
            print(f"✅ {robot_info['name']} detenido y desconectado.")
        except Exception as e:
            print(f"❌ Error al cerrar {robot_info['name']}: {e}")

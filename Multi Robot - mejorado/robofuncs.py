# robofuncs.py
# Funciones utilitarias para Vicon + RoboMaster S1
from typing import List, Dict
import sys, time, threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds
from robomaster import robot, config


# ---------------------------------------------------------------------------
# CONEXIÓN A VICON
# ---------------------------------------------------------------------------
def connect_vicon(host: str = "localhost", port: int = 801) -> vds.RetimingClient:
    """Devuelve un cliente Vicon listo para usarse."""
    cli = vds.RetimingClient()
    cli.Connect(f"{host}:{port}")
    cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                       vds.Client.AxisMapping.ELeft,
                       vds.Client.AxisMapping.EUp)
    print("⌛ Esperando primer frame Vicon…")
    while True:
        try:
            cli.UpdateFrame()
            break
        except vds.DataStreamException as e:
            if str(e) == "NoFrame":
                time.sleep(0.05)
    print("✅ Conectado a Vicon")
    return cli


# ---------------------------------------------------------------------------
# CONEXIÓN A MULTI‑ROBOT
# ---------------------------------------------------------------------------
def connect_robots(robots_info, laptop_ip):
    """
    Inicializa RoboMaster S1 en modo STA para cada entrada del
    diccionario robots_info. Devuelve la lista con objetos 'ep' y
    'chs' agregados.
    """
    connected = []
    for info in robots_info:
        config.LOCAL_IP_STR = laptop_ip
        config.ROBOT_IP_STR = info["ip"]
        try:
            bot = robot.Robot()
            bot.initialize(conn_type="sta")
            info["ep"] = bot
            info["chs"] = bot.chassis
            connected.append(info)
            print(f"🤖  {info['name']} conectado en {info['ip']}")
        except Exception as err:
            print(f"❌  {info['name']} → {err}")
            sys.exit(1)
    return connected


# ---------------------------------------------------------------------------
# PLOTEO EN HILO
# ---------------------------------------------------------------------------
def start_plotting_thread(cli, robots_info, cola_muestras=1):
    """
    Lanza un hilo con matplotlib que actualiza la posición (x, y) y
    la orientación yaw de cada robot en tiempo real.
    """
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect("equal")
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.grid(True)

    # Un punto + flecha por robot
    puntos, flechas = [], []
    for info in robots_info:
        p, = ax.plot([], [], "o", color=info["color"], label=info["name"])
        q = ax.quiver([], [], [], [], color=info["color"], scale=5)
        puntos.append(p)
        flechas.append(q)

    ax.legend()
    plt.title("Posición & orientación (Vicon)")
    plt.tight_layout()

    # --- función de refresco ---
    def update(_frame):
        try:
            cli.UpdateFrame()
        except vds.DataStreamException:
            return puntos + flechas

        for i, info in enumerate(robots_info):
            try:
                pos = cli.GetSegmentGlobalTranslation(info["name"],
                                                      info["name"])[0]
                yaw = cli.GetSegmentGlobalRotationEulerXYZ(info["name"],
                                                           info["name"])[0][2]
                x, y = pos[0] / 1000.0, pos[1] / 1000.0
                puntos[i].set_data([x], [y])
                flechas[i].set_offsets(np.array([[x, y]]))
                flechas[i].set_UVC(0.25 * np.cos(yaw),
                                   0.25 * np.sin(yaw))
            except vds.DataStreamException:
                continue
        return puntos + flechas

    # --- hilo matplotlib ---
    def _runner():
        FuncAnimation(fig, update, interval=100, blit=True)
        plt.show()

    threading.Thread(target=_runner, daemon=True).start()
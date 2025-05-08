# ctrl_angulo.py
# Control P de orientación (yaw) para N RoboMaster S1
import time, threading, signal, sys, numpy as np
from robofuncs import connect_vicon, connect_robots, start_plotting_thread

# --------------------------- configuración ---------------------------
robots_info = [
    {"name": "ROBO_dany",   "ip": "192.168.2.16", "color": "tab:red"},
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"},
    {"name": "ROBO_sebas",  "ip": "192.168.2.19", "color": "tab:green"},
]
LAPTOP_IP = "192.168.2.11"

TH_DES = 0.0          # [deg] referencia de yaw (0 = mirar eje Vicon +X)
K_YAW = 3.0           # ganancia P
W_MAX = 20.0          # [deg/s] saturación
DT     = 0.05         # [s] periodo de control  (≈20 Hz)
# ---------------------------------------------------------------------

# Conexiones
cli    = connect_vicon()
robots = connect_robots(robots_info, LAPTOP_IP)
start_plotting_thread(cli, robots_info)

# ---------- lazo de control ----------
STOP = False
def control_loop():
    global STOP
    while not STOP:
        try:
            cli.UpdateFrame()
        except vds.DataStreamException:
            time.sleep(DT)
            continue

        for r in robots:
            try:
                yaw_rad = cli.GetSegmentGlobalRotationEulerXYZ(
                              r["name"], r["name"])[0][2]
                yaw_deg = np.rad2deg(yaw_rad)

                # Error angular envuelto en (-180, 180] para evitar saltos
                e_th = (yaw_deg - TH_DES + 180) % 360 - 180

                # Control P
                w_cmd = -K_YAW * e_th          # [deg/s]
                w_cmd = np.clip(w_cmd, -W_MAX, W_MAX)

                r["chs"].drive_speed(x=0, y=0, z=w_cmd)
            except vds.DataStreamException:
                continue

        time.sleep(DT)

    # —— apagado limpio ——
    for r in robots:
        r["chs"].drive_speed(0, 0, 0)
        r["ep"].close()
    print("🛑 Robots detenidos. Bye!")

# Manejar Ctrl +C para salir ordenadamente
def handler(sig, frame):
    global STOP
    print("\nCtrl+C detectado → cerrando…")
    STOP = True

signal.signal(signal.SIGINT, handler)

# Ejecutar en primer hilo
control_loop()
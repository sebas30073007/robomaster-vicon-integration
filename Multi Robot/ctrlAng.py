# -*- coding:utf-8 -*-
"""
Control SOLO de orientación (yaw) para 4 robots RoboMaster S1 con feedback Vicon.
Cada robot gira hasta quedar con θ = 0 rad (mirando eje +X global).
No se envían velocidades lineales – solo velocidad angular z.

Robots:
  • ROBO_dany  ─ IP 192.168.2.16
  • ROBO_manolo─ IP 192.168.2.14
  • ROBO_sebas ─ IP 192.168.2.19
  • ROBO_haili ─ IP 192.168.2.25
"""

import sys, time, threading, numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds
from robomaster import robot, config

# ------------- parámetros -----------------
LAPTOP_IP = "192.168.2.11"
TH_DES    = 0         # rad  (cambia para otro ángulo)
K_YAW     = 1          # ganancia P sobre el error angular
W_MAX     = 2 # rad/s  (saturación)
DT        = 0.05          # 20 Hz
# ------------------------------------------

robots_info = [
    {"name": "ROBO_dany",   "ip": "192.168.2.16", "color": "tab:red"},
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"},
    {"name": "ROBO_sebas",  "ip": "192.168.2.19", "color": "tab:green"},
]

# ---------- conexión Vicon ----------
cli = vds.RetimingClient()
cli.Connect("localhost:801")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                   vds.Client.AxisMapping.ELeft,
                   vds.Client.AxisMapping.EUp)
print("Esperando primer frame Vicon…")
while True:
    try:
        cli.UpdateFrame(); break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame": time.sleep(0.05)

# ---------- conexión robots ----------
robots = []
for info in robots_info:
    config.LOCAL_IP_STR = LAPTOP_IP
    config.ROBOT_IP_STR = info["ip"]
    try:
        bot = robot.Robot(); bot.initialize(conn_type="sta")
        info["ep"]  = bot
        info["chs"] = bot.chassis
        robots.append(info)
        print(f"✅ Conectado a {info['name']}")
    except Exception as err:
        print(f"❌ Error conectando a {info['name']}: {err}")
        sys.exit(1)

def wrap(deg):
    """Envuelve a rango (-180, 180] (°)."""
    return (deg + 180) % 360 - 180

# ---------- lazo de control ----------
STOP = False
def orient_control():
    global STOP
    while not STOP:
        try:
            cli.UpdateFrame()
        except vds.DataStreamException as e:
            if str(e) == "NoFrame":
                time.sleep(DT); continue
            else:
                raise

        for r in robots:
            try:
                # Yaw actual en radianes  →  grados
                yaw_rad  = cli.GetSegmentGlobalRotationEulerXYZ(
                                r["name"], r["name"])[0][2]
                yaw_deg = np.rad2deg(yaw_rad)

                e_th = wrap(TH_DES - yaw_deg)   # error más corto (signo CCW positivo)

                w_cmd = K_YAW * e_th            # [°/s]
                w_cmd = np.clip(w_cmd, -W_MAX, W_MAX)
                
                # Solo rotación; sin traslación
                r["chs"].drive_speed(x=0, y=0, z=w_cmd)
            except vds.DataStreamException:
                continue
        time.sleep(DT)

    # al salir, detener y cerrar
    for r in robots:
        r["chs"].drive_speed(0,0,0)
        r["ep"].close()

threading.Thread(target=orient_control, daemon=True).start()

# ---------- visualización (opcional) ----------
fig, ax = plt.subplots(figsize=(6,6)); ax.set_aspect('equal'); ax.grid(True)
ax.set_xlim(-2,2); ax.set_ylim(-2,2)
plots, arrows = [], []
for info in robots_info:
    p, = ax.plot([],[],'o',color=info["color"],label=info["name"])
    q = ax.quiver([],[],[],[],color=info["color"],scale=5)
    plots.append(p); arrows.append(q)
ax.legend()

def update(_):
    try:
        cli.UpdateFrame()
    except vds.DataStreamException:
        return plots+arrows
    for i,info in enumerate(robots_info):
        try:
            pos = cli.GetSegmentGlobalTranslation(info["name"],info["name"])[0]
            yaw = cli.GetSegmentGlobalRotationEulerXYZ(info["name"],info["name"])[0][2]
            x,y = pos[0]/1000, pos[1]/1000
            plots[i].set_data([x],[y])
            arrows[i].set_offsets(np.array([[x,y]]))
            arrows[i].set_UVC(0.25*np.cos(yaw),0.25*np.sin(yaw))
        except vds.DataStreamException:
            continue
    return plots+arrows

anim = FuncAnimation(fig, update, interval=100, blit=True)
plt.title("Control solo de orientación (θ → 0 rad)")
plt.tight_layout(); plt.show()
STOP = True

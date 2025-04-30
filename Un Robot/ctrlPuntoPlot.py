"""
robomaster_point_control.py
Control punto proporcional usando Vicon + RoboMaster S1 (ROBO_sebas).
Incluye:
  • Conexión robusta con try/except.
  • Loop de control (thread) a 10 Hz.
  • Visualización Matplotlib + Vicon.
"""

import sys
import time
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds
from robomaster import robot, config

# ───────────── Parámetros generales ─────────────
LAPTOP_IP = "192.168.2.11"   # IP de tu laptop
ROBOT_IP  = "192.168.2.19"   # IP del S1
SUBJECT   = "ROBO_sebas"     # Nombre en Vicon

# Punto meta global (m)
X_DES, Y_DES = -1.0, -1.0
META = np.array([X_DES, Y_DES])

# Ganancias y límites
K_P   = 3          # P lineal
V_MAX = 1          # m/s máx
LOOP_DT = 0.1        # s (10 Hz)

# Geometría S1 (m)
r_wheel = 0.05
L = 0.11
l_ = 0.105

# ───────────── Conexión Vicon ─────────────
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
        if str(e) == "NoFrame":
            time.sleep(0.05)

# ───────────── Conexión RoboMaster (con try/except) ─────────────
config.LOCAL_IP_STR = LAPTOP_IP
config.ROBOT_IP_STR = ROBOT_IP
try:
    robot_ep = robot.Robot()
    robot_ep.initialize(conn_type="sta")
    chs = robot_ep.chassis
    print("✅ Conectado a RoboMaster S1 (ROBO_sebas)")
except Exception as err:
    print(f"❌ Error conectando al RoboMaster: {err}")
    sys.exit(1)

# ───────────── Loop de control ─────────────
STOP_FLAG = False

def control_loop():
    global STOP_FLAG
    W_inv = (1/r_wheel) * np.array([[ 1, 1, -(L+l_)],
                                    [ 1, -1,  (L+l_)],
                                    [ 1, 1, (L+l_)],
                                    [ 1, -1,  -(L+l_)]])
    while not STOP_FLAG:
        cli.UpdateFrame()
        # Pose actual
        px, py, _ = cli.GetSegmentGlobalTranslation(SUBJECT, SUBJECT)[0]
        pos = np.array([px, py]) / 1000.0
        err = META - pos
        u_bar = K_P * err
        # Saturación lineal
        norm = np.linalg.norm(u_bar)
        if norm > V_MAX:
            u_bar = (u_bar / norm) * V_MAX
        # Orientación (rad)
        yaw = cli.GetSegmentGlobalRotationEulerXYZ(SUBJECT, SUBJECT)[0][2]
        R_gl2body = np.array([[ np.cos(yaw),  np.sin(yaw)],
                              [-np.sin(yaw),  np.cos(yaw)]])
        v_body = R_gl2body @ u_bar
        xi_body = np.array([v_body[0], v_body[1], 0.0])  # Wz=0
        wheel_rads = W_inv @ xi_body
        # Normaliza a %PWM
        pwm = (wheel_rads / 30.0) * 100.0
        pwm = np.clip(pwm, -100, 100)
        lf, lb, rf, rb = pwm
        chs.drive_wheels(lf, lb, rf, rb, timeout=LOOP_DT*0.6)
        time.sleep(LOOP_DT)
    chs.drive_wheels(0,0,0,0,timeout=0.2)
    robot_ep.close()

# ───────────── Loop de control con drive_speed ─────────────
def control_loop_speed():
    """
    Publica velocidades lineales (Vx_body, Vy_body) con drive_speed().
    No se usan las ruedas individuales.
    """
    global STOP_FLAG
    while not STOP_FLAG:
        cli.UpdateFrame()

        # --- Estado actual ---
        px, py, _ = cli.GetSegmentGlobalTranslation(SUBJECT, SUBJECT)[0]
        pos = np.array([px, py]) / 1000.0                    # m
        err = META - pos                                     # error global

        # --- Ley de control P saturada ---
        v_des_glob = K_P * err
        n = np.linalg.norm(v_des_glob)
        if n > V_MAX:
            v_des_glob = (v_des_glob / n) * V_MAX            # limitador

        # --- Convierte a frame cuerpo ---
        yaw = cli.GetSegmentGlobalRotationEulerXYZ(SUBJECT, SUBJECT)[0][2]
        R = np.array([[ np.cos(yaw),  np.sin(yaw)],
                      [-np.sin(yaw),  np.cos(yaw)]])
        v_body = R @ v_des_glob                              # [Vx_body, Vy_body]
        v_body[1] *= -1

        # --- Comando al chasis ---
        chs.drive_speed(x=v_body[0],    # m/s adelante (+) / atrás (–)
                        y=v_body[1],    # m/s izquierda (+) / derecha (–)
                        z=0,            # °/s giro (0 en este control)
                        timeout=LOOP_DT * 0.8)

        time.sleep(LOOP_DT)

    # Al salir: freno y cierre
    chs.drive_speed(0, 0, 0, timeout=0.2)
    robot_ep.close()


# ─────────── Visualización ───────────
fig, ax = plt.subplots(figsize=(6,6)); ax.set_aspect('equal'); ax.grid(True)
ax.set_xlim(-2,2); ax.set_ylim(-2,2)
ax.plot(*META,'r*',ms=12,label='Meta')
pt, = ax.plot([], [], 'bo', label='Robot'); trail, = ax.plot([],[],'b-')
arrow = ax.quiver(0,0,0,0,angles='xy',scale_units='xy',scale=1,
                  color='orange',width=0.008,label='+X local')
info_txt = ax.text(0.02,0.02,'',transform=ax.transAxes,fontsize=8,
                   va='bottom',ha='left',bbox=dict(boxstyle='round',fc='w',alpha=0.6))
ax.legend(loc='upper right')
hist=[]

def update(_):
    try:
        cli.UpdateFrame()
        px,py,_ = cli.GetSegmentGlobalTranslation(SUBJECT,SUBJECT)[0]
        x,y = px/1000, py/1000
        yaw = cli.GetSegmentGlobalRotationEulerXYZ(SUBJECT,SUBJECT)[0][2]
        yaw_deg = (np.rad2deg(yaw)+360)%360
        # punto & rastro
        pt.set_data([x],[y]); hist.append((x,y));
        if len(hist)>120: hist.pop(0)
        hx,hy=zip(*hist); trail.set_data(hx,hy)
        # flecha
        dx,dy = 0.25*np.cos(yaw), 0.25*np.sin(yaw)
        arrow.set_offsets([x,y]); arrow.set_UVC(dx,dy)
        # texto info
        info_txt.set_text(f"X={x:+.2f} m\nY={y:+.2f} m\nYaw={yaw_deg:5.1f}°")
        return pt, trail, arrow, info_txt
    except vds.DataStreamException:
        return []

# ─────────── Lanzamiento ───────────
if __name__=='__main__':
    threading.Thread(target=control_loop_speed, daemon=True).start()
    anim = FuncAnimation(fig, update, interval=100, blit=True)
    plt.tight_layout(); plt.show(); STOP = True
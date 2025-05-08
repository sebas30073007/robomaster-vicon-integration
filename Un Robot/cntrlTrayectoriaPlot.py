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
ROBOT_IP  = "192.168.2.14"   # IP del S1
SUBJECT   = "ROBO_manolo"     # Nombre en Vicon

# Punto meta global (m)
X_DES, Y_DES = -1.0, -1.0
META = np.array([X_DES, Y_DES])

# Ganancias y límites
K_TRACK = 3.0      # posición
K_YAW   = 5.0      # orientación
V_MAX   = 0.5      # m/s máx
DT      = 0.1     # 20 Hz control

# Trayectoria circular
R_TRAJ  = 1.0      # m   radio
OMEGA   = 0.2      # rad/s  (≈15.7 s por vuelta)
START_T = time.time()

# ───────────── Trayectoria Circular ─────────────
def trajectory_circle(t, r=R_TRAJ, omega=OMEGA):
    pos = np.array([r*np.cos(omega*t), r*np.sin(omega*t)])
    vel = np.array([-r*omega*np.sin(omega*t), r*omega*np.cos(omega*t)])
    acc = np.array([-r*omega**2*np.cos(omega*t), -r*omega**2*np.sin(omega*t)])
    return pos, vel, acc

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
STOP = False

def control_loop_tray():
    global STOP
    while not STOP:
        cli.UpdateFrame()
        t = time.time() - START_T
        
        # ── trayectoria deseada (pos, vel, acc) ──
        xd, vd, ad = trajectory_circle(t)
        
        # ── estado actual ──
        px, py, _ = cli.GetSegmentGlobalTranslation(SUBJECT, SUBJECT)[0]
        pos = np.array([px, py]) / 1000.0
        err = pos - xd
        
        # Ley de control en global
        u = -K_TRACK * err + vd*0.1            # feed‑forward + P
        n = np.linalg.norm(u)
        if u [0]> V_MAX:
            u[0] =  V_MAX
        if u [1]> V_MAX:
            u[1] =  V_MAX
            
            
        if u [0]< -V_MAX:
            u[0] =  -V_MAX
        if u [1]< -V_MAX:
            u[1] =  -V_MAX
            
        # ── orientación ──
        yaw = cli.GetSegmentGlobalRotationEulerXYZ(SUBJECT, SUBJECT)[0][2]   # rad
        th_des = np.arctan2(vd[1], vd[0]) + np.pi
        # velocidad angular deseada (th_des_dot)
        num   = ad[0] * vd[1] - vd[0] * ad[1]        # (ax * vy  - vx * ay)
        den   = vd[0]**2 + vd[1]**2            # evita /0
        th_des_dot = num / den                       # rad/s
        
        # control P en yaw con feed‑forward
        e_th = ((yaw - th_des + np.pi) % (2*np.pi)) - np.pi
        #e_th = ((yaw - th_des))
        w_cmd = 0.1*th_des_dot - K_YAW * e_th            # rad/s
        
        # ── transforma a cuerpo ──
        R = np.array([[ np.cos(yaw),  np.sin(yaw)],
                      [-np.sin(yaw),  np.cos(yaw)]])
        v_body = R @ u
        v_body[1] *= -1     # convención SDK (+y = derecha)
        
        chs.drive_speed(x=v_body[0],
                        y=v_body[1],
                        z=w_cmd,         # °/s para SDK
                        timeout=DT * 2)
        print(np.rad2deg(w_cmd))

        time.sleep(DT)
    chs.drive_speed(0, 0, 0, timeout=0.2)
    ep.close() 

# ─────────── Visualización ───────────
fig, ax = plt.subplots(figsize=(6,6)); ax.set_aspect('equal'); ax.grid(True)
ax.set_xlim(-2,2); ax.set_ylim(-2,2)

phi = np.linspace(0,2*np.pi,200)
ax.plot(R_TRAJ*np.cos(phi), R_TRAJ*np.sin(phi), 'r--', label='Trayectoria')

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
        if len(hist)>150: hist.pop(0)
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
    threading.Thread(target=control_loop_tray, daemon=True).start()
    anim = FuncAnimation(fig, update, interval=100, blit=True)
    plt.tight_layout(); plt.show(); STOP = True
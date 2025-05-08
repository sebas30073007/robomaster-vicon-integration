import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parámetros y parametrización
a = 1.0
t = np.linspace(0, 2 * np.pi, 1000)
x = (a * np.sqrt(2) * np.cos(t)) / (1 + np.sin(t)**2)
y = (a * np.sqrt(2) * np.cos(t) * np.sin(t)) / (1 + np.sin(t)**2)

# Derivadas
dx_dt = a * np.sqrt(2) * (np.sin(t)**3 - 3*np.sin(t)) / (1 + np.sin(t)**2)**2
dy_dt = a * np.sqrt(2) * (1 - 3*np.sin(t)**2) / (1 + np.sin(t)**2)**2

# --- Plot 1: Trayectoria de la derivada (dx/dt vs dy/dt) ---
fig1, ax1 = plt.subplots()
ax1.plot(x, y)
point1, = ax1.plot([], [], marker='o')
ax1.set_aspect('equal', 'box')
ax1.set_xlabel('dx/dt')
ax1.set_ylabel('dy/dt')
ax1.set_title('Trayectoria de la derivada')

def init1():
    point1.set_data([], [])
    return point1,

def update1(i):
    point1.set_data(x[i], y[i])
    #point1.set_data(dx_dt[i], dy_dt[i])
    return point1,

ani1 = FuncAnimation(fig1, update1, frames=len(t), init_func=init1, interval=20, blit=True)

# --- Plot 2: x(t) y dx/dt(t) vs t ---
fig2, ax2 = plt.subplots()
ax2.plot(t, x)
#ax2.plot(t, dx_dt)
point2_x, = ax2.plot([], [], marker='o')
#point2_dx, = ax2.plot([], [], marker='.')
ax2.set_xlabel('t')
ax2.set_ylabel('x y dx/dt')
ax2.set_title('x(t) y dx/dt(t) vs t')

def init2():
    point2_x.set_data([], [])
    #point2_dx.set_data([], [])
    return point2_x, point2_dx

def update2(i):
    point2_x.set_data(t[i], x[i])
#    point2_dx.set_data(t[i], dx_dt[i])
    return point2_x, point2_dx

ani2 = FuncAnimation(fig2, update2, frames=len(t), init_func=init2, interval=20, blit=True)

# --- Plot 3: y(t) y dy/dt(t) vs t ---
fig3, ax3 = plt.subplots()
ax3.plot(t, y)
#ax3.plot(t, dy_dt)
point3_y, = ax3.plot([], [], marker='o')
#point3_dy, = ax3.plot([], [], marker='.')
ax3.set_xlabel('t')
ax3.set_ylabel('y y dy/dt')
ax3.set_title('y(t) y dy/dt(t) vs t')

def init3():
    point3_y.set_data([], [])
#    point3_dy.set_data([], [])
    return point3_y, point3_dy

def update3(i):
    point3_y.set_data(t[i], y[i])
#    point3_dy.set_data(t[i], dy_dt[i])
    return point3_y, point3_dy

ani3 = FuncAnimation(fig3, update3, frames=len(t), init_func=init3, interval=20, blit=True)

plt.show()

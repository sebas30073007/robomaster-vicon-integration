import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robomaster import robot, config
from vicon_dssdk import ViconDataStream as vds

# ── Configuración de red y Vicon ─────────────────────────────
config.LOCAL_IP_STR = "192.168.2.11"

robots_info = [
    {"name": "ROBO_manolo", "ip": "192.168.2.14", "color": "tab:blue"},
    {"name": "ROBO_peble", "ip": "192.168.2.25", "color": "orange"},
    {"name": "ROBO_dany", "ip": "192.168.2.16", "color": "tab:red"},
    {"name": "ROBO_sebas", "ip": "192.168.2.19", "color": "green"}
]

robots = []

# Conexión Vicon
HOST, PORT = "localhost", 801
cli = vds.RetimingClient()
cli.Connect(f"{HOST}:{PORT}")
cli.SetAxisMapping(vds.Client.AxisMapping.EForward,
                   vds.Client.AxisMapping.ELeft,
                   vds.Client.AxisMapping.EUp)

print("Esperando primer frame de Vicon…")
while True:
    try:
        cli.UpdateFrame()
        break
    except vds.DataStreamException as e:
        if str(e) == "NoFrame":
            time.sleep(0.05)
        else:
            raise

# ── Conexión a cada robot ─────────────────────────────────────
for robot_info in robots_info:
    config.ROBOT_IP_STR = robot_info["ip"]
    ep = robot.Robot()
    try:
        ep.initialize(conn_type="sta")
        robot_info["ep"] = ep
        robot_info["chs"] = ep.chassis
        robots.append(robot_info)
        print(f"✅ Conectado a {robot_info['name']} ({robot_info['ip']})")
    except Exception as e:
        print(f"❌ Error al conectar a {robot_info['name']} ({robot_info['ip']}): {e}")

# ── Función para mover todos los robots ───────────────────────
def move_all(direction, distance=0.5, speed=0.7):
    dir_map = {
        'forward': (distance, 0, 0),
        'backward': (-distance, 0, 0),
        'left': (0, distance, 0),
        'right': (0, -distance, 0)
    }
    if direction not in dir_map:
        print("⚠️ Dirección no válida. Usa: 'forward', 'backward', 'left', 'right'")
        return

    x, y, z = dir_map[direction]
    print(f"🚀 Moviendo todos los robots {direction} ({distance} m)...")

    for robot_info in robots:
        try:
            robot_info["chs"].move(x, y, z, speed).wait_for_completed()
            print(f"✅ {robot_info['name']} completó el movimiento.")
        except Exception as e:
            print(f"❌ Error moviendo a {robot_info['name']}: {e}")

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

# ── Movimiento y visualización ────────────────────────────────
if __name__ == "__main__":
    from matplotlib.animation import FuncAnimation
    import threading

    anim = FuncAnimation(fig, update, interval=60, blit=True)
    plt.title("Visualización con rastro semi-histórico y movimiento")
    plt.tight_layout()

    def move_sequence():
        time.sleep(1)
        move_all('forward')
        time.sleep(1)
        move_all('backward')
        time.sleep(1)
        move_all('left')
        time.sleep(1)
        move_all('right')
        print("🏁 Movimientos terminados.")
        # Cerrar las conexiones al final (opcional):
        for robot_info in robots:
            robot_info["ep"].close()

    # ── Lanzar el movimiento en paralelo ──────────────────────
    mover_thread = threading.Thread(target=move_sequence)
    mover_thread.start()
    
    plt.show()
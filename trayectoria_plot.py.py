import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from vicon_dssdk import ViconDataStream as vds

# ── Configuración de Vicon ───────────────────────────────────
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

# ── Lista de objetos (robots) ────────────────────────────────
robots_names = [
    "ROBO_manolo",
    "ROBO_sebas",
    "ROBO_dany",
    "ROBO_diegistic",
    "ROBO_peble"
]

colors = ["tab:blue", "tab:green", "tab:red", "yellow", "orange"]
trail_length = 50  # ← Número de puntos del rastro (ajusta aquí)

# ── Preparar el plot ─────────────────────────────────────────
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_aspect("equal")
ax.grid(True)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)

plots = []
trails = []
positions_history = [[] for _ in robots_names]

for idx, name in enumerate(robots_names):
    pt, = ax.plot([], [], "o", color=colors[idx], markersize=10, label=name)
    trail, = ax.plot([], [], "-", color=colors[idx], alpha=0.7)  # Trail más tenue (alpha)
    plots.append(pt)
    trails.append(trail)

# ── Leyenda fuera del gráfico ────────────────────────────────
legend = ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize=10)

# ── Función de actualización ─────────────────────────────────
def update(_):
    try:
        cli.UpdateFrame()
    except vds.DataStreamException:
        return plots + trails

    artists = []
    for idx, name in enumerate(robots_names):
        try:
            pos = cli.GetSegmentGlobalTranslation(name, name)[0]
            x, y = pos[0] / 1000, pos[1] / 1000

            # Actualizar posición actual
            plots[idx].set_data([x], [y])

            # Guardar historial
            positions_history[idx].append((x, y))
            if len(positions_history[idx]) > trail_length:
                positions_history[idx].pop(0)

            # Actualizar trail (historial)
            hx, hy = zip(*positions_history[idx])
            trails[idx].set_data(hx, hy)

            artists += [plots[idx], trails[idx]]
        except vds.DataStreamException:
            continue

    return artists

# ── Animar plot ──────────────────────────────────────────────
plt.title("Visualización con rastro semi-histórico (5 robots)")
anim = FuncAnimation(fig, update, interval=60, blit=True)
plt.tight_layout()
plt.show()

"""
Validação visual dos Módulos 1 (TrajectoryGenerator) e 2 (StationLayoutGenerator).

Gera:
  - trajectory_views.png: vista lateral (x-z) e vista superior (x-y) do
    ground truth, além dos perfis de velocidade, para inspeção de suavidade,
    início/fim e altura máxima no meio do percurso.
  - scenario_1_layout.png / scenario_2_layout.png / scenario_3_layout.png:
    vista superior (x-y) de cada cenário, com as estações e a trajetória
    sobrepostas, para inspeção da geometria.
"""

import matplotlib.pyplot as plt

from config import TRAJECTORY, STATIONS
from trajectory_generator import generate_trajectory
from station_layout_generator import generate_stations

OUT_DIR = "./outputs"


def plot_trajectory(df):
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # Vista lateral (x-z) - perfil do arco
    ax = axes[0, 0]
    ax.plot(df.x, df.z, color="tab:blue")
    ax.scatter([df.x.iloc[0], df.x.iloc[-1]], [df.z.iloc[0], df.z.iloc[-1]],
               color="black", zorder=5, label="início/fim")
    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("z - Up / altitude (m)")
    ax.set_title("Vista lateral (perfil do arco)")
    ax.legend()
    ax.grid(alpha=0.3)

    # Vista superior (x-y)
    ax = axes[0, 1]
    ax.plot(df.x, df.y, color="tab:blue")
    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("y - North / profundidade (m)")
    ax.set_title("Vista superior (top-down)")
    ax.set_ylim(df.y.min() - 10, df.y.max() + 10)
    ax.grid(alpha=0.3)

    # Velocidade escalar e vertical ao longo do tempo
    ax = axes[1, 0]
    ax.plot(df.t, df.speed, label="speed (escalar)")
    ax.plot(df.t, df.vertical_speed, label="vertical_speed")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("m/s")
    ax.set_title("Perfis de velocidade")
    ax.legend()
    ax.grid(alpha=0.3)

    # Course over ground ao longo do tempo
    ax = axes[1, 1]
    ax.plot(df.t, df.course_over_ground, color="tab:green")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("course over ground (graus)")
    ax.set_title("Rumo de deslocamento")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/trajectory_views.png", dpi=150)
    plt.close(fig)


def plot_scenario(scenario_id, traj_df, stations_df):
    fig, ax = plt.subplots(figsize=(7, 6))

    ax.plot(traj_df.x, traj_df.y, color="tab:blue", label="trajetória (ground truth)")
    ax.scatter([traj_df.x.iloc[0]], [traj_df.y.iloc[0]], color="green",
               zorder=5, label="início", marker="^", s=80)
    ax.scatter([traj_df.x.iloc[-1]], [traj_df.y.iloc[-1]], color="red",
               zorder=5, label="fim", marker="s", s=80)

    ax.scatter(stations_df.x, stations_df.y, color="black", marker="X",
               s=150, zorder=6, label="estações")
    for _, row in stations_df.iterrows():
        ax.annotate(f"E{int(row.station_id)}", (row.x, row.y),
                    textcoords="offset points", xytext=(8, 8), fontsize=10)

    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("y - North (m)")
    ax.set_title(f"Cenário {scenario_id} - vista superior")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/scenario_{scenario_id}_layout.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    traj_df = generate_trajectory(TRAJECTORY)
    plot_trajectory(traj_df)
    print("Gerado: trajectory_views.png")

    for sc in (1, 2, 3):
        stations_df = generate_stations(sc, TRAJECTORY, STATIONS)
        plot_scenario(sc, traj_df, stations_df)
        print(f"Gerado: scenario_{sc}_layout.png")

    print("\nValidação visual dos Módulos 1 e 2 concluída.")
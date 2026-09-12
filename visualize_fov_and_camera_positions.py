"""
Visualização do campo de visão (FOV) das câmeras sobre o layout de cada
cenário, e exportação da posição 3D estimada pela câmera (por estação, a
cada instante) ao longo da trajetória.

Gera:
  - scenario_{1,2,3}_fov.png: vista superior de cada cenário, com o setor
    de FOV (90°, apontado para o boresight fixo de cada estação) desenhado
    sobre a trajetória.
  - camera_position_estimates_scenario_{1,2,3}.csv: posição 3D estimada
    pela câmera de cada estação, a cada instante em que o drone esteve no
    FOV, junto com o ground truth e o erro de posição 3D.
  - camera_position_estimates_summary.png: comparação visual entre a
    posição estimada por câmera (por estação) e o ground truth, cenário 1.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge

from config import TRAJECTORY, STATIONS, CAMERA
from station_layout_generator import generate_stations
from trajectory_generator import generate_trajectory
from camera_sensor_model import _station_boresight, generate_camera_position_estimates

OUT_DIR = "./outputs"


def plot_scenario_with_fov(scenario_id, traj_df, stations_df, camera_config, fov_range_m=130):
    fig, ax = plt.subplots(figsize=(8, 7))

    ax.plot(traj_df.x, traj_df.y, color="tab:blue", lw=2, zorder=5,
            label="trajetória (ground truth)")
    ax.scatter([traj_df.x.iloc[0]], [traj_df.y.iloc[0]], color="green",
               zorder=6, label="início", marker="^", s=80)
    ax.scatter([traj_df.x.iloc[-1]], [traj_df.y.iloc[-1]], color="red",
               zorder=6, label="fim", marker="s", s=80)

    half_fov = camera_config["fov_horizontal_deg"] / 2.0
    for _, row in stations_df.iterrows():
        station_xyz = (row.x, row.y, row.z)
        boresight_az, _ = _station_boresight(station_xyz, TRAJECTORY)
        # Wedge do matplotlib usa convenção matemática (0°=leste, sentido
        # anti-horário) - convertemos do azimute (0°=norte/y, horário).
        theta_center = 90.0 - boresight_az
        wedge = Wedge((row.x, row.y), fov_range_m,
                      theta_center - half_fov, theta_center + half_fov,
                      color="tab:orange", alpha=0.15, zorder=1)
        ax.add_patch(wedge)

    ax.scatter(stations_df.x, stations_df.y, color="black", marker="X",
               s=150, zorder=6, label="estações")
    for _, row in stations_df.iterrows():
        ax.annotate(f"E{int(row.station_id)}", (row.x, row.y),
                    textcoords="offset points", xytext=(8, 8), fontsize=10, zorder=6)

    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("y - North (m)")
    ax.set_title(f"Cenário {scenario_id} - trajetória + FOV das câmeras ({camera_config['fov_horizontal_deg']:.0f}°)")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/scenario_{scenario_id}_fov.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    traj_df = generate_trajectory(TRAJECTORY)

    for sc in (1, 2, 3):
        stations_df = generate_stations(sc, TRAJECTORY, STATIONS)
        plot_scenario_with_fov(sc, traj_df, stations_df, CAMERA)
        print(f"Gerado: scenario_{sc}_fov.png")

        pos_est = generate_camera_position_estimates(sc, TRAJECTORY, stations_df, CAMERA)
        csv_path = f"{OUT_DIR}/camera_position_estimates_scenario_{sc}.csv"
        pos_est.to_csv(csv_path, index=False)
        print(f"Gerado: {csv_path} ({len(pos_est)} linhas)")
        print(f"  Erro de posição (3D) - média: {pos_est.position_error_m.mean():.2f} m | "
              f"mediana: {pos_est.position_error_m.median():.2f} m | "
              f"máx: {pos_est.position_error_m.max():.2f} m")

    # Gráfico-resumo: posição estimada vs. ground truth, cenário 1
    stations_1 = generate_stations(1, TRAJECTORY, STATIONS)
    pos_est_1 = generate_camera_position_estimates(1, TRAJECTORY, stations_1, CAMERA)

    fig, ax = plt.subplots(figsize=(8, 7))
    ax.plot(traj_df.x, traj_df.y, color="black", lw=2, label="ground truth", zorder=5)
    colors = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}
    for sid, d in pos_est_1.groupby("station_id"):
        ax.scatter(d.est_x, d.est_y, s=8, alpha=0.4, color=colors[sid],
                   label=f"estimativa E{sid} (só câmera)")
    ax.scatter(stations_1.x, stations_1.y, color="black", marker="X", s=150, zorder=6)
    for _, row in stations_1.iterrows():
        ax.annotate(f"E{int(row.station_id)}", (row.x, row.y),
                    textcoords="offset points", xytext=(8, 8), fontsize=10)
    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("y - North (m)")
    ax.set_title("Posição estimada por câmera (por estação) vs. ground truth - Cenário 1")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/camera_position_estimates_summary.png", dpi=150)
    plt.close(fig)
    print("\nGerado: camera_position_estimates_summary.png")

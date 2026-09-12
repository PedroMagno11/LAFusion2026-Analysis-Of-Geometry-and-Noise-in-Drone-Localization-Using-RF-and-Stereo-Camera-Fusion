"""
Validação do Módulo 6 (GDOPCalculator).

Gera:
  - gdop_along_trajectory.png: GDOP teórico ao longo do tempo, para os 3
    cenários sobrepostos - confirma visualmente as hipóteses de
    degradação/uniformidade discutidas na metodologia (Item 4).
  - gdop_heatmap_scenario_{1,2,3}.png: mapa de calor de GDOP em toda a
    área de cobertura, com a trajetória e as estações sobrepostas -
    validação clássica ("GDOP mínimo no centro do polígono regular").
"""

import numpy as np
import matplotlib.pyplot as plt

from config import TRAJECTORY, STATIONS
from station_layout_generator import generate_stations
from trajectory_generator import generate_trajectory
from gdop_calculator import compute_gdop_along_trajectory, compute_gdop_grid

OUT_DIR = "./outputs"


if __name__ == "__main__":
    traj_df = generate_trajectory(TRAJECTORY)

    # --- GDOP ao longo do tempo, 3 cenários sobrepostos ---
    fig, ax = plt.subplots(figsize=(10, 6))
    colors = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}
    labels = {1: "Cenário 1 (linear)", 2: "Cenário 2 (circular)", 3: "Cenário 3 (assimétrico)"}

    for sc in (1, 2, 3):
        stations = generate_stations(sc, TRAJECTORY, STATIONS)
        gdop_df = compute_gdop_along_trajectory(sc, TRAJECTORY, stations)
        ax.plot(gdop_df.t, gdop_df.gdop, color=colors[sc], label=labels[sc], lw=2)

    ax.set_xlabel("t (s)")
    ax.set_ylabel("Índice de diluição geométrica bearing-only (m, sob variância angular unitária)")
    ax.set_title("GDOP teórico ao longo da trajetória, por cenário")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/gdop_along_trajectory.png", dpi=150)
    plt.close(fig)
    print("Gerado: gdop_along_trajectory.png")

    # --- Mapas de calor espaciais, por cenário ---
    x_range = (TRAJECTORY["x_start"] - 30, TRAJECTORY["x_end"] + 30)
    y_range = (-20, 100)

    for sc in (1, 2, 3):
        stations = generate_stations(sc, TRAJECTORY, STATIONS)
        grid = compute_gdop_grid(stations, x_range, y_range, n_points=120)

        fig, ax = plt.subplots(figsize=(8, 6))
        gdop_clipped = np.clip(grid["GDOP"], 0, np.nanpercentile(grid["GDOP"], 95))
        im = ax.pcolormesh(grid["X"], grid["Y"], gdop_clipped, shading="auto", cmap="viridis_r")
        fig.colorbar(im, ax=ax, label="GDOP (recortado no percentil 95)")

        ax.plot(traj_df.x, traj_df.y, color="red", lw=2, label="trajetória")
        ax.scatter(stations.x, stations.y, color="white", edgecolor="black",
                   marker="X", s=150, zorder=6, label="estações")
        for _, row in stations.iterrows():
            ax.annotate(f"E{int(row.station_id)}", (row.x, row.y),
                        textcoords="offset points", xytext=(8, 8),
                        fontsize=10, color="white")

        ax.set_xlabel("x - East (m)")
        ax.set_ylabel("y - North (m)")
        ax.set_title(f"Mapa de GDOP - Cenário {sc}")
        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()
        fig.savefig(f"{OUT_DIR}/gdop_heatmap_scenario_{sc}.png", dpi=150)
        plt.close(fig)
        print(f"Gerado: gdop_heatmap_scenario_{sc}.png")

"""
Validação do Módulo 7 (FusionEngine/EKF).

Gera:
  - ekf_trajectory_comparison.png: trajetória estimada (fusão) vs. ground
    truth, vista superior e perfil de altitude.
  - ekf_error_over_time.png: erro de posição 3D ao longo do tempo, e
    comparação de RMSE entre fusão, RF-only e câmera-only.
  - ekf_nis_diagnostic.png: histograma de NIS por tipo de sensor, com as
    referências teóricas (média esperada = graus de liberdade), para
    diagnosticar se o ruído de processo (q) está bem calibrado.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import chi2

from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG
from station_layout_generator import generate_stations
from trajectory_generator import generate_trajectory, trajectory_state_at
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from ekf_fusion import run_fusion_ekf

OUT_DIR = "./outputs"


def add_ground_truth_error(result_df, traj_config):
    true_state = trajectory_state_at(result_df.timestamp.values, traj_config)
    result_df = result_df.copy()
    result_df["true_x"] = true_state["x"]
    result_df["true_y"] = true_state["y"]
    result_df["true_z"] = true_state["z"]
    result_df["pos_error_m"] = np.sqrt(
        (result_df.x - result_df.true_x) ** 2
        + (result_df.y - result_df.true_y) ** 2
        + (result_df.z - result_df.true_z) ** 2
    )
    return result_df


if __name__ == "__main__":
    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)
    traj_df = generate_trajectory(TRAJECTORY)

    print("Rodando fusão (RF+câmera)...")
    fusion = add_ground_truth_error(run_fusion_ekf(merged, stations, FILTER_CONFIG, "fusion"), TRAJECTORY)
    print("Rodando RF-only...")
    rf_only = add_ground_truth_error(run_fusion_ekf(merged, stations, FILTER_CONFIG, "rf_only"), TRAJECTORY)
    print("Rodando câmera-only...")
    cam_only = add_ground_truth_error(run_fusion_ekf(merged, stations, FILTER_CONFIG, "camera_only"), TRAJECTORY)

    # --- Trajetória estimada vs. ground truth ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax = axes[0]
    ax.plot(traj_df.x, traj_df.y, color="black", lw=2, label="ground truth", zorder=5)
    ax.plot(fusion.x, fusion.y, color="tab:blue", alpha=0.7, label="fusão (RF+câmera)")
    ax.scatter(stations.x, stations.y, color="red", marker="X", s=100, zorder=6, label="estações")
    ax.set_xlabel("x - East (m)")
    ax.set_ylabel("y - North (m)")
    ax.set_title("Trajetória estimada (fusão) vs. ground truth - vista superior")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_aspect("equal", adjustable="datalim")

    ax = axes[1]
    ax.plot(traj_df.t, traj_df.z, color="black", lw=2, label="ground truth")
    ax.plot(fusion.timestamp, fusion.z, color="tab:blue", alpha=0.7, label="fusão")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("z - altitude (m)")
    ax.set_title("Perfil de altitude - fusão vs. ground truth")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/ekf_trajectory_comparison.png", dpi=150)
    plt.close(fig)
    print("Gerado: ekf_trajectory_comparison.png")

    # --- Erro ao longo do tempo + comparação de RMSE ---
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    ax = axes[0]
    ax.plot(fusion.timestamp, fusion.pos_error_m, label="fusão", color="tab:blue")
    ax.plot(rf_only.timestamp, rf_only.pos_error_m, label="RF-only", color="tab:red", alpha=0.6)
    ax.plot(cam_only.timestamp, cam_only.pos_error_m, label="câmera-only", color="tab:orange", alpha=0.6)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Erro de posição 3D (m)")
    ax.set_title("Erro de posição ao longo do tempo, por modo")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    modes = ["fusão", "RF-only", "câmera-only"]
    rmses = [
        np.sqrt((fusion.pos_error_m**2).mean()),
        np.sqrt((rf_only.pos_error_m**2).mean()),
        np.sqrt((cam_only.pos_error_m**2).mean()),
    ]
    bars = ax.bar(modes, rmses, color=["tab:blue", "tab:red", "tab:orange"])
    for bar, val in zip(bars, rmses):
        ax.text(bar.get_x() + bar.get_width()/2, val, f"{val:.1f}m",
                ha="center", va="bottom", fontsize=10)
    ax.set_ylabel("RMSE de posição 3D (m)")
    ax.set_title("RMSE global, por modo de sensor")
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/ekf_error_over_time.png", dpi=150)
    plt.close(fig)
    print("Gerado: ekf_error_over_time.png")
    print(f"\nRMSE - Fusão: {rmses[0]:.2f}m | RF-only: {rmses[1]:.2f}m | Câmera-only: {rmses[2]:.2f}m")

    # --- Diagnóstico de NIS ---
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for ax, stype, dof in zip(axes, ("RF", "camera"), (1, 3)):
        nis_vals = fusion[fusion.sensor_type == stype].nis.dropna()
        nis_vals = nis_vals[np.isfinite(nis_vals)]
        # Recorta o eixo X no percentil 99.5 para não deixar o transiente
        # de inicialização (outlier isolado) ilegível o gráfico.
        x_clip = nis_vals.quantile(0.995)
        nis_plot = nis_vals[nis_vals <= x_clip]
        nis_mean_no_burnin = nis_vals[nis_vals <= nis_vals.quantile(0.999)].mean()

        ax.hist(nis_plot, bins=60, density=True, alpha=0.6, color="tab:blue",
                label=f"NIS observado (n={len(nis_plot)}, recortado no p99.5)")
        xs = np.linspace(0.001, x_clip, 200)
        ax.plot(xs, chi2.pdf(xs, df=dof), color="black", lw=2,
                label=f"χ²({dof}) teórico")
        ax.axvline(dof, color="red", ls="--", label=f"média esperada = {dof}")
        ax.axvline(nis_mean_no_burnin, color="green", ls="--",
                   label=f"média observada (sem transiente) = {nis_mean_no_burnin:.2f}")
        ax.set_title(f"NIS - detecções {stype}")
        ax.set_xlabel("NIS")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/ekf_nis_diagnostic.png", dpi=150)
    plt.close(fig)
    print("Gerado: ekf_nis_diagnostic.png")

    for stype, dof in (("RF", 1), ("camera", 3)):
        nis_all = fusion[fusion.sensor_type == stype].nis.dropna()
        nis_all = nis_all[np.isfinite(nis_all)]
        # Exclui o transiente de inicialização (maior valor de NIS da
        # câmera, sempre no/perto do instante de inicialização) - efeito
        # esperado e documentado, não um problema de calibração do filtro.
        threshold = nis_all.quantile(0.999)
        nis_burnin_excluded = nis_all[nis_all <= threshold]
        pct_above_95 = (nis_burnin_excluded > chi2.ppf(0.95, dof)).mean() * 100
        print(f"{stype}: NIS médio (com transiente)={nis_all.mean():.2f} | "
              f"NIS médio (sem transiente, excluindo top 0,1%)={nis_burnin_excluded.mean():.2f} "
              f"(esperado~{dof}) | % acima do percentil 95 teórico: {pct_above_95:.1f}% (esperado ~5%)")
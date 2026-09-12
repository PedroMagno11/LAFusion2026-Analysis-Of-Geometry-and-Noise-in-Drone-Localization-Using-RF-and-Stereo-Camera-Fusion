"""
Validação do Módulo 8 (BaselineEstimator).

Gera:
  - baseline_vs_ekf_comparison.png: 2 painéis -
      (a) erro de posição 3D ao longo do tempo, para os 4 modos (fusão
          EKF, baseline geométrico, RF-only, câmera-only);
      (b) RMSE global comparado entre os 4 modos, isolando visualmente o
          ganho da combinação multi-sensor (baseline vs. sensores
          isolados) do ganho adicional da filtragem temporal (EKF vs.
          baseline).
"""

import numpy as np
import matplotlib.pyplot as plt

from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG
from station_layout_generator import generate_stations
from trajectory_generator import trajectory_state_at
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from ekf_fusion import run_fusion_ekf
from baseline_estimator import run_baseline

OUT_DIR = "./outputs"


def pos_error(df):
    true_state = trajectory_state_at(df.timestamp.values, TRAJECTORY)
    return np.sqrt(
        (df.x - true_state["x"])**2 + (df.y - true_state["y"])**2
        + (df.z - true_state["z"])**2
    )


if __name__ == "__main__":
    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)

    print("Rodando EKF fusão...")
    ekf_fusion_result = run_fusion_ekf(merged, stations, FILTER_CONFIG, "fusion")
    ekf_fusion_result["pos_error_m"] = pos_error(ekf_fusion_result)

    print("Rodando EKF RF-only...")
    ukf_rf = run_fusion_ekf(merged, stations, FILTER_CONFIG, "rf_only")
    ukf_rf["pos_error_m"] = pos_error(ukf_rf)

    print("Rodando EKF câmera-only...")
    ukf_cam = run_fusion_ekf(merged, stations, FILTER_CONFIG, "camera_only")
    ukf_cam["pos_error_m"] = pos_error(ukf_cam)

    print("Rodando baseline geométrico...")
    baseline = run_baseline(merged, stations)
    baseline["pos_error_m"] = pos_error(baseline)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    ax = axes[0]
    ax.plot(ekf_fusion_result.timestamp, ekf_fusion_result.pos_error_m, label="EKF fusão", color="tab:blue", lw=1.5)
    ax.plot(baseline.timestamp, baseline.pos_error_m, label="Baseline geométrico", color="tab:green", lw=1.2)
    ax.plot(ukf_rf.timestamp, ukf_rf.pos_error_m, label="EKF RF-only", color="tab:red", alpha=0.5)
    ax.plot(ukf_cam.timestamp, ukf_cam.pos_error_m, label="EKF câmera-only", color="tab:orange", alpha=0.5)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Erro de posição 3D (m)")
    ax.set_title("Erro de posição ao longo do tempo - todos os modos")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    ax = axes[1]
    modes = ["EKF\nfusão", "Baseline\ngeométrico", "EKF\nRF-only", "EKF\ncâmera-only"]
    rmses = [
        np.sqrt((ekf_fusion_result.pos_error_m**2).mean()),
        np.sqrt((baseline.pos_error_m**2).mean()),
        np.sqrt((ukf_rf.pos_error_m**2).mean()),
        np.sqrt((ukf_cam.pos_error_m**2).mean()),
    ]
    colors = ["tab:blue", "tab:green", "tab:red", "tab:orange"]
    bars = ax.bar(modes, rmses, color=colors)
    for bar, val in zip(bars, rmses):
        ax.text(bar.get_x() + bar.get_width()/2, val, f"{val:.2f}m",
                ha="center", va="bottom", fontsize=10)
    ax.set_ylabel("RMSE de posição 3D (m)")
    ax.set_title("RMSE global - ganho de multi-sensor vs. ganho de filtragem temporal")
    ax.grid(alpha=0.3, axis="y")

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/baseline_vs_ekf_comparison.png", dpi=150)
    plt.close(fig)

    print(f"\nRMSE - EKF fusão: {rmses[0]:.2f}m | Baseline geométrico: {rmses[1]:.2f}m | "
          f"EKF RF-only: {rmses[2]:.2f}m | EKF câmera-only: {rmses[3]:.2f}m")
    print(f"\nGanho da combinação multi-sensor (sensor isolado -> baseline): "
          f"{min(rmses[2],rmses[3]) - rmses[1]:.2f}m de redução")
    print(f"Ganho adicional da filtragem temporal (baseline -> EKF fusão): "
          f"{rmses[1] - rmses[0]:.2f}m de redução")
    print("\nGerado: baseline_vs_ekf_comparison.png")

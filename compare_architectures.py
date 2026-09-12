"""
Compara as duas arquiteturas de fusão do projeto, nas MESMAS detecções
(mesmas sementes de ruído) - substitui compare_ekf_ukf.py (removido
junto com o FILTER_CONFIG, ver achados_divergencia_fusao.md §8):

  - "fusion" (EKF global): um único EKF processando as detecções de
    todas as estações em sequência (ekf_fusion.py).
  - "track_to_track_ci": um EKF local por estação + combinador global
    por Interseção de Covariância (track_to_track_fusion.py).

Inclui de propósito os casos já mapeados nas 30 repetições reais
(experiment_results/raw_run_summaries.csv) como divergentes no EKF
global, e alguns controles convergentes, para caracterizar o
comportamento das duas arquiteturas lado a lado.

Uso: python3 compare_architectures.py
"""
import numpy as np
import pandas as pd

from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG, EXPERIMENT
from station_layout_generator import generate_stations
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from orchestrator import thin_rf_detections
from ekf_fusion import run_fusion_ekf
from track_to_track_fusion import run_track_to_track_fusion
from metrics_evaluator import estimate_convergence_time, compute_rmse_common_grid

CASES = [
    (1, 0, "controle"),
    (2, 0, "controle"),
    (2, 1, "controle"),
    (3, 0, "controle"),
    (2, 7, "divergia no EKF global (init antiga)"),
    (2, 27, "divergia no EKF global (init antiga)"),
    (2, 3, "divergia no EKF global (init antiga)"),
    (2, 12, "divergia no EKF global (init antiga)"),
    (3, 19, "divergia no EKF global (init antiga) - caso mais dificil"),
    (3, 20, "divergia no EKF global (init antiga)"),
    (3, 23, "divergia no EKF global (init antiga)"),
    (3, 27, "divergia no EKF global (init antiga)"),
    (3, 29, "divergia no EKF global (init antiga)"),
]


def build_detections(scenario_id, repetition):
    stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_cfg = dict(RF); rf_cfg["seed"] = RF["seed"] + repetition * 1000 + scenario_id
    cam_cfg = dict(CAMERA); cam_cfg["seed"] = CAMERA["seed"] + repetition * 1000 + scenario_id
    rf_det = thin_rf_detections(generate_rf_detections(scenario_id, TRAJECTORY, stations_df, rf_cfg),
                                 EXPERIMENT["rf_decimation_factor"])
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations_df, cam_cfg)
    merged = associate_detections(rf_det, cam_det)
    return stations_df, merged


def summarize(track, label):
    t_conv = estimate_convergence_time(track) if "pos_uncertainty" in track.columns else None
    g = compute_rmse_common_grid(track, TRAJECTORY, exclude_before_s=t_conv)
    full = compute_rmse_common_grid(track, TRAJECTORY)
    return {
        "arquitetura": label, "convergiu": t_conv is not None,
        "rmse_full_m": full["rmse_m"], "rmse_steady_m": g["rmse_m"],
    }


print(f"{'cenario':>7s} {'rep':>4s} {'arquitetura':>18s} {'convergiu':>10s} {'rmse_full':>10s} {'rmse_steady':>12s}  nota")
print("-" * 100)
rows = []
for scenario_id, repetition, nota in CASES:
    stations_df, merged = build_detections(scenario_id, repetition)

    track_global = run_fusion_ekf(merged, stations_df, FILTER_CONFIG, sensor_mode="fusion")
    track_t2t, _local = run_track_to_track_fusion(
        scenario_id, repetition, stations_df=stations_df, merged=merged)

    for track, label in [(track_global, "EKF global"), (track_t2t, "track_to_track_ci")]:
        s = summarize(track, label)
        s["scenario_id"] = scenario_id
        s["repetition"] = repetition
        s["nota"] = nota
        rows.append(s)
        print(f"{scenario_id:7d} {repetition:4d} {label:>18s} {str(s['convergiu']):>10s} "
              f"{s['rmse_full_m']:10.3f} {s['rmse_steady_m']:12.3f}  {nota}")

pd.DataFrame(rows).to_csv("ekf_global_vs_track_to_track_comparison.csv", index=False)
print("\nSalvo em ekf_global_vs_track_to_track_comparison.csv")

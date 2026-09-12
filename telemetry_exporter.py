"""
Módulo 9 - TelemetryExporter

Formata a saída de qualquer método de tracking no esquema de saída padronizado, 
anexa o ground truth correspondente, e grava em arquivo (CSV e/ou JSON).

Esquema de saída (Item 8b):
    timestamp, x, y, z, vx, vy, vz, speed, course_over_ground,
    vertical_speed, pos_uncertainty, vel_uncertainty, scenario_id,
    sensor_mode, ground_truth_x/y/z, ground_truth_vx/vy/vz

NOTA: o baseline geométrico não possui estado de velocidade - os campos vx/vy/vz, speed, course_over_ground, 
vertical_speed, pos_uncertainty e vel_uncertainty ficam como NaN para esse modo, e isso é documentado 
explicitamente na saída.
"""

import numpy as np
import pandas as pd

from trajectory_generator import trajectory_state_at

_SCHEMA_COLUMNS = [
    "timestamp", "x", "y", "z", "vx", "vy", "vz",
    "speed", "course_over_ground", "vertical_speed",
    "pos_uncertainty", "vel_uncertainty",
    "scenario_id", "sensor_mode",
    "ground_truth_x", "ground_truth_y", "ground_truth_z",
    "ground_truth_vx", "ground_truth_vy", "ground_truth_vz",
]


def attach_ground_truth(track_df: pd.DataFrame, traj_config: dict) -> pd.DataFrame:
    """Anexa a posição/velocidade verdadeira (ground truth) a cada timestamp da trilha."""
    true_state = trajectory_state_at(track_df.timestamp.values, traj_config)
    out = track_df.copy()
    out["ground_truth_x"] = true_state["x"]
    out["ground_truth_y"] = true_state["y"]
    out["ground_truth_z"] = true_state["z"]
    out["ground_truth_vx"] = true_state["vx"]
    out["ground_truth_vy"] = true_state["vy"]
    out["ground_truth_vz"] = true_state["vz"]
    return out


def to_standard_schema(track_df: pd.DataFrame, traj_config: dict) -> pd.DataFrame:
    """
    Converte a saída de run_fusion() ou run_baseline() para o esquema
    padronizado do Item 8b, preenchendo com NaN os campos não aplicáveis
    (ex.: velocidade para o baseline geométrico).
    """
    df = attach_ground_truth(track_df, traj_config)
    for col in _SCHEMA_COLUMNS:
        if col not in df.columns:
            df[col] = np.nan
    return df[_SCHEMA_COLUMNS]


def export_telemetry(track_df: pd.DataFrame, traj_config: dict, output_path: str,
                      fmt: str = "csv") -> pd.DataFrame:
    """
    Formata e grava a telemetria em arquivo.

    Parâmetros
    ----------
    track_df : saída de run_fusion() ou run_baseline()
    traj_config : bloco TRAJECTORY do config.py
    output_path : caminho do arquivo de saída (sem extensão, ou com -
        a extensão é normalizada conforme `fmt`)
    fmt : "csv" ou "json"

    Retorna
    -------
    O DataFrame já no esquema padronizado (também gravado em disco).
    """
    standardized = to_standard_schema(track_df, traj_config)

    base = output_path.rsplit(".", 1)[0] if output_path.endswith((".csv", ".json")) else output_path
    if fmt == "csv":
        standardized.to_csv(f"{base}.csv", index=False)
    elif fmt == "json":
        standardized.to_json(f"{base}.json", orient="records", indent=2)
    else:
        raise ValueError(f"Formato não suportado: {fmt} (use 'csv' ou 'json')")

    return standardized


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG
    from station_layout_generator import generate_stations
    from rf_sensor_model import generate_rf_detections
    from camera_sensor_model import generate_camera_detections
    from detection_association import associate_detections
    from ekf_fusion import run_fusion_ekf
    from baseline_estimator import run_baseline

    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)

    fusion_result = run_fusion_ekf(merged, stations, FILTER_CONFIG, "fusion")
    fusion_std = export_telemetry(
        fusion_result, TRAJECTORY,
        "/mnt/user-data/outputs/telemetry_scenario1_fusion", fmt="csv",
    )
    print("Esquema exportado (fusão):")
    print(fusion_std.head(3).to_string())
    print(f"\nColunas: {list(fusion_std.columns)}")

    baseline_result = run_baseline(merged, stations)
    baseline_std = export_telemetry(
        baseline_result, TRAJECTORY,
        "./outputs/telemetry_scenario1_baseline", fmt="csv",
    )
    print("\nEsquema exportado (baseline) - campos de velocidade devem ser NaN:")
    print(baseline_std[["timestamp", "x", "y", "z", "vx", "speed", "sensor_mode"]].head(3).to_string())

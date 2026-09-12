"""
Extensão do Módulo 4/9 - Exportador de Tracks Locais por Estação

Gera uma tabela por estação e instante de detecção, com:
  - station_id: identificador da estação
  - local_track_id: identificador do "segmento" de detecção contínua
    daquela estação (incrementa toda vez que a estação perde e recupera
    o alvo - ex.: sai e volta ao FOV da câmera)
  - matriz de covariância de posição (3x3), obtida por propagação
    linearizada de erro (jacobiano) a partir das incertezas de medição
    (azimute, elevação, distância) já calibradas nos Módulos 3 e 4.

"""

import numpy as np
import pandas as pd

from camera_sensor_model import estimate_position_from_bearing


def camera_position_covariance(azimuth_deg: float, elevation_deg: float, distance_m: float,
                                az_std_deg: float, el_std_deg: float, dist_std_m: float) -> np.ndarray:
    """
    Matriz de covariância de posição 3D (x,y,z), por propagação linearizada
    de erro (jacobiano) a partir da covariância diagonal da medição
    esférica (azimute, elevação, distância) - técnica padrão de primeira
    ordem (linearização local), consistente com a mesma aproximação usada
    internamente pelo FILTER_CONFIG para medições não lineares.

    Retorna
    -------
    np.ndarray 3x3 - matriz de covariância em (x, y, z), unidades m².
    """
    az = np.radians(azimuth_deg)
    el = np.radians(elevation_deg)
    d = distance_m

    # Jacobiano d(x,y,z)/d(azimute, elevação, distância) - radianos, radianos, metros
    J = np.array([
        [d * np.cos(el) * np.cos(az), -d * np.sin(el) * np.sin(az), np.cos(el) * np.sin(az)],
        [-d * np.cos(el) * np.sin(az), -d * np.sin(el) * np.cos(az), np.cos(el) * np.cos(az)],
        [0.0, d * np.cos(el), np.sin(el)],
    ])

    R = np.diag([
        np.radians(az_std_deg) ** 2,
        np.radians(el_std_deg) ** 2,
        dist_std_m ** 2,
    ])

    return J @ R @ J.T


def _assign_local_track_ids(timestamps: np.ndarray, expected_period_s: float,
                             gap_factor: float = 2.0) -> np.ndarray:
    """
    Atribui um local_track_id sequencial, incrementando toda vez que o
    intervalo entre detecções consecutivas da mesma estação excede
    `gap_factor` vezes o período esperado (indicando perda e reaquisição
    do alvo, ex.: saída e retorno ao FOV da câmera).
    """
    if len(timestamps) == 0:
        return np.array([], dtype=int)
    gaps = np.diff(timestamps, prepend=timestamps[0])
    new_segment = gaps > (expected_period_s * gap_factor)
    return np.cumsum(new_segment) + 1


def export_local_tracks_camera(scenario_id: int, traj_config: dict, stations_df: pd.DataFrame,
                                camera_config: dict) -> pd.DataFrame:
    """
    Gera a tabela de tracks locais por estação de CÂMERA, com covariância
    de posição 3D completa.

    Retorna
    -------
    pd.DataFrame: timestamp, station_id, local_track_id, est_x, est_y, est_z,
        cov_xx, cov_xy, cov_xz, cov_yy, cov_yz, cov_zz, scenario_id
    """
    from camera_sensor_model import generate_camera_detections

    det = generate_camera_detections(scenario_id, traj_config, stations_df, camera_config)
    station_lookup = stations_df.set_index("station_id")[["x", "y", "z"]]
    expected_period = 1.0 / camera_config["frame_rate_hz"]

    rows = []
    for sid, d in det.groupby("station_id"):
        d = d.sort_values("timestamp").reset_index(drop=True)
        track_ids = _assign_local_track_ids(d.timestamp.values, expected_period)

        station_xyz = tuple(station_lookup.loc[sid])
        for i, row in d.iterrows():
            est_x, est_y, est_z = estimate_position_from_bearing(
                station_xyz, row.meas_azimuth_deg, row.meas_elevation_deg, row.meas_distance_m,
            )
            cov = camera_position_covariance(
                row.meas_azimuth_deg, row.meas_elevation_deg, row.meas_distance_m,
                row.bearing_error_std_deg, row.bearing_error_std_deg, row.depth_error_std_m,
            )
            rows.append({
                "timestamp": row.timestamp, "station_id": int(sid),
                "local_track_id": int(track_ids[i]),
                "est_x": est_x, "est_y": est_y, "est_z": est_z,
                "cov_xx": cov[0, 0], "cov_xy": cov[0, 1], "cov_xz": cov[0, 2],
                "cov_yy": cov[1, 1], "cov_yz": cov[1, 2], "cov_zz": cov[2, 2],
                "scenario_id": scenario_id,
            })

    return pd.DataFrame(rows).sort_values(["station_id", "timestamp"]).reset_index(drop=True)


def export_local_tracks_rf(scenario_id: int, traj_config: dict, stations_df: pd.DataFrame,
                            rf_config: dict) -> pd.DataFrame:
    """
    Gera a tabela de tracks locais por estação RF. NÃO produz covariância
    de posição 3D (matematicamente indeterminada a partir de uma única
    estação bearing-only) - reporta apenas a variância do azimute medido.
    """
    from rf_sensor_model import generate_rf_detections

    det = generate_rf_detections(scenario_id, traj_config, stations_df, rf_config)
    expected_period = 1.0 / rf_config["update_rate_hz"]

    rows = []
    for sid, d in det.groupby("station_id"):
        d = d.sort_values("timestamp").reset_index(drop=True)
        track_ids = _assign_local_track_ids(d.timestamp.values, expected_period)
        for i, row in d.iterrows():
            rows.append({
                "timestamp": row.timestamp, "station_id": int(sid),
                "local_track_id": int(track_ids[i]),
                "meas_azimuth_deg": row.meas_azimuth_deg,
                "var_azimuth_rad2": np.radians(row.error_std_deg) ** 2,
                "position_covariance_3d": "indeterminada (bearing-only, 1 estação)",
                "scenario_id": scenario_id,
            })

    return pd.DataFrame(rows).sort_values(["station_id", "timestamp"]).reset_index(drop=True)


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF, CAMERA
    from station_layout_generator import generate_stations

    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)

    cam_tracks = export_local_tracks_camera(scenario_id, TRAJECTORY, stations, CAMERA)
    print(f"Tracks locais de câmera: {len(cam_tracks)} linhas")
    print(cam_tracks[["timestamp", "station_id", "local_track_id", "est_x", "est_y", "est_z",
                       "cov_xx", "cov_yy", "cov_zz"]].head(10).to_string())
    print(f"\nNº de segmentos de track por estação:")
    print(cam_tracks.groupby("station_id").local_track_id.nunique())

    rf_tracks = export_local_tracks_rf(scenario_id, TRAJECTORY, stations, RF)
    print(f"\nTracks locais de RF: {len(rf_tracks)} linhas (sem covariância 3D - ver nota no módulo)")
    print(rf_tracks[["timestamp", "station_id", "local_track_id", "meas_azimuth_deg"]].head(5).to_string())

    cam_tracks.to_csv("./outputs/local_tracks_camera_scenario1.csv", index=False)
    rf_tracks.to_csv("./outputs/local_tracks_rf_scenario1.csv", index=False)
    print("\nSalvo: local_tracks_camera_scenario1.csv, local_tracks_rf_scenario1.csv")

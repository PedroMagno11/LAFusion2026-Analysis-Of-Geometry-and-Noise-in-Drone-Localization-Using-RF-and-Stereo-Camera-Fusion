"""
Módulo 4 - CameraSensorModel

Simula as detecções por câmera de cada estação ao longo da trajetória do
drone.

Cadeia de cálculo, por estação e instante de amostragem:
  1. Geometria verdadeira: azimute, elevação e distância 3D estação->drone
     (mesma convenção usada no rf_sensor_model.py).
  2. Verificação de campo de visão (FOV): a câmera de cada estação aponta
     para um "boresight" fixo (direção do centro geométrico da trajetória);
     se o ângulo entre o boresight e a direção verdadeira do drone exceder
     metade do FOV horizontal, não há detecção nesse instante (limite
     físico/geométrico da câmera - distinto da "detecção sempre
     bem-sucedida", que se refere a alvos dentro do alcance/FOV).
  3. Erro de bearing (graus): derivado do ruído de detecção em pixels
     (Δd) convertido para ângulo via a distância focal (em pixels),
     calculada a partir da resolução e do FOV configurados.
  4. Erro de profundidade (m): ΔZ = (z²/(B·f))·Δd, usando a baseline REAL
     da estação (station_baseline_m), calculada em metros e depois
     convertida para incerteza relativa na distância 3D.
  5. Amostragem da medição ruidosa: azimute/elevação/distância verdadeiros
     + ruído gaussiano com os desvios-padrão calculados nos passos 3-4.

CALIBRAÇÃO DO RUÍDO DE DETECÇÃO (Δd): retro-calculada a partir do
resultado empírico de Sharma, Jain & Kothari (2022) - erro de profundidade
de 23% a 8 m - usando a fórmula ΔZ=(z²/(B·f))·Δd com f e B de uma
configuração de referência de câmera estéreo em AirSim [VIODE dataset].
Ver config.py para detalhes e ressalvas.
"""

import numpy as np
import pandas as pd

from trajectory_generator import trajectory_state_at

# ---------------------------------------------------------------------------
# Calibração do ruído de detecção (Δd), a partir de Sharma, Jain & Kothari
# (2022): erro de profundidade médio de 23% a uma distância de 8 m.
# ---------------------------------------------------------------------------
_CALIB_DISTANCE_M = 8.0
_CALIB_ERROR_FRACTION = 0.23


def focal_length_px(camera_config: dict) -> float:
    """Distância focal em pixels, a partir da resolução e do FOV horizontal."""
    width_px = camera_config["resolution_px"][0]
    fov_rad = np.radians(camera_config["fov_horizontal_deg"])
    return (width_px / 2.0) / np.tan(fov_rad / 2.0)


def pixel_disparity_noise_px(camera_config: dict) -> float:
    """
    Δd (erro de disparidade/centróide, em pixels), retro-calculado da
    calibração empírica de Sharma, Jain & Kothari (2022).
    """
    f = focal_length_px(camera_config)
    B_calib = camera_config["calibration_baseline_m"]
    delta_z = _CALIB_ERROR_FRACTION * _CALIB_DISTANCE_M
    return delta_z * B_calib * f / (_CALIB_DISTANCE_M ** 2)


def bearing_error_std_deg(camera_config: dict) -> float:
    """Desvio-padrão do erro de bearing (azimute/elevação), em graus."""
    f = focal_length_px(camera_config)
    dd = pixel_disparity_noise_px(camera_config)
    return np.degrees(dd / f)


def depth_error_std_m(distance_m: np.ndarray, camera_config: dict) -> np.ndarray:
    """Desvio-padrão do erro de profundidade (m), via ΔZ=(z²/(B·f))·Δd,
    usando a baseline REAL da estação (não a baseline de calibração)."""
    f = focal_length_px(camera_config)
    B = camera_config["station_baseline_m"]
    dd = pixel_disparity_noise_px(camera_config)
    return (distance_m ** 2 / (B * f)) * dd


def _true_bearing(station_xyz, drone_xyz):
    """Mesma convenção do rf_sensor_model.py: azimute (0-360, a partir do
    Norte/y, sentido horário) e elevação (graus)."""
    dx = drone_xyz[0] - station_xyz[0]
    dy = drone_xyz[1] - station_xyz[1]
    dz = drone_xyz[2] - station_xyz[2]
    horiz_dist = np.sqrt(dx**2 + dy**2)
    azimuth = np.degrees(np.arctan2(dx, dy)) % 360.0
    elevation = np.degrees(np.arctan2(dz, horiz_dist))
    dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)
    return azimuth, elevation, dist_3d


def _station_boresight(station_xyz, traj_config: dict):
    """
    Direção fixa para a qual a câmera da estação aponta: o centro
    geométrico da trajetória (ponto médio em x, altura média do arco).
    Retorna (azimute_boresight_deg, elevação_boresight_deg).
    """
    x_mid = (traj_config["x_start"] + traj_config["x_end"]) / 2.0
    y_mid = traj_config["y_offset"]
    z_mid = traj_config["z_base"] + traj_config["z_amplitude"] / 2.0
    az, el, _ = _true_bearing(station_xyz, (x_mid, y_mid, z_mid))
    return az, el


def _angular_offset_deg(az1, el1, az2, el2):
    """Ângulo entre duas direções (az, el), em graus, via produto escalar
    de vetores unitários - válido para checagem de FOV."""
    def to_unit_vector(az_deg, el_deg):
        az = np.radians(az_deg)
        el = np.radians(el_deg)
        return np.array([
            np.cos(el) * np.sin(az),
            np.cos(el) * np.cos(az),
            np.sin(el),
        ])
    v1 = to_unit_vector(az1, el1)
    v2 = to_unit_vector(np.full_like(az1, az2), np.full_like(el1, el2))
    dot = np.clip(np.sum(v1 * v2, axis=0), -1.0, 1.0)
    return np.degrees(np.arccos(dot))


def generate_camera_detections(scenario_id: int, traj_config: dict,
                                stations_df: pd.DataFrame, camera_config: dict,
                                noise_reference_stations_df: pd.DataFrame = None) -> pd.DataFrame:
    """
    Gera as detecções por câmera (uma realização de ruído) para um cenário.
    Só há linha de saída para instantes em que o drone está dentro do FOV
    da estação (limite físico da câmera).

    noise_reference_stations_df : opcional - mesmo princípio de
        rf_sensor_model.py::generate_rf_detections. O ruído ANGULAR da
        câmera (bearing_std) já não depende de distância (só de FOV/
        resolução - ver bearing_error_std_deg), então só o ruído de
        PROFUNDIDADE (depth_std, que cresce com a distância²) é afetado
        por esta referência.

    Retorna
    -------
    pd.DataFrame com colunas:
        timestamp, station_id, sensor_type, distance_m, bearing_error_std_deg,
        depth_error_std_m, true_azimuth_deg, true_elevation_deg,
        meas_azimuth_deg, meas_elevation_deg, meas_distance_m, scenario_id
    """
    rng = np.random.default_rng(camera_config["seed"])
    f = focal_length_px(camera_config)
    noise_scale = camera_config.get("noise_scale", 1.0)
    bearing_std = bearing_error_std_deg(camera_config) * noise_scale
    # noise_scale (padrão 1.0) - mesmo princípio do rf_sensor_model.py: escala
    # o ruído (angular E de profundidade) independentemente da distância.
    half_fov = camera_config["fov_horizontal_deg"] / 2.0

    dt_cam = 1.0 / camera_config["frame_rate_hz"]
    t = np.arange(0.0, traj_config["duration_s"] + dt_cam / 2, dt_cam)
    drone_state = trajectory_state_at(t, traj_config)
    drone_xyz = (drone_state["x"], drone_state["y"], drone_state["z"])

    if noise_reference_stations_df is not None:
        noise_ref_lookup = noise_reference_stations_df.set_index("station_id")

    rows = []
    for _, station in stations_df.iterrows():
        station_xyz = (station.x, station.y, station.z)
        az, el, dist = _true_bearing(station_xyz, drone_xyz)

        boresight_az, boresight_el = _station_boresight(station_xyz, traj_config)
        offset = _angular_offset_deg(az, el, boresight_az, boresight_el)
        in_fov = offset <= half_fov

        if noise_reference_stations_df is not None:
            ref = noise_ref_lookup.loc[int(station.station_id)]
            ref_xyz = (ref.x, ref.y, ref.z)
            _, _, noise_dist = _true_bearing(ref_xyz, drone_xyz)
        else:
            noise_dist = dist

        depth_std = depth_error_std_m(noise_dist, camera_config) * noise_scale

        az_meas = (az + rng.normal(0.0, bearing_std, size=len(t))) % 360.0
        el_meas = el + rng.normal(0.0, bearing_std, size=len(t))
        dist_meas = dist + rng.normal(0.0, depth_std, size=len(t))

        df_station = pd.DataFrame({
            "timestamp": t,
            "station_id": int(station.station_id),
            "sensor_type": "camera",
            "distance_m": dist,
            "bearing_error_std_deg": bearing_std,
            "depth_error_std_m": depth_std,
            "true_azimuth_deg": az,
            "true_elevation_deg": el,
            "meas_azimuth_deg": az_meas,
            "meas_elevation_deg": el_meas,
            "meas_distance_m": dist_meas,
            "in_fov": in_fov,
            "scenario_id": scenario_id,
        })
        rows.append(df_station[df_station.in_fov].drop(columns="in_fov"))

    return pd.concat(rows, ignore_index=True)


def estimate_position_from_bearing(station_xyz, azimuth_deg, elevation_deg, distance_m):
    """
    Inverso de _true_bearing(): dado azimute, elevação e distância medidos
    por UMA câmera, estima a posição 3D absoluta do drone.

    Diferente do RF (que só fornece bearing e precisa de múltiplas estações
    para triangular), a câmera fornece bearing + profundidade e por isso
    consegue gerar uma estimativa de posição 3D completa a partir de uma
    ÚNICA estação - útil como saída independente para inspeção/validação,
    e como um dos "baselines por sensor" mencionados no Item 8 da
    metodologia (câmera-only).
    """
    az = np.radians(azimuth_deg)
    el = np.radians(elevation_deg)
    horiz = distance_m * np.cos(el)
    dx = horiz * np.sin(az)
    dy = horiz * np.cos(az)
    dz = distance_m * np.sin(el)
    return station_xyz[0] + dx, station_xyz[1] + dy, station_xyz[2] + dz


def generate_camera_position_estimates(scenario_id: int, traj_config: dict,
                                        stations_df: pd.DataFrame, camera_config: dict) -> pd.DataFrame:
    """
    Gera, para cada estação e instante em que o drone está no FOV, uma
    estimativa de posição 3D independente baseada apenas na câmera daquela
    estação (bearing medido + profundidade medida), junto com a posição
    verdadeira (ground truth) para permitir cálculo de erro.

    Retorna
    -------
    pd.DataFrame com colunas:
        timestamp, station_id, scenario_id,
        est_x, est_y, est_z (posição estimada só por câmera, por estação),
        true_x, true_y, true_z (ground truth no mesmo instante),
        position_error_m (distância 3D entre estimado e verdadeiro)
    """
    det = generate_camera_detections(scenario_id, traj_config, stations_df, camera_config)
    station_lookup = stations_df.set_index("station_id")[["x", "y", "z"]]

    rows = []
    for sid, d in det.groupby("station_id"):
        station_xyz = tuple(station_lookup.loc[sid])
        est_x, est_y, est_z = estimate_position_from_bearing(
            station_xyz, d["meas_azimuth_deg"].values,
            d["meas_elevation_deg"].values, d["meas_distance_m"].values,
        )
        true_state = trajectory_state_at(d["timestamp"].values, traj_config)
        pos_err = np.sqrt(
            (est_x - true_state["x"]) ** 2
            + (est_y - true_state["y"]) ** 2
            + (est_z - true_state["z"]) ** 2
        )
        rows.append(pd.DataFrame({
            "timestamp": d["timestamp"].values,
            "station_id": sid,
            "scenario_id": scenario_id,
            "est_x": est_x, "est_y": est_y, "est_z": est_z,
            "true_x": true_state["x"], "true_y": true_state["y"], "true_z": true_state["z"],
            "position_error_m": pos_err,
        }))

    return pd.concat(rows, ignore_index=True).sort_values("timestamp").reset_index(drop=True)


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, CAMERA
    from station_layout_generator import generate_stations

    print(f"Distância focal: {focal_length_px(CAMERA):.1f} px")
    print(f"Δd (ruído de disparidade/centróide): {pixel_disparity_noise_px(CAMERA):.3f} px")
    print(f"Erro de bearing (std): {bearing_error_std_deg(CAMERA):.4f}°")

    stations = generate_stations(1, TRAJECTORY, STATIONS)
    det = generate_camera_detections(1, TRAJECTORY, stations, CAMERA)
    print(f"\nTotal de detecções (cenário 1, 3 estações): {len(det)}")
    for sid in sorted(det.station_id.unique()):
        d = det[det.station_id == sid]
        print(f"  Estação {sid}: {len(d)} detecções | dist=[{d.distance_m.min():.1f},"
              f"{d.distance_m.max():.1f}]m | erro_prof(std)=[{d.depth_error_std_m.min():.2f},"
              f"{d.depth_error_std_m.max():.2f}]m")

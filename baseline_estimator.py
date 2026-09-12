"""
Módulo 8 - BaselineEstimator

Triangulação/multilateração geométrica por mínimos quadrados ponderados,
SEM componente temporal (nenhum modelo de movimento, nenhuma memória
entre instantes) - usado para isolar o ganho da combinação multi-sensor
"pura" do ganho adicional trazido pela filtragem temporal do FILTER_CONFIG.

Agrupamento por "época": cada instante de detecção de câmera (grade de
tempo compartilhada entre as 3 estações, ~30 Hz) define uma época. Para
cada época, reúnem-se: (a) todas as detecções de câmera disponíveis
naquele instante exato (1 a 3 estações, conforme FOV) e (b) as detecções
RF de todas as estações no instante RF mais próximo (grade RF também
compartilhada entre estações, ~833 Hz - a defasagem máxima possível é
metade do período RF, ~0,6 ms, desprezível face à dinâmica do drone).

Resolução: mínimos quadrados não lineares (Levenberg-Marquardt),
minimizando o resíduo ponderado (por 1/desvio-padrão) entre a geometria
prevista (azimute, e para câmera também elevação+distância) e a medida,
com tratamento circular do resíduo de azimute. Requer pelo menos uma
detecção de câmera na época (única fonte capaz de restringir a
coordenada z - RF sozinho é bearing-only 2D).
"""

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from camera_sensor_model import estimate_position_from_bearing


def _wrap_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def _residuals(pos: np.ndarray, meas_list: list, station_lookup: dict) -> np.ndarray:
    x, y, z = pos
    res = []
    for m in meas_list:
        sx, sy, sz = station_lookup[m["station_id"]]
        dx, dy, dz = x - sx, y - sy, z - sz
        horiz = np.sqrt(dx**2 + dy**2)

        az_pred = np.arctan2(dx, dy)
        d_az = _wrap_angle(az_pred - m["az_rad"])
        res.append(d_az / m["az_std_rad"])

        if m["sensor_type"] == "camera":
            el_pred = np.arctan2(dz, horiz)
            res.append((el_pred - m["el_rad"]) / m["el_std_rad"])
            dist_pred = np.sqrt(dx**2 + dy**2 + dz**2)
            res.append((dist_pred - m["dist_m"]) / m["dist_std_m"])
    return np.array(res)


def _build_measurement(row) -> dict:
    m = {
        "station_id": row.station_id,
        "sensor_type": row.sensor_type,
        "az_rad": np.radians(row.meas_azimuth_deg),
        "az_std_rad": np.radians(row.azimuth_noise_std_deg),
    }
    if row.sensor_type == "camera":
        m["el_rad"] = np.radians(row.meas_elevation_deg)
        m["el_std_rad"] = np.radians(row.elevation_noise_std_deg)
        m["dist_m"] = row.meas_distance_m
        m["dist_std_m"] = row.distance_noise_std_m
    return m


def solve_epoch(meas_list: list, station_lookup: dict, initial_guess: np.ndarray):
    """Resolve a posição 3D de uma época via mínimos quadrados ponderados."""
    result = least_squares(
        _residuals, initial_guess, args=(meas_list, station_lookup), method="lm",
        max_nfev=200,
    )
    return result.x, result.success


def run_baseline(detections: pd.DataFrame, stations_df: pd.DataFrame) -> pd.DataFrame:
    """
    Executa o baseline geométrico sobre o fluxo de detecções do Módulo 5.

    Retorna
    -------
    pd.DataFrame: timestamp, x, y, z, n_measurements, success, scenario_id
    """
    station_lookup = {
        int(r.station_id): (r.x, r.y, r.z) for _, r in stations_df.iterrows()
    }

    cam_all = detections[detections.sensor_type == "camera"]
    rf_all = detections[detections.sensor_type == "RF"]
    rf_times = np.sort(rf_all.timestamp.unique())

    epoch_times = np.sort(cam_all.timestamp.unique())

    rows = []
    prev_solution = None

    for t_cam in epoch_times:
        cam_meas_rows = cam_all[cam_all.timestamp == t_cam]

        # RF mais próximo no tempo (grade compartilhada entre estações)
        idx = np.searchsorted(rf_times, t_cam)
        idx = np.clip(idx, 0, len(rf_times) - 1)
        # compara vizinho anterior também, pega o mais próximo
        if idx > 0 and abs(rf_times[idx - 1] - t_cam) < abs(rf_times[idx] - t_cam):
            idx -= 1
        t_rf = rf_times[idx]
        rf_meas_rows = rf_all[rf_all.timestamp == t_rf]

        meas_list = [_build_measurement(r) for _, r in cam_meas_rows.iterrows()]
        meas_list += [_build_measurement(r) for _, r in rf_meas_rows.iterrows()]

        # Chute inicial: solução da época anterior, ou fallback via inversão
        # geométrica da primeira câmera disponível
        if prev_solution is not None:
            x0 = prev_solution
        else:
            first_cam = cam_meas_rows.iloc[0]
            sxyz = station_lookup[int(first_cam.station_id)]
            x0 = np.array(estimate_position_from_bearing(
                sxyz, first_cam.meas_azimuth_deg, first_cam.meas_elevation_deg,
                first_cam.meas_distance_m,
            ))

        sol, success = solve_epoch(meas_list, station_lookup, x0)
        prev_solution = sol

        rows.append({
            "timestamp": t_cam, "x": sol[0], "y": sol[1], "z": sol[2],
            "n_measurements": len(meas_list), "success": success,
        })

    out = pd.DataFrame(rows)
    out["scenario_id"] = detections.scenario_id.iloc[0]
    out["sensor_mode"] = "baseline_geometric"
    return out


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF, CAMERA
    from station_layout_generator import generate_stations
    from rf_sensor_model import generate_rf_detections
    from camera_sensor_model import generate_camera_detections
    from detection_association import associate_detections
    from trajectory_generator import trajectory_state_at

    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)

    result = run_baseline(merged, stations)
    true_state = trajectory_state_at(result.timestamp.values, TRAJECTORY)
    pos_err = np.sqrt(
        (result.x - true_state["x"])**2 + (result.y - true_state["y"])**2
        + (result.z - true_state["z"])**2
    )
    print(f"Total de épocas: {len(result)}")
    print(f"Taxa de sucesso do solver: {result.success.mean()*100:.1f}%")
    print(f"RMSE de posição 3D (baseline geométrico): {np.sqrt((pos_err**2).mean()):.2f} m")
    print(f"Erro mediano: {pos_err.median():.2f} m | máximo: {pos_err.max():.2f} m")

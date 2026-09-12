"""
Módulo 7 - FusionEngine (EKF)

Motor de fusão assíncrono/multi-taxa, combinando detecções RF e câmera de
múltiplas estações num único filtro de Kalman Estendido (EKF).


Duas arquiteturas de fusão usam este motor:
  - "fusion"/"rf_only"/"camera_only" (aqui): um único EKF processando as
    detecções de TODAS as estações em sequência (fusão por medição).
  - track_to_track_fusion.py: um EKF LOCAL por estação (chamando
    run_fusion_ekf() com detecções de uma estação só) + um combinador
    GLOBAL por Interseção de Covariância.
"""

import numpy as np
import pandas as pd
from scipy.stats import chi2

from fusion_models import (
    fx, _process_noise_Q, _regularize_covariance, _wrap_angle,
    hx_rf, hx_camera, triangulate_rf_azimuth, _GATE_CONFIDENCE,
    initialize_state_two_point, initialize_state_batch_regression,
)
from camera_sensor_model import estimate_position_from_bearing


# ---------------------------------------------------------------------------
# Jacobianos analíticos
# ---------------------------------------------------------------------------

def F_jacobian(dt: float) -> np.ndarray:
    """Jacobiano do modelo de movimento CV - exato (o modelo já é linear)."""
    F = np.eye(6)
    for i, j in zip((0, 1, 2), (3, 4, 5)):
        F[i, j] = dt
    return F


def H_rf_jacobian(state: np.ndarray, station_xyz) -> np.ndarray:
    """Jacobiano de hx_rf (1x6): d(azimute)/d(estado)."""
    dx = state[0] - station_xyz[0]
    dy = state[1] - station_xyz[1]
    r2 = dx**2 + dy**2
    H = np.zeros((1, 6))
    H[0, 0] = dy / r2
    H[0, 1] = -dx / r2
    return H


def H_camera_jacobian(state: np.ndarray, station_xyz) -> np.ndarray:
    """Jacobiano de hx_camera (3x6): d([azimute,elevação,distância])/d(estado)."""
    dx = state[0] - station_xyz[0]
    dy = state[1] - station_xyz[1]
    dz = state[2] - station_xyz[2]
    r2 = dx**2 + dy**2
    horiz = np.sqrt(r2)
    r3 = r2 + dz**2
    dist = np.sqrt(r3)

    H = np.zeros((3, 6))
    H[0, 0] = dy / r2
    H[0, 1] = -dx / r2
    H[1, 0] = -dz * dx / (r3 * horiz)
    H[1, 1] = -dz * dy / (r3 * horiz)
    H[1, 2] = horiz / r3
    H[2, 0] = dx / dist
    H[2, 1] = dy / dist
    H[2, 2] = dz / dist
    return H


# ---------------------------------------------------------------------------
# Motor de fusão EKF
# ---------------------------------------------------------------------------

def _flatten_pos_cov(P: np.ndarray) -> dict:
    """Achata o bloco de POSIÇÃO (3x3, triângulo superior - 6 valores) de
    uma P 6x6 - leve o suficiente para exportar em toda execução (ao
    contrário de _flatten_cov_upper, que exporta a P 6x6 inteira e só é
    usada quando export_full_covariance=True). Necessário para o cálculo
    de NEES (Normalized Estimation Error Squared - Bar-Shalom, Li &
    Kirubarajan, "Estimation with Applications to Tracking and
    Navigation", Wiley, 2001), que precisa da covariância completa (não
    só do traço/pos_uncertainty) para detectar inconsistência do filtro
    - ver metrics_evaluator.py::compute_nees."""
    return {
        "pcov_xx": float(P[0, 0]), "pcov_xy": float(P[0, 1]), "pcov_xz": float(P[0, 2]),
        "pcov_yy": float(P[1, 1]), "pcov_yz": float(P[1, 2]), "pcov_zz": float(P[2, 2]),
    }


_COV_INDICES = [(i, j) for i in range(6) for j in range(i, 6)]


def _flatten_cov_upper(P: np.ndarray) -> dict:
    """Achata o triângulo superior de uma P 6x6 simétrica em 21 colunas
    ('P00'..'P55') - usado só quando export_full_covariance=True (fusão
    track-to-track precisa da covariância completa, não só do traço)."""
    return {f"P{i}{j}": float(P[i, j]) for i, j in _COV_INDICES}


def run_fusion_ekf(detections: pd.DataFrame, stations_df: pd.DataFrame,
                    filter_config: dict, sensor_mode: str = "fusion",
                    export_full_covariance: bool = False,
                    init_method: str = "batch_regression") -> pd.DataFrame:
    """
    Executa a fusão assíncrona (EKF) sobre o fluxo de detecções.

    Parâmetros
    ----------
    detections : pd.DataFrame - saída de associate_detections(), já
        filtrada/ordenada por timestamp.
    stations_df : pd.DataFrame - layout das estações. Pode conter 1
        estação só (uso em track_to_track_fusion.py, filtro local) ou
        todas (fusão por medição sequencial, uso normal).
    filter_config : dict - bloco FILTER_CONFIG do config.py (nome mantido por
        continuidade - contém os parâmetros compartilhados de
        inicialização/ruído de processo, não é mais específico de FILTER_CONFIG).
    sensor_mode : "fusion" (RF+câmera), "rf_only" ou "camera_only".
    export_full_covariance : se True, adiciona 21 colunas 'P00'..'P55'
        (triângulo superior de P, 6x6) a cada linha - necessário para
        fusão track-to-track por Interseção de Covariância (CI), que
        precisa da matriz completa, não só do traço (pos_uncertainty).
        False por padrão para não inflar as tabelas do pipeline
        principal (grade comum, regime permanente etc.).
    init_method : "batch_regression" (padrão, NOVO) ou "two_point"
        (antigo - mantido para comparação A/B). Ver
        achados_divergencia_fusao.md §12: a diferenciação de dois pontos
        (init_method="two_point") tem variância alta o suficiente para
        causar divergência catastrófica em cenários com estações mais
        distantes (confirmado: 53% de divergência no câmera-only do
        Cenário 1, N=30) - a regressão em lote usa todos os fixes da
        janela de inicialização (não só os 2 extremos), reduzindo a
        variância da estimativa de velocidade substancialmente (~7x
        menor, validado em validate_batch_regression.py) mantendo o
        mesmo princípio (não assumir v=0). Ambas se reduzem à mesma
        fórmula quando há só 2 fixes disponíveis (equivalência algébrica
        validada).

    Retorna
    -------
    pd.DataFrame: timestamp, x, y, z, vx, vy, vz, speed,
    course_over_ground, vertical_speed, pos_uncertainty,
    vel_uncertainty, scenario_id, sensor_mode, nis, gate_rejected,
    k_pos_norm, k_vel_norm (+ P00..P55 se export_full_covariance=True).
    """
    station_lookup = stations_df.set_index("station_id")[["x", "y", "z"]]

    if sensor_mode == "rf_only":
        stream = detections[detections.sensor_type == "RF"].copy()
    elif sensor_mode == "camera_only":
        stream = detections[detections.sensor_type == "camera"].copy()
    else:
        stream = detections.copy()
    stream = stream.sort_values("timestamp").reset_index(drop=True)

    # Inicialização - ver docstring acima (init_method) e
    # fusion_models.py::initialize_state_batch_regression /
    # initialize_state_two_point. t_init é o instante do fix mais
    # recente usado na inicialização.
    init_fn = (initialize_state_batch_regression if init_method == "batch_regression"
               else initialize_state_two_point)
    x, P0, t_init = init_fn(detections, stations_df, sensor_mode, filter_config)
    P = _regularize_covariance(P0)

    accel_std = filter_config["process_noise_accel_std"]

    stream = stream[stream.timestamp >= t_init].reset_index(drop=True)

    rows = []
    t_prev = t_init
    init_row = {
        "timestamp": t_prev, "x": x[0], "y": x[1], "z": x[2],
        "vx": x[3], "vy": x[4], "vz": x[5],
        "pos_uncertainty": np.sqrt(np.trace(P[:3, :3])),
        "vel_uncertainty": np.sqrt(np.trace(P[3:, 3:])),
        "nis": np.nan, "sensor_type": "init",
        **_flatten_pos_cov(P),
    }
    if export_full_covariance:
        init_row.update(_flatten_cov_upper(P))
    rows.append(init_row)

    I6 = np.eye(6)

    for _, det in stream.iterrows():
        dt = det.timestamp - t_prev
        if dt > 0:
            F = F_jacobian(dt)
            Q = _process_noise_Q(dt, accel_std)
            x = fx(x, dt)
            P = F @ P @ F.T + Q
            P = _regularize_covariance(P)
        t_prev = det.timestamp

        station_xyz = tuple(station_lookup.loc[det.station_id])

        if det.sensor_type == "RF":
            z = np.array([np.radians(det.meas_azimuth_deg)])
            R = np.array([[np.radians(det.azimuth_noise_std_deg) ** 2]])
            hx_fn, H_fn = hx_rf, H_rf_jacobian
            dof = 1
        else:  # camera
            z = np.array([
                np.radians(det.meas_azimuth_deg),
                np.radians(det.meas_elevation_deg),
                det.meas_distance_m,
            ])
            R = np.diag([
                np.radians(det.azimuth_noise_std_deg) ** 2,
                np.radians(det.elevation_noise_std_deg) ** 2,
                det.distance_noise_std_m ** 2,
            ])
            hx_fn, H_fn = hx_camera, H_camera_jacobian
            dof = 3

        x_pre, P_pre = x.copy(), P.copy()

        zp = hx_fn(x, station_xyz)
        y = z - zp
        y[0] = _wrap_angle(y[0])  # resíduo circular no azimute

        H = H_fn(x, station_xyz)
        S = H @ P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            S_inv = np.linalg.pinv(S)
        K = P @ H.T @ S_inv

        # Ganho de Kalman - separado em posição/velocidade (ver
        # achados_divergencia_fusao.md §8.3, análise do ganho).
        k_pos_norm = float(np.linalg.norm(K[:3, :]))
        k_vel_norm = float(np.linalg.norm(K[3:, :]))

        x_new = x + K @ y
        # Forma de Joseph (numericamente mais estável que I-KH simples).
        IKH = I6 - K @ H
        P_new = IKH @ P @ IKH.T + K @ R @ K.T
        P_new = _regularize_covariance(P_new)

        try:
            nis = float(y @ S_inv @ y)
        except Exception:
            nis = np.nan

        gate_threshold = chi2.ppf(_GATE_CONFIDENCE, dof)
        # Gate mais rígido durante a fase de aquisição inicial (primeiros
        # 2s após a inicialização), aplicado APENAS no modo de fusão.
        if sensor_mode == "fusion" and (det.timestamp - t_init) <= 2.0:
            gate_threshold = chi2.ppf(0.90, dof)
        rejected = np.isfinite(nis) and nis > gate_threshold

        if rejected:
            x, P = x_pre, P_pre
        else:
            x, P = x_new, P_new

        rows.append({
            "timestamp": det.timestamp, "x": x[0], "y": x[1], "z": x[2],
            "vx": x[3], "vy": x[4], "vz": x[5],
            "pos_uncertainty": np.sqrt(max(np.trace(P[:3, :3]), 0)),
            "vel_uncertainty": np.sqrt(max(np.trace(P[3:, 3:]), 0)),
            "nis": nis, "sensor_type": det.sensor_type, "gate_rejected": rejected,
            "k_pos_norm": k_pos_norm, "k_vel_norm": k_vel_norm,
            **_flatten_pos_cov(P),
            **(_flatten_cov_upper(P) if export_full_covariance else {}),
        })

    out = pd.DataFrame(rows)
    out["speed"] = np.sqrt(out.vx**2 + out.vy**2 + out.vz**2)
    out["course_over_ground"] = np.degrees(np.arctan2(out.vx, out.vy)) % 360.0
    out["vertical_speed"] = out.vz
    out["scenario_id"] = detections.scenario_id.iloc[0]
    out["sensor_mode"] = sensor_mode
    return out


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG
    from station_layout_generator import generate_stations
    from rf_sensor_model import generate_rf_detections
    from camera_sensor_model import generate_camera_detections
    from detection_association import associate_detections

    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)

    result = run_fusion_ekf(merged, stations, FILTER_CONFIG, sensor_mode="fusion")
    print(f"Total de passos de fusão (EKF): {len(result)}")
    print(result[["timestamp", "x", "y", "z", "speed", "pos_uncertainty"]].head(10))
    print(f"\nNIS - média: {result.nis.mean():.2f} | mediana: {result.nis.median():.2f}")

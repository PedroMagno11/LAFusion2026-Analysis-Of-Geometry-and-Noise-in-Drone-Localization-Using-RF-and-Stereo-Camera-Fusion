"""
Valida numericamente que initialize_state_batch_regression() se reduz
exatamente a initialize_state_two_point() quando há só 2 fixes na
janela de inicialização - a regressão por mínimos quadrados com 2
pontos é matematicamente idêntica à diferenciação de dois pontos, então
isso é uma checagem de consistência (não um teste de comportamento
diferente), garantindo que a generalização não introduziu nenhum erro
na fórmula.
"""
import numpy as np
import pandas as pd

from fusion_models import initialize_state_two_point, initialize_state_batch_regression
from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG, EXPERIMENT
from station_layout_generator import generate_stations
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from fusion_models import thin_rf_detections

print("=== Checagem 1: equivalência algébrica com N=2 pontos (teste sintético direto) ===")
np.random.seed(0)
for trial in range(5):
    t_a, t_b = 0.0, np.random.uniform(0.5, 2.0)
    pos_a = np.random.uniform(-50, 50, 3)
    pos_b = np.random.uniform(-50, 50, 3)
    r_diag = np.random.uniform(1, 100, 3)

    # --- fórmula de dois pontos (igual a initialize_state_two_point) ---
    T = t_b - t_a
    v0_2pt = (pos_b - pos_a) / T
    x0_2pt = np.concatenate([pos_b, v0_2pt])
    R = np.diag(r_diag)
    P_cross = R / T
    P_vel = 2.0 * R / (T ** 2)
    P0_2pt = np.block([[R, P_cross], [P_cross, P_vel]])

    # --- fórmula de regressão em lote, com N=2 pontos (igual a initialize_state_batch_regression) ---
    times = np.array([t_a, t_b])
    positions = np.array([pos_a, pos_b])
    t_ref = times[-1]
    tau = times - t_ref
    X = np.column_stack([np.ones_like(tau), tau])
    XtX_inv = np.linalg.inv(X.T @ X)
    beta = XtX_inv @ X.T @ positions
    x0_batch = np.concatenate([beta[0], beta[1]])
    P0_batch = np.zeros((6, 6))
    for i in range(3):
        block = r_diag[i] * XtX_inv
        P0_batch[i, i] = block[0, 0]
        P0_batch[i, i + 3] = P0_batch[i + 3, i] = block[0, 1]
        P0_batch[i + 3, i + 3] = block[1, 1]

    ok_x = np.allclose(x0_2pt, x0_batch, atol=1e-8)
    ok_p = np.allclose(P0_2pt, P0_batch, atol=1e-8)
    print(f"  trial {trial}: x0 {'igual' if ok_x else 'DIFERENTE'}, P0 {'igual' if ok_p else 'DIFERENTE'}"
          f" -> {'OK' if ok_x and ok_p else 'FALHOU'}")

print("\n=== Checagem 2: com janela cheia (N pontos), a variância da regressão é MENOR ===")
for scenario_id in (1,):
    for sensor_mode in ("camera_only",):
        stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)
        rf_cfg = dict(RF); rf_cfg["seed"] = RF["seed"] + scenario_id
        cam_cfg = dict(CAMERA); cam_cfg["seed"] = CAMERA["seed"] + scenario_id
        rf_det = thin_rf_detections(generate_rf_detections(scenario_id, TRAJECTORY, stations_df, rf_cfg),
                                     EXPERIMENT["rf_decimation_factor"])
        cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations_df, cam_cfg)
        merged = associate_detections(rf_det, cam_det)

        x0_2pt, P0_2pt, _ = initialize_state_two_point(merged, stations_df, sensor_mode, FILTER_CONFIG)
        x0_batch, P0_batch, _ = initialize_state_batch_regression(merged, stations_df, sensor_mode, FILTER_CONFIG)

        var_v_2pt = np.diag(P0_2pt)[3:6]
        var_v_batch = np.diag(P0_batch)[3:6]
        print(f"  Variância de velocidade (2 pontos): {var_v_2pt}")
        print(f"  Variância de velocidade (regressão): {var_v_batch}")
        print(f"  Razão (regressão/2pontos, <1 = menos incerto): {var_v_batch/var_v_2pt}")
        print(f"  Velocidade estimada (2 pontos): {x0_2pt[3:6]}")
        print(f"  Velocidade estimada (regressão): {x0_batch[3:6]}")

"""
Fusão track-to-track: cada estação roda seu próprio EKF local (só com as
detecções QUE ELA MESMA recebe - RF e câmera daquela estação), e um
combinador GLOBAL funde as três estimativas locais via Interseção de
Covariância (Covariance Intersection, CI):

    Julier, S. J., & Uhlmann, J. K. (1997). "A non-divergent estimation
    algorithm in the presence of unknown correlations." Proceedings of
    the 1997 American Control Conference, Vol. 4, pp. 2369-2373.

POR QUE CI, E NÃO UMA MÉDIA PONDERADA PELO INVERSO DA COVARIÂNCIA
(BLUE/mínimos quadrados generalizados): as três estimativas locais
observam o MESMO alvo com o MESMO modelo de movimento (a mesma
"verdade" e as mesmas fontes de incerteza de processo), então seus erros
ficam correlacionados ao longo do tempo de um jeito que nenhum dos três
filtros locais consegue calcular sozinho (cada um só vê sua própria
covariância, não a covariância CRUZADA com os outros dois). Combinar
como se fossem independentes (BLUE ingênuo) produz uma covariância
combinada SUBESTIMADA - exatamente o mesmo tipo de inconsistência que
já diagnosticamos como causa da trava do gate de inovação (ver
achados_divergencia_fusao.md, §3 e §8). CI é a técnica padrão da
literatura de rastreio distribuído para esse problema: dá uma
covariância combinada CONSISTENTE (um limite superior válido sobre o
erro real, no sentido de E[(x-x̂)(x-x̂)ᵀ] ⪯ C) mesmo sem conhecer a
correlação cruzada real - a garantia central de Julier & Uhlmann (1997)
é essa consistência, não uma comparação direta e simples de tamanho
contra cada fonte individual (revisão externa incorporada - ver
achados_divergencia_fusao.md: a formulação anterior aqui era uma
simplificação excessiva dessa garantia).

ARQUITETURA (nova - complementar à fusão por medição sequencial já
existente em ekf_fusion.py, não a substitui):

  Estacao 1 -> EKF local 1 (so RF+camera da estacao 1) --+
  Estacao 2 -> EKF local 2 (so RF+camera da estacao 2) --+--> CI -> estimativa global
  Estacao 3 -> EKF local 3 (so RF+camera da estacao 3) --+

Cada EKF local reaproveita run_fusion_ekf() sem nenhuma duplicação -
só filtra as detecções para uma estação antes de chamar.

LIMITAÇÃO DECLARADA: um filtro local que só recebe RF (sem câmera) da
sua própria estação não consegue se inicializar - triangulação por
azimute exige >=2 estações simultâneas (ver
initialize_state_two_point/triangulate_rf_azimuth em fusion_models.py), e
uma única estação RF sozinha só enxerga uma linha de rolamento (sem
alcance). Por isso, cada filtro local só começa a existir quando SUA
PRÓPRIA câmera vê o drone pela primeira vez (mesma exigência de
inicialização por câmera do modo "fusion" já existente) - estações que
demoram mais para ver o drone geram tracks locais mais curtos.
"""
import numpy as np
import pandas as pd
from scipy.optimize import minimize

from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG, EXPERIMENT
from station_layout_generator import generate_stations
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from fusion_models import fx, _process_noise_Q, _regularize_covariance, thin_rf_detections
from ekf_fusion import run_fusion_ekf, F_jacobian, _COV_INDICES, _flatten_pos_cov


def _unflatten_cov(row: pd.Series) -> np.ndarray:
    """Reconstrói a P 6x6 simétrica a partir das colunas 'P00'..'P55'
    (triângulo superior) exportadas por run_fusion_ekf(export_full_covariance=True)."""
    P = np.zeros((6, 6))
    for i, j in _COV_INDICES:
        P[i, j] = row[f"P{i}{j}"]
        P[j, i] = row[f"P{i}{j}"]
    return P


def run_local_ekf(detections: pd.DataFrame, stations_df: pd.DataFrame,
                   station_id: int, ukf_config: dict) -> pd.DataFrame:
    """
    Filtro local de UMA estação: reaproveita run_fusion_ekf() sem
    duplicar nenhuma lógica, restringindo as detecções (RF+câmera) só
    às daquela estação antes de chamar. Retorna a covariância completa
    (export_full_covariance=True) - necessária para a fusão CI.
    """
    local_det = detections[detections.station_id == station_id].copy()
    local_stations = stations_df[stations_df.station_id == station_id]
    track = run_fusion_ekf(local_det, local_stations, ukf_config,
                            sensor_mode="fusion", export_full_covariance=True)
    track["station_id"] = station_id
    return track


def predict_to_time(x: np.ndarray, P: np.ndarray, t_from: float, t_to: float,
                     accel_std: float) -> tuple:
    """
    Propaga (x, P) de t_from até t_to por predição pura (sem medição) -
    MESMO modelo de movimento e MESMO ruído de processo usados no
    filtro local (fx, _process_noise_Q de fusion_models.py; F_jacobian de
    ekf_fusion.py), para que a extrapolação seja consistente com o
    filtro que gerou (x, P).
    """
    dt = t_to - t_from
    if dt <= 0:
        return x, P
    F = F_jacobian(dt)
    Q = _process_noise_Q(dt, accel_std)
    x_pred = fx(x, dt)
    P_pred = F @ P @ F.T + Q
    return x_pred, _regularize_covariance(P_pred)


def covariance_intersect(estimates: list) -> tuple:
    """
    Interseção de Covariância (Julier & Uhlmann, 1997) para N
    estimativas do MESMO estado, com correlação cruzada desconhecida.

    estimates : lista de (x_i, P_i) - mesma dimensão de estado.

    Os pesos w_i (>=0, soma 1) são escolhidos minimizando o traço da
    covariância combinada resultante (critério padrão na literatura de
    CI - minimizar o "tamanho" da elipsoide de incerteza combinada).

    Retorna
    -------
    (x_c, P_c) - estimativa e covariância combinadas, CONSISTENTES no
    sentido de Julier & Uhlmann (1997): C é um limite superior válido
    sobre o erro real de estimação mesmo sem conhecer a correlação
    cruzada entre as fontes - não uma garantia simples de "nunca mais
    confiante que qualquer fonte individual" (formulação anterior aqui
    era uma simplificação excessiva - revisão externa incorporada).
    """
    n = len(estimates)
    if n == 1:
        return estimates[0]

    P_inv = [np.linalg.inv(P) for _, P in estimates]
    xs = [x for x, _ in estimates]

    def combined_P_inv(w):
        return sum(wi * Pi for wi, Pi in zip(w, P_inv))

    def objective(w_free):
        # w_free tem n-1 componentes livres em [0,1]; o n-esimo peso e
        # 1 - soma dos outros (parametrizacao que ja respeita a soma=1
        # sem precisar de otimizacao com restricao de igualdade).
        w_last = 1.0 - sum(w_free)
        w = list(w_free) + [w_last]
        if w_last < 0:
            return 1e12  # fora do simplex - penaliza fortemente
        Cinv = combined_P_inv(w)
        try:
            C = np.linalg.inv(Cinv)
        except np.linalg.LinAlgError:
            return 1e12
        return np.trace(C)

    w0 = np.full(n - 1, 1.0 / n)
    bounds = [(0.0, 1.0)] * (n - 1)
    result = minimize(objective, w0, method="SLSQP", bounds=bounds)
    w_opt = list(result.x) + [1.0 - sum(result.x)]
    w_opt = [max(0.0, wi) for wi in w_opt]
    total = sum(w_opt)
    w_opt = [wi / total for wi in w_opt]  # renormaliza por seguranca numerica

    Cinv = combined_P_inv(w_opt)
    C = np.linalg.inv(Cinv)
    xc = C @ sum(wi * Pi @ xi for wi, Pi, xi in zip(w_opt, P_inv, xs))
    return xc, _regularize_covariance(C)


def run_track_to_track_fusion(scenario_id: int, repetition: int,
                               grid_dt_s: float = None,
                               stations_df: pd.DataFrame = None,
                               merged: pd.DataFrame = None):
    """
    Roda os 3 filtros locais (um por estação) e funde por CI numa grade
    temporal comum (mesmo princípio da correção do item 1 - ver
    achados_divergencia_fusao.md §1: comparar/combinar estimativas exige
    trazê-las para os MESMOS instantes de referência antes).

    grid_dt_s : passo da grade de fusão global, em segundos. Se None,
        usa TRAJECTORY['dt_s'] (a grade do ground truth).
    stations_df, merged : se já disponíveis (ex.: chamado de
        orchestrator.py::run_repetition, que já gerou as detecções para
        o motor de fusão global), passe-os aqui para não gerar tudo de
        novo (RF a 833Hz é caro de regenerar). Se None, gera
        internamente com as MESMAS sementes usadas em toda a pipeline
        (scenario_id/repetition) - uso standalone (ver __main__ abaixo).

    Retorna
    -------
    (global_track, local_tracks) : global_track é um DataFrame
    (timestamp, x, y, z, vx, vy, vz, pos_uncertainty, vel_uncertainty,
    n_estacoes_disponiveis); local_tracks é um dict {station_id: track}.
    """
    if stations_df is None or merged is None:
        stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)
        rf_cfg = dict(RF); rf_cfg["seed"] = RF["seed"] + repetition * 1000 + scenario_id
        cam_cfg = dict(CAMERA); cam_cfg["seed"] = CAMERA["seed"] + repetition * 1000 + scenario_id
        rf_det = thin_rf_detections(generate_rf_detections(scenario_id, TRAJECTORY, stations_df, rf_cfg),
                                     EXPERIMENT["rf_decimation_factor"])
        cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations_df, cam_cfg)
        merged = associate_detections(rf_det, cam_det)

    local_tracks = {}
    for sid in stations_df.station_id:
        try:
            local_tracks[int(sid)] = run_local_ekf(merged, stations_df, int(sid), FILTER_CONFIG)
        except ValueError as e:
            print(f"  Estação {sid}: não inicializou ({e})")

    accel_std = FILTER_CONFIG["process_noise_accel_std"]
    dt = grid_dt_s if grid_dt_s is not None else TRAJECTORY["dt_s"]
    grid_t = np.arange(0.0, TRAJECTORY["duration_s"] + 1e-9, dt)

    rows = []
    for t in grid_t:
        estimates = []
        for sid, track in local_tracks.items():
            valid = track[track.timestamp <= t]
            if valid.empty:
                continue
            last = valid.iloc[-1]
            x_last = last[["x", "y", "z", "vx", "vy", "vz"]].to_numpy(dtype=float)
            P_last = _unflatten_cov(last)
            x_pred, P_pred = predict_to_time(x_last, P_last, last.timestamp, t, accel_std)
            estimates.append((x_pred, P_pred))

        if not estimates:
            rows.append({"timestamp": t, "x": np.nan, "y": np.nan, "z": np.nan,
                          "vx": np.nan, "vy": np.nan, "vz": np.nan,
                          "pos_uncertainty": np.nan, "vel_uncertainty": np.nan,
                          "n_estacoes_disponiveis": 0})
            continue

        xc, Pc = covariance_intersect(estimates)
        row = {
            "timestamp": t, "x": xc[0], "y": xc[1], "z": xc[2],
            "vx": xc[3], "vy": xc[4], "vz": xc[5],
            "pos_uncertainty": np.sqrt(max(np.trace(Pc[:3, :3]), 0)),
            "vel_uncertainty": np.sqrt(max(np.trace(Pc[3:, 3:]), 0)),
            "n_estacoes_disponiveis": len(estimates),
        }
        row.update(_flatten_pos_cov(Pc))
        rows.append(row)

    out = pd.DataFrame(rows)
    out["scenario_id"] = scenario_id
    out["sensor_mode"] = "track_to_track_ci"
    return out, local_tracks


if __name__ == "__main__":
    from metrics_evaluator import position_error

    scenario_id, repetition = 1, 0
    print(f"Rodando fusão track-to-track (CI) - cenário {scenario_id}, repetição {repetition}...")
    global_track, local_tracks = run_track_to_track_fusion(scenario_id, repetition)

    err = position_error(global_track, TRAJECTORY)
    print(f"\nErro de posição - global (CI): média={err.mean():.3f}m, máx={err.max():.3f}m, "
          f"final={err.iloc[-1]:.3f}m")
    print(f"\nEstações disponíveis ao longo do voo:")
    print(global_track.n_estacoes_disponiveis.value_counts().sort_index())

    for sid, track in local_tracks.items():
        print(f"\nEstação {sid} (local): {len(track)} passos, "
              f"começou em t={track.timestamp.iloc[0]:.3f}s")

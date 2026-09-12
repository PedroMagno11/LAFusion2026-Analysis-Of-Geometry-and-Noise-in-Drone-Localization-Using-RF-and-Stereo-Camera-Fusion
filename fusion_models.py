"""
Modelos compartilhados de movimento e medição - usados pelo EKF
(ekf_fusion.py) e pelos filtros locais da fusão track-to-track
(track_to_track_fusion.py).

HISTÓRICO: este conteúdo estava em ukf_fusion.py (Módulo 7 original,
FusionEngine/FILTER_CONFIG). O FILTER_CONFIG foi REMOVIDO do projeto (ver
achados_divergencia_fusao.md, §8: o FILTER_CONFIG piorava com a inicialização por
diferenciação de dois pontos, por uma incompatibilidade entre a
correlação posição-velocidade dessa inicialização e a amostragem por
sigma-points - mecanismo confirmado via inspeção do ganho de Kalman). O
que sobra aqui é só o modelo físico (movimento CV, medição RF/câmera,
inicialização) - compartilhado, não específico de nenhum filtro.

Estado: [x, y, z, vx, vy, vz] (referencial local ENU, metros e m/s).

Modelo de movimento (predição): velocidade constante (CV), com ruído de
processo de aceleração branca discretizada (ver `process_noise_accel_std`
no config.py).

Modelo de medição - DOIS casos (RF só mede azimute; câmera mede
azimute+elevação+distância):
  - RF:     h(estado) = [azimute]                       (1D)
  - Câmera: h(estado) = [azimute, elevação, distância]  (3D)

Tratamento da descontinuidade angular (0°/360°): todas as contas internas
são feitas em RADIANOS, com resíduo circular (`_wrap_angle`) no
componente de azimute.
"""

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from camera_sensor_model import estimate_position_from_bearing

# Limiar de gating de inovação (NIS): atualizações cujo NIS exceda o
# percentil 99,99% da distribuição chi-quadrado esperada (para os graus
# de liberdade da medição) são rejeitadas - tratadas como outlier/medição
# espúria, não aplicadas ao estado. Prática padrão em tracking (gating/
# validation gate) para evitar que uma única medição anômala faça o
# filtro divergir irreversivelmente.
_GATE_CONFIDENCE = 0.9999

# ---------------------------------------------------------------------------
# Modelo de movimento e utilitários angulares
# ---------------------------------------------------------------------------


def fx(state: np.ndarray, dt: float) -> np.ndarray:
    """Modelo de velocidade constante (CV)."""
    x, y, z, vx, vy, vz = state
    return np.array([x + vx * dt, y + vy * dt, z + vz * dt, vx, vy, vz])


def _process_noise_Q(dt: float, accel_std: float) -> np.ndarray:
    """
    Matriz de ruído de processo (6x6), para o estado ordenado
    [x, y, z, vx, vy, vz] - bloco 2x2 de aceleração branca discretizada
    por eixo (x, y, z independentes), fórmula clássica:
        Q_pos_pos = dt^4/4 * var ; Q_pos_vel = dt^3/2 * var ; Q_vel_vel = dt^2 * var
    """
    var = accel_std ** 2
    q_pp = dt**4 / 4 * var
    q_pv = dt**3 / 2 * var
    q_vv = dt**2 * var

    Q = np.zeros((6, 6))
    for i, j in zip((0, 1, 2), (3, 4, 5)):  # (x,vx), (y,vy), (z,vz)
        Q[i, i] = q_pp
        Q[j, j] = q_vv
        Q[i, j] = Q[j, i] = q_pv
    return Q


def _regularize_covariance(P: np.ndarray, min_eig: float = 1e-10) -> np.ndarray:
    """
    Garante que P seja simétrica e positiva definida, corrigindo deriva
    numérica acumulada ao longo de milhares de atualizações sequenciais
    (comum em filtros rodando a taxas altas, como o RF a ~833 Hz aqui).
    Simetriza e recorta autovalores muito pequenos/negativos para um piso
    mínimo positivo.
    """
    P = 0.5 * (P + P.T)
    eigval, eigvec = np.linalg.eigh(P)
    eigval_clipped = np.clip(eigval, min_eig, None)
    return eigvec @ np.diag(eigval_clipped) @ eigvec.T


def _wrap_angle(a: np.ndarray) -> np.ndarray:
    """Normaliza ângulo(s) para o intervalo [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def hx_rf(state: np.ndarray, station_xyz) -> np.ndarray:
    """Medição RF: [azimute] (radianos)."""
    dx = state[0] - station_xyz[0]
    dy = state[1] - station_xyz[1]
    az = np.arctan2(dx, dy)
    return np.array([az])


def hx_camera(state: np.ndarray, station_xyz) -> np.ndarray:
    """Medição câmera: [azimute, elevação, distância] (radianos, radianos, metros)."""
    dx = state[0] - station_xyz[0]
    dy = state[1] - station_xyz[1]
    dz = state[2] - station_xyz[2]
    horiz = np.sqrt(dx**2 + dy**2)
    az = np.arctan2(dx, dy)
    el = np.arctan2(dz, horiz)
    dist = np.sqrt(dx**2 + dy**2 + dz**2)
    return np.array([az, el, dist])


def residual_z(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Resíduo de medição, com tratamento circular do componente de azimute
    (índice 0), tanto para RF (dim=1) quanto para câmera (dim=3).
    """
    res = a - b
    res[0] = _wrap_angle(res[0])
    return res


# ---------------------------------------------------------------------------
# Triangulação e inicialização
# ---------------------------------------------------------------------------

def triangulate_rf_azimuth(rf_rows: pd.DataFrame, station_lookup: dict) -> tuple:
    """
    Triangulação 2D (x, y) por interseção de bearings de azimute, usando
    SOMENTE medições RF de múltiplas estações - sem nenhuma informação de
    câmera. Usada para inicializar o modo RF-only de forma verdadeiramente
    isolada, refletindo o que um sistema RF autônomo conseguiria estimar
    sozinho (apenas posição horizontal; RF não informa elevação).

    Requer medições de pelo menos 2 estações não-colineares com o alvo.
    """
    stations_xy = np.array([station_lookup[int(r.station_id)][:2] for _, r in rf_rows.iterrows()])
    x0, y0 = stations_xy[:, 0].mean(), stations_xy[:, 1].mean()

    def residuals(pos):
        x, y = pos
        res = []
        for _, r in rf_rows.iterrows():
            sx, sy, _ = station_lookup[int(r.station_id)]
            dx, dy = x - sx, y - sy
            az_pred = np.arctan2(dx, dy)
            az_meas = np.radians(r.meas_azimuth_deg)
            az_std = np.radians(r.azimuth_noise_std_deg)
            res.append(_wrap_angle(az_pred - az_meas) / az_std)
        return np.array(res)

    result = least_squares(residuals, [x0, y0], method="lm", max_nfev=200)
    return result.x[0], result.x[1]


def thin_rf_detections(rf_det: pd.DataFrame, factor: int) -> pd.DataFrame:
    """Subamostra a taxa RF por um fator inteiro, mantendo todas as estações.
    Movido de orchestrator.py para aqui - usado tanto pelo EKF global
    quanto pela fusão track-to-track, e mantê-lo em orchestrator.py
    causava import circular com track_to_track_fusion.py."""
    if factor <= 1:
        return rf_det
    out = []
    for sid, d in rf_det.groupby("station_id"):
        d = d.sort_values("timestamp").reset_index(drop=True)
        out.append(d.iloc[::factor])
    return pd.concat(out, ignore_index=True)


def initialize_state_two_point(detections: pd.DataFrame, stations_df: pd.DataFrame,
                                sensor_mode: str, filter_config: dict):
    """
    Inicializa posição E velocidade via DIFERENCIAÇÃO DE DOIS PONTOS
    (two-point differencing) - técnica padrão de inicialização de
    filtros de rastreio quando não há sensor de velocidade direto (ver
    Blackman, S. S. & Popoli, R., "Design and Analysis of Modern
    Tracking Systems", Artech House, 1999, Seção 5.5.3).

    CORREÇÃO DE PROTOCOLO (item de inicialização de velocidade -
    substituiu vx=vy=vz=0 fixo, identificado como causa raiz mais
    provável da divergência de fusão em geometrias circular/assimétrica
    - ver achados_divergencia_fusao.md, seção 3): a trajetória real
    NUNCA começa parada, e assumir velocidade zero com uma incerteza
    fixa e desconectada da qualidade real da estimativa produz uma
    covariância inicial que subestima o erro verdadeiro em certas
    geometrias - a raiz do travamento por rejeição do gate documentado
    naquele arquivo.

    Usa os dois fixes de posição mais afastados dentro da janela
    filter_config['init_window_s']:

        x0 = [pos_b; (pos_b - pos_a) / T]
        P0 = [[R,      R/T   ],
              [R/T,    2R/T^2]]

    onde pos_a é o primeiro fix na janela, pos_b o último, T = t_b - t_a,
    e R é a incerteza assumida (igual nos dois fixes) de um único fix de
    posição derivado do sensor daquele modo.

    Para o modo RF-only: como o prior de altitude (z) não varia entre
    os fixes RF (RF não mede elevação), a diferenciação produz vz0=0
    com incerteza LARGA (2*rf_only_z_prior_std_m²/T²) - reflete
    corretamente que não há informação alguma sobre velocidade vertical
    nesse modo.

    LIMITAÇÃO DESCOBERTA (ver achados_divergencia_fusao.md §12): usar só
    2 pontos tem viés baixo mas VARIÂNCIA ALTA - quando R é grande (ex.:
    estações mais distantes, como no Cenário 1) ou a janela T é curta, a
    estimativa de velocidade ocasionalmente sai catastroficamente errada
    (documentado empiricamente: 53% de divergência no câmera-only do
    Cenário 1 com N=30). Prefira initialize_state_batch_regression() (
    abaixo), que generaliza esta função para usar TODOS os fixes da
    janela via mínimos quadrados, reduzindo a variância - esta função
    permanece para comparação/referência histórica.

    Levanta ValueError se houver menos de 2 fixes disponíveis na janela.

    Retorna
    -------
    (x0, P0, t_ref) : x0 é o vetor de estado 6D, P0 a covariância 6x6,
    t_ref = t_b (o instante do fix mais recente usado) - usado pelo
    chamador para não reprocessar as detecções já consumidas aqui.
    """
    fixes, r_diag = _collect_init_fixes(detections, stations_df, sensor_mode, filter_config)

    t_a, pos_a = fixes[0]
    t_b, pos_b = fixes[-1]
    T = t_b - t_a
    if T <= 0:
        raise ValueError("Intervalo de tempo nulo entre os dois fixes de inicialização.")

    v0 = (pos_b - pos_a) / T
    x0 = np.concatenate([pos_b, v0])

    R = np.diag(r_diag)
    P_cross = R / T
    P_vel = 2.0 * R / (T ** 2)
    P0 = np.block([[R, P_cross], [P_cross, P_vel]])

    return x0, P0, t_b


def _collect_init_fixes(detections: pd.DataFrame, stations_df: pd.DataFrame,
                         sensor_mode: str, filter_config: dict):
    """
    Coleta os fixes de posição (timestamp, posição 3D) dentro da janela
    filter_config['init_window_s'] - compartilhado entre
    initialize_state_two_point() (usa só os 2 extremos) e
    initialize_state_batch_regression() (usa todos).

    Retorna (fixes, r_diag): fixes é uma lista [(t, pos_array), ...]
    ordenada por tempo; r_diag é a incerteza (variância) assumida de um
    único fix, por eixo [var_x, var_y, var_z].

    Levanta ValueError se houver menos de 2 fixes na janela.
    """
    station_lookup_dict = {int(r.station_id): (r.x, r.y, r.z) for _, r in stations_df.iterrows()}
    window_s = filter_config.get("init_window_s", 1.0)

    fixes = []
    if sensor_mode == "rf_only":
        rf_dets = detections[detections.sensor_type == "RF"]
        if len(rf_dets) == 0:
            raise ValueError("Nenhuma detecção RF disponível para inicializar o modo RF-only.")
        t0 = rf_dets.timestamp.iloc[0]
        window = rf_dets[rf_dets.timestamp <= t0 + window_s]
        z_prior = filter_config.get("rf_only_z_prior_m", 25.0)
        for t in sorted(window.timestamp.unique()):
            epoch = window[window.timestamp == t]
            if epoch.station_id.nunique() >= 2:
                x, y = triangulate_rf_azimuth(epoch, station_lookup_dict)
                fixes.append((t, np.array([x, y, z_prior])))
        if len(fixes) < 2:
            raise ValueError("Menos de 2 épocas RF (>=2 estações) na janela de inicialização "
                              f"(init_window_s={window_s}s) - triangulação de velocidade não é possível.")
        r_std_xy = filter_config["initial_pos_std_m"]
        r_std_z = filter_config.get("rf_only_z_prior_std_m", 25.0)
        r_diag = np.array([r_std_xy ** 2, r_std_xy ** 2, r_std_z ** 2])
    else:
        cam_dets = detections[detections.sensor_type == "camera"]
        if len(cam_dets) == 0:
            raise ValueError("Nenhuma detecção de câmera disponível para inicializar o filtro.")
        t0 = cam_dets.timestamp.iloc[0]
        window = cam_dets[cam_dets.timestamp <= t0 + window_s]
        for _, det in window.iterrows():
            sxyz = station_lookup_dict[int(det.station_id)]
            pos = np.array(estimate_position_from_bearing(
                sxyz, det.meas_azimuth_deg, det.meas_elevation_deg, det.meas_distance_m))
            fixes.append((det.timestamp, pos))
        if len(fixes) < 2:
            raise ValueError("Menos de 2 detecções de câmera na janela de inicialização "
                              f"(init_window_s={window_s}s).")
        r_std = filter_config["initial_pos_std_m"]
        r_diag = np.array([r_std ** 2, r_std ** 2, r_std ** 2])

    fixes.sort(key=lambda f: f[0])
    return fixes, r_diag


def initialize_state_batch_regression(detections: pd.DataFrame, stations_df: pd.DataFrame,
                                       sensor_mode: str, filter_config: dict):
    """
    Generalização de initialize_state_two_point(): em vez de usar só os 2
    fixes extremos da janela, ajusta posição e velocidade por MÍNIMOS
    QUADRADOS usando TODOS os fixes disponíveis em
    filter_config['init_window_s'] (ex.: câmera a 30Hz dá ~30 fixes numa
    janela de 1s, contra só 2 usados antes).

    MOTIVAÇÃO (ver achados_divergencia_fusao.md §12): a diferenciação de
    dois pontos é não-viesada mas tem variância alta - com R grande
    (estações distantes) ou T curto, ocasionalmente produz uma
    velocidade catastroficamente errada (confirmado empiricamente: 53%
    de divergência no câmera-only do Cenário 1 com N=30, causa raiz
    isolada e confirmada comparando a mesma semente com as duas
    inicializações). Usar mais pontos reduz a variância da estimativa
    proporcionalmente (regressão com N pontos ~ N vezes menos variância
    que 2 pontos, para pontos igualmente espaçados) - trade-off clássico
    viés-variância: mantém o mesmo princípio (não assumir v=0) mas com
    uma estimativa mais estável.

    Para cada eixo (x, y, z) independente, ajusta o modelo linear
        pos(τ) = pos_ref + v * τ,   τ = t - t_ref
    por mínimos quadrados ordinários (mesma matriz de projeto X para os
    3 eixos, já que usam os mesmos instantes de tempo), com t_ref = o
    timestamp do fix MAIS RECENTE da janela (mesma convenção de
    initialize_state_two_point - o estado é definido "agora", não no
    passado). A covariância de (pos_ref, v) vem da fórmula padrão de
    mínimos quadrados: Cov(β) = σ² (XᵀX)⁻¹, com σ²=r_diag[eixo].

    VERIFICAÇÃO DE CONSISTÊNCIA: com exatamente 2 fixes, esta função se
    reduz algebricamente à fórmula de initialize_state_two_point() -
    XᵀX)⁻¹ = [[1, 1/T], [1/T, 2/T²]] para 2 pontos espaçados por T,
    idêntico a [[R,R/T],[R/T,2R/T²]]/R. Validado numericamente em
    validate_batch_regression.py.

    Retorna
    -------
    (x0, P0, t_ref) - mesmo formato de initialize_state_two_point().
    """
    fixes, r_diag = _collect_init_fixes(detections, stations_df, sensor_mode, filter_config)

    times = np.array([f[0] for f in fixes])
    positions = np.array([f[1] for f in fixes])  # shape (N, 3)
    t_ref = times[-1]
    tau = times - t_ref  # <= 0, tau[-1] == 0

    if np.allclose(tau, tau[0]):
        raise ValueError("Todos os fixes de inicialização têm o mesmo timestamp - "
                          "impossível estimar velocidade por regressão.")

    X = np.column_stack([np.ones_like(tau), tau])  # (N, 2): [intercepto, tempo]
    XtX = X.T @ X
    XtX_inv = np.linalg.inv(XtX)
    beta = XtX_inv @ X.T @ positions  # (2, 3): linha 0 = posição em t_ref, linha 1 = velocidade

    pos_ref = beta[0]
    v_est = beta[1]
    x0 = np.concatenate([pos_ref, v_est])

    P0 = np.zeros((6, 6))
    for i in range(3):
        block = r_diag[i] * XtX_inv  # 2x2: [[var_pos, cov_pos_vel], [cov_pos_vel, var_vel]]
        P0[i, i] = block[0, 0]
        P0[i, i + 3] = P0[i + 3, i] = block[0, 1]
        P0[i + 3, i + 3] = block[1, 1]

    return x0, P0, t_ref

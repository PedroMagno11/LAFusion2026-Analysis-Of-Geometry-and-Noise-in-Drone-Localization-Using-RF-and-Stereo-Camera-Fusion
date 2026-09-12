"""
Módulo 10 - MetricsEvaluator

Centraliza o cálculo das métricas de avaliação definidas no Item 9 da
metodologia consolidada:
  1. RMSE de posição 3D (por cenário/modo/repetição).
  2. Validação cruzada entre GDOP teórico (Módulo 6) e RMSE empírico.
  3. Comparação estatística entre cenários (múltiplas repetições Monte
     Carlo, geradas pelo Orquestrador - Módulo 11).
  4. Ganho da fusão (RF-only / câmera-only / fusão).

Este módulo consome as saídas já padronizadas do Módulo 9
(TelemetryExporter) ou diretamente os DataFrames de run_fusion()/
run_baseline().
"""

import numpy as np
import pandas as pd
from scipy import stats

from trajectory_generator import trajectory_state_at
from gdop_calculator import compute_gdop_along_trajectory


def _position_error_components(track_df: pd.DataFrame, traj_config: dict):
    """Retorna (dx, dy, dz) - erro por eixo entre a trilha estimada e o
    ground truth. Usado por position_error (3D) e position_error_horizontal
    (2D, xy - ver item 4 da revisão externa: RF só mede azimute, então
    RMSE 3D penaliza injustamente por um eixo (altitude) que o RF-only
    estruturalmente não observa - análogo à distinção HDOP/VDOP em
    GPS/navegação, aplicada aqui a RMSE em vez de DOP)."""
    if "ground_truth_x" in track_df.columns:
        gx, gy, gz = track_df.ground_truth_x, track_df.ground_truth_y, track_df.ground_truth_z
    else:
        true_state = trajectory_state_at(track_df.timestamp.values, traj_config)
        gx, gy, gz = true_state["x"], true_state["y"], true_state["z"]
    return track_df.x - gx, track_df.y - gy, track_df.z - gz


def position_error(track_df: pd.DataFrame, traj_config: dict) -> np.ndarray:
    """Erro de posição 3D (m) entre a trilha estimada e o ground truth, por linha."""
    dx, dy, dz = _position_error_components(track_df, traj_config)
    return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)


def position_error_horizontal(track_df: pd.DataFrame, traj_config: dict) -> np.ndarray:
    """Erro de posição HORIZONTAL (xy, m) - ignora o eixo z (altitude).
    Métrica principal para métodos que não observam altitude (RF-only) -
    ver docstring de _position_error_components. Reportada para TODOS os
    métodos (não só RF-only) para permitir comparação justa nos mesmos
    termos (revisão externa, item 4)."""
    dx, dy, _dz = _position_error_components(track_df, traj_config)
    return np.sqrt(dx ** 2 + dy ** 2)


def compute_rmse(track_df: pd.DataFrame, traj_config: dict) -> float:
    """RMSE de posição 3D de uma única execução (trilha), sobre a grade
    NATIVA de exportação da trilha (uma linha por atualização do
    estimador). ATENÇÃO: não comparar valores desta função entre modos
    de sensor diferentes (fusão/RF-only/câmera-only/baseline) - cada
    modo grava uma linha em taxas diferentes (833 Hz vs 30 Hz vs ~20 Hz),
    então cada RMSE está implicitamente ponderado pela cadência de
    atualização do seu próprio sensor, não pelo tempo relógio. Para
    comparação entre modos, usar compute_rmse_common_grid().
    """
    err = position_error(track_df, traj_config)
    return float(np.sqrt(np.mean(err ** 2)))


def resample_to_common_grid(track_df: pd.DataFrame, traj_config: dict,
                             grid_dt_s: float = None) -> pd.DataFrame:
    """
    Reamostra a trilha estimada (gravada na taxa nativa do sensor/modo -
    833 Hz para RF, 30 Hz para câmera, assíncrona para fusão) para uma
    grade temporal comum, por padrão a mesma grade do ground truth
    (traj_config['dt_s'], de 0 a duration_s).

    CORREÇÃO DE PROTOCOLO (registrada em conversa com o orientador,
    item 1 da lista de correções): antes desta função, o RMSE agregado
    era calculado linha a linha do CSV exportado, sem sincronização
    temporal entre modos. Como cada atualização do filtro vira uma
    linha, e o RF atualiza ~833 Hz contra ~30 Hz da câmera, o RF ficava
    implicitamente sobre-representado no RMSE da fusão. A solução -
    associar/interpolar as trajetórias estimadas nos mesmos instantes de
    referência antes de comparar - segue a definição de ATE (Absolute
    Trajectory Error) de Sturm, Engelhard, Endres, Burgard & Cremers,
    "A benchmark for the evaluation of RGB-D SLAM systems", IROS 2012
    (associação de poses por timestamp antes do cálculo do erro), e o
    tratamento de alinhamento temporal detalhado em Zhang & Scaramuzza,
    "A Tutorial on Quantitative Trajectory Evaluation for Visual(-
    Inertial) Odometry", IROS 2018 (a confirmar seção/página exata na
    redação final do artigo).

    NUNCA extrapola: instantes da grade fora do intervalo coberto pelas
    estimativas nativas da trilha (antes da primeira ou depois da
    última) recebem NaN em x/y/z, e não NaN inventado por
    interpolação constante - isso evita "esticar" artificialmente a
    primeira/última estimativa disponível.

    Parâmetros
    ----------
    track_df : DataFrame com colunas 'timestamp', 'x', 'y', 'z' (saída
        nativa de um dos modos).
    traj_config : dict de configuração da trajetória (TRAJECTORY).
    grid_dt_s : passo da grade comum, em segundos. Se None, usa
        traj_config['dt_s'] (a taxa do ground truth, 20 Hz por padrão -
        a única taxa do sistema que não é derivada de nenhum sensor).

    Retorna
    -------
    DataFrame com colunas 'timestamp', 'x', 'y', 'z' (NaN fora do
    intervalo coberto), 'ground_truth_x/y/z' (verdade exata, calculada
    analiticamente via trajectory_state_at - não interpolada a partir
    das colunas de ground truth do CSV nativo, para não somar dois
    erros de interpolação). Os atributos .attrs trazem
    'acquisition_gap_s' (tempo, em s, até a primeira estimativa nativa
    disponível) e 'n_native_points' (tamanho da trilha original, para
    auditoria).
    """
    dt = grid_dt_s if grid_dt_s is not None else traj_config["dt_s"]
    grid_t = np.arange(0.0, traj_config["duration_s"] + 1e-9, dt)

    t_native = track_df["timestamp"].to_numpy()
    order = np.argsort(t_native)
    t_native = t_native[order]
    t0, t1 = t_native[0], t_native[-1]
    in_range = (grid_t >= t0) & (grid_t <= t1)

    resampled = {"timestamp": grid_t}
    for col in ("x", "y", "z"):
        v_native = track_df[col].to_numpy()[order]
        v_interp = np.interp(grid_t, t_native, v_native)
        resampled[col] = np.where(in_range, v_interp, np.nan)

    true_state = trajectory_state_at(grid_t, traj_config)
    resampled["ground_truth_x"] = true_state["x"]
    resampled["ground_truth_y"] = true_state["y"]
    resampled["ground_truth_z"] = true_state["z"]

    out = pd.DataFrame(resampled)
    out.attrs["acquisition_gap_s"] = float(t0)
    out.attrs["n_native_points"] = int(len(t_native))
    return out


def compute_rmse_common_grid(track_df: pd.DataFrame, traj_config: dict,
                              grid_dt_s: float = None,
                              exclude_before_s: float = None) -> dict:
    """
    RMSE (e mediana/p95) calculados sobre a grade temporal comum - a
    versão a usar para QUALQUER comparação entre modos de sensor (ver
    resample_to_common_grid para a justificativa).

    exclude_before_s : se fornecido, descarta também os instantes da
        grade anteriores a esse tempo (em s), para separar o transiente
        de aquisição/convergência do regime permanente. Se None,
        inclui todo o intervalo coberto pela trilha nativa (só exclui o
        que é matematicamente indefinido - antes da 1ª estimativa).

    Retorna dict com rmse_m, median_error_m, p95_error_m (3D) e
    rmse_xy_m, median_error_xy_m, p95_error_xy_m (horizontal, ignora z -
    ver position_error_horizontal), n_grid_points (tamanho total da
    grade comum), n_valid_points (quantos pontos da grade entraram no
    cálculo) e acquisition_gap_s (para auditoria/diagnóstico de quanto
    tempo cada modo levou até a 1ª estimativa).
    """
    grid = resample_to_common_grid(track_df, traj_config, grid_dt_s)
    err = np.sqrt((grid.x - grid.ground_truth_x) ** 2 +
                  (grid.y - grid.ground_truth_y) ** 2 +
                  (grid.z - grid.ground_truth_z) ** 2)
    err_xy = np.sqrt((grid.x - grid.ground_truth_x) ** 2 +
                      (grid.y - grid.ground_truth_y) ** 2)

    mask = err.notna()
    if exclude_before_s is not None:
        mask &= grid.timestamp >= exclude_before_s
    err_valid = err[mask].to_numpy()
    err_xy_valid = err_xy[mask].to_numpy()

    if len(err_valid) == 0:
        return {"rmse_m": np.nan, "median_error_m": np.nan, "p95_error_m": np.nan,
                "rmse_xy_m": np.nan, "median_error_xy_m": np.nan, "p95_error_xy_m": np.nan,
                "n_grid_points": len(grid), "n_valid_points": 0,
                "acquisition_gap_s": grid.attrs["acquisition_gap_s"]}

    return {
        "rmse_m": float(np.sqrt(np.mean(err_valid ** 2))),
        "median_error_m": float(np.median(err_valid)),
        "p95_error_m": float(np.percentile(err_valid, 95)),
        "rmse_xy_m": float(np.sqrt(np.mean(err_xy_valid ** 2))),
        "median_error_xy_m": float(np.median(err_xy_valid)),
        "p95_error_xy_m": float(np.percentile(err_xy_valid, 95)),
        "n_grid_points": len(grid),
        "n_valid_points": int(len(err_valid)),
        "acquisition_gap_s": grid.attrs["acquisition_gap_s"],
    }


def summarize_run(track_df: pd.DataFrame, traj_config: dict, scenario_id: int,
                   sensor_mode: str, repetition: int = 0) -> dict:
    """Resumo de métricas de uma única execução, no formato de uma linha de tabela."""
    err = position_error(track_df, traj_config)
    return {
        "scenario_id": scenario_id,
        "sensor_mode": sensor_mode,
        "repetition": repetition,
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        "mean_error_m": float(np.mean(err)),
        "median_error_m": float(np.median(err)),
        "p95_error_m": float(np.percentile(err, 95)),
        "max_error_m": float(np.max(err)),
        "n_points": len(err),
    }


def aggregate_runs(run_summaries: list) -> pd.DataFrame:
    """
    Agrega múltiplos resumos de execução (ex.: várias repetições Monte
    Carlo, cenários e modos) em uma tabela única, com estatísticas
    (média, desvio-padrão) por combinação cenário x modo.
    """
    df = pd.DataFrame(run_summaries)
    agg = df.groupby(["scenario_id", "sensor_mode"]).agg(
        rmse_mean=("rmse_m", "mean"),
        rmse_std=("rmse_m", "std"),
        mean_error_mean=("mean_error_m", "mean"),
        median_error_mean=("median_error_m", "mean"),
        n_repetitions=("repetition", "count"),
    ).reset_index()
    return agg, df


def correlate_gdop_rmse(track_df: pd.DataFrame, traj_config: dict, stations_df: pd.DataFrame,
                         scenario_id: int, n_bins: int = 20) -> dict:
    """
    Validação cruzada: correlaciona o GDOP teórico (calculado ao longo da
    trajetória, Módulo 6) com o erro de posição empírico observado na
    mesma trajetória, agregando em bins de tempo para suavizar ruído
    ponto-a-ponto e comparar as TENDÊNCIAS (não o valor bruto, que tem
    escalas diferentes - GDOP bearing-only tem unidade de metro (sob variância angular unitária), não é adimensional - ver gdop_calculator.py).

    Retorna
    -------
    dict com: correlação de Pearson, p-valor, e os arrays binados (para
    plotagem).
    """
    gdop_df = compute_gdop_along_trajectory(scenario_id, traj_config, stations_df)
    err = position_error(track_df, traj_config)

    t_bins = np.linspace(0, traj_config["duration_s"], n_bins + 1)
    bin_idx_track = np.digitize(track_df.timestamp, t_bins) - 1
    bin_idx_gdop = np.digitize(gdop_df.t, t_bins) - 1

    err_binned = np.array([
        err[bin_idx_track == i].mean() if np.any(bin_idx_track == i) else np.nan
        for i in range(n_bins)
    ])
    gdop_binned = np.array([
        gdop_df.gdop.values[bin_idx_gdop == i].mean() if np.any(bin_idx_gdop == i) else np.nan
        for i in range(n_bins)
    ])

    valid = np.isfinite(err_binned) & np.isfinite(gdop_binned)
    if valid.sum() < 3:
        return {"pearson_r": np.nan, "p_value": np.nan,
                "err_binned": err_binned, "gdop_binned": gdop_binned}

    r, p = stats.pearsonr(gdop_binned[valid], err_binned[valid])
    return {"pearson_r": float(r), "p_value": float(p),
            "err_binned": err_binned, "gdop_binned": gdop_binned,
            "t_bins": t_bins[:-1]}


def estimate_convergence_time(track_df: pd.DataFrame, k: float = 3.0,
                               dwell_s: float = 1.0,
                               floor_fraction: float = 0.5,
                               max_floor_to_min_ratio: float = 2.0) -> float:
    """
    Estima t_conv - o instante em que a execução (FILTER_CONFIG: fusão, RF-only ou
    câmera-only) atinge regime permanente - a partir da própria
    incerteza de posição que o filtro exporta (pos_uncertainty =
    sqrt(trace(P[:3,:3])), ver ekf_fusion.py::run_fusion_ekf).

    CRITÉRIO (documentado e aprovado em
    criterio_transiente_regime_permanente.md - item 3 da lista de
    correções do protocolo de avaliação):
        floor_run = mediana de pos_uncertainty na segunda metade da
                    trajetória (t >= floor_fraction * duração)
        threshold = k * floor_run
        t_conv    = primeiro timestamp a partir do qual pos_uncertainty
                    permanece <= threshold por pelo menos dwell_s
                    segundos contínuos (ou até o fim dos dados
                    disponíveis, se a trajetória acabar antes do dwell
                    completo sem nenhum ponto acima do limiar nesse
                    trecho final).

    k, dwell_s e floor_fraction são ASSUNÇÃO DE PROJETO (decisão de
    engenharia, não valor de literatura) - ver documento de critério
    para a justificativa e o plano de validação visual antes de travar
    esses números em definitivo.

    Retorna
    -------
    float (t_conv, em segundos) ou None se:
      (a) a trilha não tem pos_uncertainty válido (ex.: baseline
          geométrico - memoryless, ver telemetry_exporter.py); ou
      (b) a execução NUNCA atinge o critério de convergência dentro da
          duração registrada - classificada como NÃO-CONVERGENTE, não
          como "transiente igual à trajetória inteira" (ver §5 do
          documento de critério: isso deve virar taxa de divergência,
          não ser absorvido silenciosamente pela exclusão de transiente); ou
    (c) a execução não apresenta um regime permanente estável de fato -
          isto é, pos_uncertainty CRESCE ao longo da trajetória em vez
          de decair e estabilizar em um piso (ex.: RF-only, cuja
          incerteza de altitude nunca é corrigida por nenhuma medição
          subsequente e degrada de forma aproximadamente monotônica com
          a evolução do GDOP ao longo do trajeto - ver
          rf_only_z_prior_std_m em config.py). Detectado comparando o
          piso da segunda metade (floor_run) com o mínimo observado em
          toda a execução: se floor_run > max_floor_to_min_ratio vezes
          esse mínimo, não existe "regime permanente" a isolar - a
          consequência errada de NÃO checar isso seria reportar
          t_conv ~ 0s (falso "convergiu instantaneamente"), quando na
          verdade a execução nunca estabiliza. Validado empiricamente:
          fusão e câmera-only têm razão piso/mínimo ~1,1-1,45 nos três
          cenários; RF-only tem ~3,15 nos três - separação nítida que
          sustenta max_floor_to_min_ratio=2.0 como padrão (ASSUNÇÃO DE
          PROJETO, calibrada nos dados-piloto, não da literatura).
    """
    if "pos_uncertainty" not in track_df.columns:
        return None
    df = track_df.dropna(subset=["pos_uncertainty"]).sort_values("timestamp")
    if df.empty:
        return None

    t = df["timestamp"].to_numpy()
    u = df["pos_uncertainty"].to_numpy()
    duration = t[-1] - t[0]
    if duration <= 0:
        return None

    global_min = float(np.min(u))
    if not np.isfinite(global_min) or global_min <= 0:
        return None

    floor_mask = t >= (t[0] + floor_fraction * duration)
    if not floor_mask.any():
        return None
    floor_run = float(np.median(u[floor_mask]))
    if not np.isfinite(floor_run) or floor_run <= 0:
        return None

    if floor_run > max_floor_to_min_ratio * global_min:
        # sem regime permanente estável (crescimento monotônico/
        # estrutural, não transiente de aquisição) - ver caso (c) acima.
        return None

    threshold = k * floor_run
    below = u <= threshold

    n = len(t)
    for i in range(n):
        if not below[i]:
            continue
        window_end = t[i] + dwell_s
        j = i
        while j < n and t[j] <= window_end and below[j]:
            j += 1
        if j == n:
            # dados acabam antes do fim da janela de dwell, mas nada
            # ultrapassou o limiar nesse trecho final - aceita por
            # ausência de evidência em contrário.
            return float(t[i])
        if t[j] > window_end:
            # manteve-se abaixo do limiar pela janela de dwell inteira
            return float(t[i])
        # below[j] é False dentro da janela -> falha, tenta o próximo i

    return None  # nunca convergiu dentro da duração observada


def compute_nees(track_df: pd.DataFrame, traj_config: dict) -> np.ndarray:
    """
    NEES (Normalized Estimation Error Squared) de posição - teste de
    CONSISTÊNCIA de filtro padrão (Bar-Shalom, Li & Kirubarajan,
    "Estimation with Applications to Tracking and Navigation", Wiley,
    2001, Cap. 5): mede se a covariância que o filtro reporta (P) é
    estatisticamente compatível com o erro REAL (conhecido aqui por ser
    simulação, via ground truth) - não apenas se P é "pequena" (que pode
    ocorrer mesmo com o filtro genuinamente perdido - ver
    achados_divergencia_fusao.md §3: covariância subestimada é
    justamente o mecanismo da trava do gate já diagnosticado ali).

        NEES(t) = e(t)ᵀ P_pos(t)⁻¹ e(t)

    onde e(t) é o erro de posição 3D real e P_pos a submatriz 3x3 de
    posição da covariância do filtro. Para um filtro consistente, o
    valor ESPERADO de NEES é igual à dimensão do erro (3, aqui) - forma
    quadrática de um vetor gaussiano 3D normalizado pela própria
    covariância segue qui-quadrado com 3 graus de liberdade. NEES
    sistematicamente > 3 indica covariância SUBESTIMADA (filtro
    overconfident, meio pelo qual a trava do gate acontece); < 3 indica
    SUPERESTIMADA (conservador demais).

    Requer as colunas pcov_xx/xy/xz/yy/yz/zz (ver
    ekf_fusion.py::_flatten_pos_cov) - ausentes para baseline_geometric
    (memoryless, sem covariância de filtro) e para track-to-track em
    instantes sem nenhuma estação disponível (NaN). Retorna NaN nesses
    pontos, não erro.

    Retorna
    -------
    np.ndarray com NEES(t), mesmo comprimento/índice de track_df - NaN
    onde não computável (matriz singular ou colunas de covariância
    ausentes).
    """
    required = ["pcov_xx", "pcov_xy", "pcov_xz", "pcov_yy", "pcov_yz", "pcov_zz"]
    if not all(c in track_df.columns for c in required):
        return np.full(len(track_df), np.nan)

    dx, dy, dz = _position_error_components(track_df, traj_config)
    dx, dy, dz = dx.to_numpy(), dy.to_numpy(), dz.to_numpy()

    a, b, c = track_df["pcov_xx"].to_numpy(), track_df["pcov_xy"].to_numpy(), track_df["pcov_xz"].to_numpy()
    d, e_, f = track_df["pcov_yy"].to_numpy(), track_df["pcov_yz"].to_numpy(), track_df["pcov_zz"].to_numpy()

    # Inversa analítica de matriz 3x3 simétrica [[a,b,c],[b,d,e_],[c,e_,f]] -
    # vetorizada (evita laço Python + np.linalg.inv por linha, caro em
    # trilhas de dezenas de milhares de pontos).
    det = a * (d * f - e_ ** 2) - b * (b * f - c * e_) + c * (b * e_ - c * d)
    with np.errstate(divide="ignore", invalid="ignore"):
        i00 = (d * f - e_ ** 2) / det
        i01 = (c * e_ - b * f) / det
        i02 = (b * e_ - c * d) / det
        i11 = (a * f - c ** 2) / det
        i12 = (b * c - a * e_) / det
        i22 = (a * d - b ** 2) / det

        nees = (dx * dx * i00 + dy * dy * i11 + dz * dz * i22
                + 2 * dx * dy * i01 + 2 * dx * dz * i02 + 2 * dy * dz * i12)

    nees = np.where(np.isfinite(det) & (np.abs(det) > 1e-300), nees, np.nan)
    return nees


def empirical_divergence_time(track_df: pd.DataFrame, traj_config: dict,
                               error_threshold_m: float = 50.0,
                               sustained_s: float = 2.0) -> float:
    """
    Critério de DIVERGÊNCIA EMPÍRICA, baseado no erro REAL (ground
    truth) - complementa (não substitui) estimate_convergence_time(),
    que só usa a covariância que o próprio filtro acredita ter. Ver
    revisão externa incorporada em achados_divergencia_fusao.md: um
    filtro pode estar divergindo com covariância pequena ao mesmo tempo
    (inconsistência/excesso de confiança) - a covariância sozinha não
    basta para detectar isso numa simulação onde o ground truth está
    disponível.

    Retorna o instante em que o erro real ultrapassa error_threshold_m e
    permanece acima por pelo menos sustained_s segundos contínuos (ou
    None se isso nunca acontece - execução considerada NÃO divergente
    empiricamente).

    error_threshold_m=50,0 e sustained_s=2,0 são ASSUNÇÃO DE PROJETO
    (decisão de engenharia, não valor de literatura): a trajetória mede
    ~100m de extensão e as estações ficam a 40-70m de distância dela; um
    erro sustentado acima de 50m é fisicamente "perder o alvo" nesse
    cenário específico, não ruído normal de medição. A confirmar/
    recalibrar com mais dados antes de travar em definitivo no artigo.
    """
    err = position_error(track_df, traj_config).to_numpy()
    t = track_df["timestamp"].to_numpy()
    order = np.argsort(t)
    t, err = t[order], err[order]

    above = err > error_threshold_m
    n = len(t)
    for i in range(n):
        if not above[i]:
            continue
        window_end = t[i] + sustained_s
        j = i
        while j < n and t[j] <= window_end and above[j]:
            j += 1
        if j == n or t[j] > window_end:
            return float(t[i])
    return None



def summarize_run_common_grid(track_df: pd.DataFrame, traj_config: dict,
                               scenario_id: int, sensor_mode: str,
                               repetition: int = 0, k: float = 3.0,
                               dwell_s: float = 1.0,
                               floor_fraction: float = 0.5,
                               max_floor_to_min_ratio: float = 2.0,
                               empirical_divergence_threshold_m: float = 50.0,
                               empirical_divergence_sustained_s: float = 2.0) -> dict:
    """
    Substitui summarize_run() para qualquer comparação ENTRE modos:
    combina a correção de grade temporal comum (item 1) com a separação
    de transiente/regime permanente (item 3), RMSE horizontal (xy) ao
    lado do 3D (revisão externa, item 4), e DOIS critérios de qualidade
    complementares, que NÃO devem ser confundidos:

    - 'covariance_stabilized' (antes chamado 'converged' - renomeado
      para deixar claro o que mede): a covariância que o PRÓPRIO FILTRO
      reporta estabilizou num piso baixo (estimate_convergence_time).
      Não usa o ground truth - é o que um sistema real, sem acesso à
      verdade, teria disponível para diagnosticar a si mesmo.
    - 'empirically_diverged': o erro REAL (ground truth, disponível
      aqui por ser simulação) ficou acima de um limiar por tempo
      sustentado (empirical_divergence_time) - detecta divergência
      mesmo quando a covariância do filtro fica pequena ao mesmo tempo
      (inconsistência/excesso de confiança - ver achados_divergencia_
      fusao.md §3, mecanismo da trava do gate). Ver revisão externa
      incorporada: a covariância sozinha NÃO basta para detectar
      divergência numa simulação onde a verdade é conhecida.

    Um filtro pode ter covariance_stabilized=True e empirically_diverged
    =True ao mesmo tempo - isso É o sintoma de inconsistência, não um
    erro de cálculo; ver também 'mean_nees_steady' (diagnóstico de
    consistência via NEES - Bar-Shalom, Li & Kirubarajan, 2001 - valor
    esperado 3,0 para um filtro consistente; >>3 indica covariância
    subestimada).

    Para o baseline geométrico (sem pos_uncertainty - memoryless), não
    há noção de convergência de filtro a aplicar: 'covariance_stabilized'
    vem True, 'mean_nees_steady' vem NaN (sem covariância de filtro a
    testar), e 'rmse_steady_m' é igual a 'rmse_full_m' (ver §6 do
    documento de critério). 'empirically_diverged' ainda se aplica
    normalmente (usa só o erro real, não depende de covariância).
    """
    has_filter_uncertainty = ("pos_uncertainty" in track_df.columns
                               and track_df["pos_uncertainty"].notna().any())

    t_conv = (estimate_convergence_time(track_df, k=k, dwell_s=dwell_s,
                                         floor_fraction=floor_fraction,
                                         max_floor_to_min_ratio=max_floor_to_min_ratio)
              if has_filter_uncertainty else None)
    covariance_stabilized = (t_conv is not None) if has_filter_uncertainty else True

    t_emp_div = empirical_divergence_time(
        track_df, traj_config,
        error_threshold_m=empirical_divergence_threshold_m,
        sustained_s=empirical_divergence_sustained_s,
    )
    empirically_diverged = t_emp_div is not None

    full = compute_rmse_common_grid(track_df, traj_config)
    steady = (compute_rmse_common_grid(track_df, traj_config, exclude_before_s=t_conv)
              if t_conv is not None else full)

    nees = compute_nees(track_df, traj_config)
    if t_conv is not None:
        steady_mask = track_df["timestamp"].to_numpy() >= t_conv
    else:
        steady_mask = np.ones(len(track_df), dtype=bool)
    nees_steady = nees[steady_mask]
    mean_nees_steady = float(np.nanmean(nees_steady)) if np.isfinite(nees_steady).any() else np.nan

    return {
        "scenario_id": scenario_id,
        "sensor_mode": sensor_mode,
        "repetition": repetition,
        "covariance_stabilized": covariance_stabilized,
        "t_conv_s": t_conv,
        "empirically_diverged": empirically_diverged,
        "t_empirical_divergence_s": t_emp_div,
        "mean_nees_steady": mean_nees_steady,
        "rmse_full_m": full["rmse_m"],
        "median_error_full_m": full["median_error_m"],
        "p95_error_full_m": full["p95_error_m"],
        "rmse_full_xy_m": full["rmse_xy_m"],
        "rmse_steady_m": steady["rmse_m"],
        "median_error_steady_m": steady["median_error_m"],
        "p95_error_steady_m": steady["p95_error_m"],
        "rmse_steady_xy_m": steady["rmse_xy_m"],
        "n_grid_points": full["n_grid_points"],
        "n_valid_points_steady": steady["n_valid_points"],
    }


def _coerce_bool(series: pd.Series) -> pd.Series:
    """
    Normaliza uma coluna booleana que pode ter passado por round-trip em
    CSV (pandas grava True/False como texto 'True'/'False', não bool -
    ao reler, a coluna vira object/string, e um .sum() sobre strings não
    conta convergências corretamente). Usado por divergence_rate() e
    aggregate_runs_common_grid(), que precisam somar 'covariance_stabilized'/'empirically_diverged' tanto a
    partir de summarize_run_common_grid() em memória (bool nativo) quanto
    recarregado de partial_results/*.csv (string) - ver run_all_parallel.py.
    """
    return series.apply(lambda v: v if isinstance(v, (bool, np.bool_)) else str(v).strip() == "True")


def divergence_rate(run_summaries: list) -> pd.DataFrame:
    """
    Taxa de divergência por cenário x modo - DUAS versões lado a lado,
    que medem coisas diferentes (ver docstring de
    summarize_run_common_grid, e revisão externa incorporada em
    achados_divergencia_fusao.md):

    - 'divergence_rate_pct' (covariância): fração de repetições em que
      a covariância do PRÓPRIO FILTRO nunca estabilizou
      (covariance_stabilized=False) - é o que um sistema real, sem
      acesso ao ground truth, conseguiria diagnosticar sozinho.
    - 'empirical_divergence_rate_pct' (erro real): fração de repetições
      em que o erro REAL ficou acima do limiar por tempo sustentado
      (empirically_diverged=True) - só calculável em simulação, onde a
      verdade é conhecida. Pode ser MAIOR que a taxa de covariância
      (filtro "confiante" mas errado - inconsistência) ou MENOR
      (filtro "inseguro" mas na verdade próximo do alvo).

    Espera uma lista de dicts no formato retornado por
    summarize_run_common_grid().
    """
    df = pd.DataFrame(run_summaries)
    df["covariance_stabilized"] = _coerce_bool(df["covariance_stabilized"])
    df["empirically_diverged"] = _coerce_bool(df["empirically_diverged"])
    out = df.groupby(["scenario_id", "sensor_mode"]).agg(
        n_repetitions=("repetition", "count"),
        n_covariance_stabilized=("covariance_stabilized", "sum"),
        n_empirically_diverged=("empirically_diverged", "sum"),
    ).reset_index()
    out["divergence_rate_pct"] = 100.0 * (1.0 - out["n_covariance_stabilized"] / out["n_repetitions"])
    out["empirical_divergence_rate_pct"] = 100.0 * (out["n_empirically_diverged"] / out["n_repetitions"])
    return out


def aggregate_runs_common_grid(run_summaries: list):
    """
    Agrega resumos gerados por summarize_run_common_grid() (grade
    temporal comum + separação de regime permanente - itens 1 e 3 da
    correção do protocolo de avaliação) em uma tabela por cenário x modo.

    SUBSTITUI aggregate_runs() para qualquer pipeline que já usa
    summarize_run_common_grid() em vez de summarize_run() - é o caso de
    run_repetition() em orchestrator.py (usado tanto pela execução
    sequencial quanto por run_all_parallel.py). aggregate_runs() é
    mantida sem alterações, para não quebrar código legado que ainda
    produza o esquema antigo.

    Reporta rmse_full (sobre a grade comum completa, incluindo
    transiente) e rmse_steady (regime permanente, após t_conv - igual
    ao full quando não há convergência a excluir, ex.: RF-only e
    baseline) lado a lado, para que a diferença entre os dois fique
    visível na própria tabela, em vez de escondida.

    Retorna
    -------
    (aggregated, raw, divergence) - as três tabelas: agregada por
    cenário x modo (inclui divergence_rate_pct já mesclada), raw (uma
    linha por execução) e a tabela de divergência isolada (útil para
    reportar separadamente na seção de Resultados).
    """
    df = pd.DataFrame(run_summaries)
    df["covariance_stabilized"] = _coerce_bool(df["covariance_stabilized"])
    df["empirically_diverged"] = _coerce_bool(df["empirically_diverged"])

    agg = df.groupby(["scenario_id", "sensor_mode"]).agg(
        rmse_full_mean=("rmse_full_m", "mean"),
        rmse_full_std=("rmse_full_m", "std"),
        rmse_steady_mean=("rmse_steady_m", "mean"),
        rmse_steady_std=("rmse_steady_m", "std"),
        rmse_steady_xy_mean=("rmse_steady_xy_m", "mean"),
        rmse_steady_xy_std=("rmse_steady_xy_m", "std"),
        median_error_steady_mean=("median_error_steady_m", "mean"),
        mean_nees_steady_mean=("mean_nees_steady", "mean"),
        n_repetitions=("repetition", "count"),
    ).reset_index()

    div = divergence_rate(run_summaries)
    agg = agg.merge(
        div[["scenario_id", "sensor_mode", "divergence_rate_pct", "empirical_divergence_rate_pct"]],
        on=["scenario_id", "sensor_mode"], how="left",
    )
    return agg, df, div


def _holm_bonferroni(p_values: list) -> list:
    """
    Correção de Holm-Bonferroni (Holm, S., 1979, "A simple sequentially
    rejective multiple test procedure", Scandinavian Journal of
    Statistics) para múltiplas comparações - menos conservadora que
    Bonferroni simples, mantendo controle da taxa de erro familywise.
    Retorna os p-valores ajustados, na MESMA ordem da entrada.
    """
    n = len(p_values)
    order = np.argsort(p_values)
    adjusted = np.empty(n)
    running_max = 0.0
    for rank, idx in enumerate(order):
        val = min(1.0, (n - rank) * p_values[idx])
        running_max = max(running_max, val)
        adjusted[idx] = running_max
    return adjusted.tolist()


def _rank_biserial_wilcoxon(x: np.ndarray, y: np.ndarray) -> float:
    """
    Tamanho de efeito para o teste de Wilcoxon pareado (correlação
    rank-biserial pareada): r = (n_pos - n_neg) / n_total, onde n_pos/
    n_neg contam quantas diferenças (x-y) são positivas/negativas
    (empates descartados). Varia de -1 a 1; |r| pequeno~0,1, médio~0,3,
    grande~0,5 (convenção de Cohen, 1988, adaptada a estatística de
    postos).
    """
    diff = np.asarray(x) - np.asarray(y)
    diff = diff[diff != 0]
    if len(diff) == 0:
        return 0.0
    n_pos = int(np.sum(diff > 0))
    n_neg = int(np.sum(diff < 0))
    return (n_pos - n_neg) / len(diff)


def compare_scenarios_statistical(rmse_by_scenario: dict) -> dict:
    """
    Compara o RMSE entre os cenários (cada chave do dict = scenario_id,
    cada valor = lista de RMSEs de repetições Monte Carlo - AMOSTRAS
    INDEPENDENTES entre cenários, já que cada (cenário, repetição) usa
    uma semente de ruído própria - ver run_repetition() em
    orchestrator.py) via ANOVA (se aproximadamente normal) ou
    Kruskal-Wallis (não-paramétrico, mais seguro com poucas repetições/
    distribuição desconhecida).

    Teste OMNIBUS (Kruskal-Wallis) testa só "existe alguma diferença
    entre os cenários" - não diz QUAIS pares diferem, nem testa uma
    hipótese direcional específica (ex.: H1 prevê circular > linear).
    Por isso, quando o omnibus é significativo (p<0,05), roda também
    POST-HOC pareado a pareado (Mann-Whitney U - apropriado aqui porque
    as amostras são independentes entre cenários, não pareadas) com
    correção de Holm-Bonferroni para múltiplas comparações, e tamanho
    de efeito (r rank-biserial de Mann-Whitney) - revisão externa
    incorporada (ver achados_divergencia_fusao.md).

    Retorna
    -------
    dict com o teste omnibus usado, estatística e p-valor, mais
    'post_hoc' (lista de dicts por par de cenários, só presente se o
    omnibus for significativo).
    """
    groups = list(rmse_by_scenario.values())
    labels = list(rmse_by_scenario.keys())
    if any(len(g) < 2 for g in groups):
        return {"test": None, "statistic": np.nan, "p_value": np.nan,
                "note": "Repetições insuficientes por cenário (mínimo 2) para teste estatístico."}

    stat, p = stats.kruskal(*groups)
    result = {"test": "Kruskal-Wallis", "statistic": float(stat), "p_value": float(p)}

    if p < 0.05:
        pairs = [(i, j) for i in range(len(labels)) for j in range(i + 1, len(labels))]
        raw_p = []
        pair_stats = []
        for i, j in pairs:
            a, b = np.asarray(groups[i]), np.asarray(groups[j])
            u_stat, u_p = stats.mannwhitneyu(a, b, alternative="two-sided")
            # r rank-biserial de Mann-Whitney: r = 1 - 2U/(n1*n2)
            r_rb = 1.0 - (2.0 * u_stat) / (len(a) * len(b))
            raw_p.append(u_p)
            pair_stats.append({"scenario_a": labels[i], "scenario_b": labels[j],
                                "u_statistic": float(u_stat), "p_value_raw": float(u_p),
                                "effect_size_rank_biserial": float(r_rb)})
        adjusted = _holm_bonferroni(raw_p)
        for ps, p_adj in zip(pair_stats, adjusted):
            ps["p_value_holm_bonferroni"] = float(p_adj)
            ps["significant_after_correction"] = bool(p_adj < 0.05)
        result["post_hoc"] = pair_stats
        result["post_hoc_method"] = "Mann-Whitney U pareado (amostras independentes), correção Holm-Bonferroni"

    return result


def compare_methods_paired(values_by_method: dict) -> dict:
    """
    Compara múltiplos métodos/arquiteturas (fusão, câmera-only,
    track-to-track etc.) DENTRO do mesmo cenário, usando o teste de
    Friedman (Friedman, M., 1937, "The use of ranks to avoid the
    assumption of normality implicit in the analysis of variance",
    Journal of the American Statistical Association) - apropriado
    porque os métodos são comparados PAREADOS pela mesma repetição
    (mesmas detecções RF/câmera - ver run_repetition() em
    orchestrator.py, que roda todos os modos sobre a MESMA geração de
    ruído dentro de uma repetição), não amostras independentes.

    Revisão externa incorporada (ver achados_divergencia_fusao.md):
    "para comparar métodos dentro da mesma repetição, os dados são
    pareados... use Friedman seguido de Wilcoxon pareado".

    values_by_method : dict {nome_do_metodo: [valores por repetição]} -
        todas as listas devem ter o MESMO comprimento e estar na MESMA
        ordem de repetição (values_by_method['fusion'][k] e
        values_by_method['camera_only'][k] devem vir da repetição k).

    Retorna
    -------
    dict com o teste omnibus (Friedman) e, se significativo (p<0,05),
    'post_hoc': lista de comparações par a par (Wilcoxon pareado),
    com p-valor bruto, corrigido (Holm-Bonferroni) e tamanho de efeito
    (correlação rank-biserial pareada).
    """
    labels = list(values_by_method.keys())
    groups = [np.asarray(values_by_method[m]) for m in labels]
    lengths = {len(g) for g in groups}
    if len(lengths) != 1:
        return {"test": None, "statistic": np.nan, "p_value": np.nan,
                "note": "Os métodos não têm o mesmo número de repetições - "
                        "não é possível parear (Friedman exige dados balanceados)."}
    n = lengths.pop()
    if n < 2 or len(labels) < 3:
        return {"test": None, "statistic": np.nan, "p_value": np.nan,
                "note": "Friedman exige >=3 métodos e >=2 repetições pareadas."}

    stat, p = stats.friedmanchisquare(*groups)
    result = {"test": "Friedman", "statistic": float(stat), "p_value": float(p), "n_repetitions": n}

    if p < 0.05:
        pairs = [(i, j) for i in range(len(labels)) for j in range(i + 1, len(labels))]
        raw_p = []
        pair_stats = []
        for i, j in pairs:
            a, b = groups[i], groups[j]
            try:
                w_stat, w_p = stats.wilcoxon(a, b)
            except ValueError:
                # todas as diferenças zero, ou amostra pequena demais
                w_stat, w_p = np.nan, 1.0
            r_rb = _rank_biserial_wilcoxon(a, b)
            raw_p.append(w_p)
            pair_stats.append({"method_a": labels[i], "method_b": labels[j],
                                "w_statistic": float(w_stat) if np.isfinite(w_stat) else None,
                                "p_value_raw": float(w_p),
                                "effect_size_rank_biserial": float(r_rb),
                                "median_a": float(np.median(a)), "median_b": float(np.median(b))})
        adjusted = _holm_bonferroni(raw_p)
        for ps, p_adj in zip(pair_stats, adjusted):
            ps["p_value_holm_bonferroni"] = float(p_adj)
            ps["significant_after_correction"] = bool(p_adj < 0.05)
        result["post_hoc"] = pair_stats
        result["post_hoc_method"] = "Wilcoxon pareado (mesmas repetições/detecções), correção Holm-Bonferroni"

    return result


def summarize_gdop_correlations(pearson_r_values: list) -> dict:
    """
    Resume a distribuição de correlações GDOP x erro ATRAVÉS das
    repetições (cada repetição já contribui com UM r, calculado por
    correlate_gdop_rmse sobre os 20 bins temporais daquela repetição) -
    substitui "contar quantas das N repetições tiveram p<0,05 dentro
    da própria repetição", que a revisão externa apontou não ser um
    teste agregado válido de H3 (ver achados_divergencia_fusao.md).

    DUAS RESSALVAS DE AUTOCORRELAÇÃO IMPORTANTES (revisão externa
    incorporada), que se aplicam em NÍVEIS DIFERENTES:
    (a) Os 20 bins temporais DENTRO de uma única repetição são
        autocorrelacionados (o erro e o GDOP em t e t+1s não são
        observações independentes) - isso infla artificialmente a
        significância (p-valor) de CADA correlação individual
        calculada por correlate_gdop_rmse. Não corrigido aqui.
    (b) Já as N repetições ENTRE SI são independentes (cada uma usa uma
        semente de ruído própria - ver run_repetition() em
        orchestrator.py) - então agregar o r de cada repetição num
        conjunto de N valores independentes, como esta função faz, é
        estatisticamente válido, mesmo com a ressalva (a) ainda valendo
        para cada r individual.

    Usa transformação de Fisher (arctanh) para calcular a média de
    forma estatisticamente correta (r não é aditivo, mas z=arctanh(r) é
    aproximadamente normal) - Fisher, R.A. (1915). Também roda um teste
    de Wilcoxon de uma amostra (não-paramétrico) dos r-valores contra
    zero, como teste agregado de H3 (GDOP prediz erro) através das
    repetições.

    Retorna
    -------
    dict com: mean_r (média simples, para leitura rápida), mean_r_fisher
    (média via transformação de Fisher - preferível estatisticamente),
    median_r, std_r, n, wilcoxon_statistic, wilcoxon_p_value (H0: a
    distribuição de r é simétrica em torno de zero).
    """
    r = np.asarray([v for v in pearson_r_values if np.isfinite(v)])
    n = len(r)
    if n < 2:
        return {"mean_r": np.nan, "mean_r_fisher": np.nan, "median_r": np.nan,
                "std_r": np.nan, "n": n, "wilcoxon_statistic": np.nan, "wilcoxon_p_value": np.nan,
                "note": "Menos de 2 valores de r válidos - resumo não calculável."}

    z = np.arctanh(np.clip(r, -0.999999, 0.999999))
    mean_r_fisher = float(np.tanh(np.mean(z)))

    try:
        w_stat, w_p = stats.wilcoxon(r)
    except ValueError:
        w_stat, w_p = np.nan, np.nan

    return {
        "mean_r": float(np.mean(r)),
        "mean_r_fisher": mean_r_fisher,
        "median_r": float(np.median(r)),
        "std_r": float(np.std(r, ddof=1)) if n > 1 else np.nan,
        "n": n,
        "wilcoxon_statistic": float(w_stat) if np.isfinite(w_stat) else None,
        "wilcoxon_p_value": float(w_p) if np.isfinite(w_p) else None,
    }


def fusion_gain_summary(rmse_fusion: float, rmse_rf_only: float, rmse_camera_only: float,
                         rmse_baseline: float = None) -> dict:
    """Resume o ganho da fusão sobre cada sensor isolado (e sobre o baseline, se fornecido)."""
    out = {
        "rmse_fusion_m": rmse_fusion,
        "rmse_rf_only_m": rmse_rf_only,
        "rmse_camera_only_m": rmse_camera_only,
        "gain_vs_rf_only_m": rmse_rf_only - rmse_fusion,
        "gain_vs_rf_only_pct": (rmse_rf_only - rmse_fusion) / rmse_rf_only * 100,
        "gain_vs_camera_only_m": rmse_camera_only - rmse_fusion,
        "gain_vs_camera_only_pct": (rmse_camera_only - rmse_fusion) / rmse_camera_only * 100,
    }
    if rmse_baseline is not None:
        out["rmse_baseline_m"] = rmse_baseline
        out["gain_temporal_filtering_m"] = rmse_baseline - rmse_fusion
        out["gain_multisensor_m"] = min(rmse_rf_only, rmse_camera_only) - rmse_baseline
    return out


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

    fusion = run_fusion_ekf(merged, stations, FILTER_CONFIG, "fusion")
    rf_only = run_fusion_ekf(merged, stations, FILTER_CONFIG, "rf_only")
    cam_only = run_fusion_ekf(merged, stations, FILTER_CONFIG, "camera_only")
    baseline = run_baseline(merged, stations)

    print("--- Resumo de execução (fusão) ---")
    print(summarize_run(fusion, TRAJECTORY, scenario_id, "fusion"))

    print("\n--- Ganho da fusão ---")
    gain = fusion_gain_summary(
        compute_rmse(fusion, TRAJECTORY), compute_rmse(rf_only, TRAJECTORY),
        compute_rmse(cam_only, TRAJECTORY), compute_rmse(baseline, TRAJECTORY),
    )
    for k, v in gain.items():
        print(f"  {k}: {v:.3f}" if isinstance(v, float) else f"  {k}: {v}")

    print("\n--- Validação cruzada GDOP x RMSE empírico (fusão) ---")
    corr = correlate_gdop_rmse(fusion, TRAJECTORY, stations, scenario_id)
    print(f"  Correlação de Pearson (GDOP vs. erro binado): r={corr['pearson_r']:.3f}, "
          f"p={corr['p_value']:.4f}")

    print("\n--- Teste estatístico entre cenários (exemplo com dados simulados) ---")
    fake_rmse = {1: [0.84, 0.90, 0.79], 2: [0.65, 0.70, 0.68], 3: [0.75, 0.80, 0.77]}
    test = compare_scenarios_statistical(fake_rmse)
    print(f"  {test}")

    print("\n--- Grade temporal comum + regime permanente (correção itens 1 e 3) ---")
    for name, track in [("fusion", fusion), ("rf_only", rf_only),
                         ("camera_only", cam_only), ("baseline", baseline)]:
        s = summarize_run_common_grid(track, TRAJECTORY, scenario_id, name)
        tconv = f"{s['t_conv_s']:.2f}s" if s["t_conv_s"] is not None else "N/A"
        print(f"  {name:12s} covariancia_estabilizada={s['covariance_stabilized']!s:5s} "
              f"divergiu_empiricamente={s['empirically_diverged']!s:5s} t_conv={tconv:>7s} | "
              f"RMSE completo={s['rmse_full_m']:7.3f}  "
              f"RMSE regime permanente={s['rmse_steady_m']:7.3f}")

    print("\n--- Taxa de divergência (exemplo com 1 repetição só - "
          "rodar com N=30 para o número real) ---")
    summaries = [summarize_run_common_grid(t, TRAJECTORY, scenario_id, n)
                 for n, t in [("fusion", fusion), ("rf_only", rf_only),
                              ("camera_only", cam_only), ("baseline", baseline)]]
    print(divergence_rate(summaries).to_string(index=False))

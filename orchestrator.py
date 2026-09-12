"""
Módulo 11 - Orchestrator

Executa a matriz completa de experimentos - 3 cenários geométricos x
modos/arquiteturas de fusão x N repetições Monte Carlo (variando a
semente de ruído), e agrega os resultados usando os Módulos 9
(exportação) e 10 (métricas).

ARQUITETURAS COMPARADAS:
  - "fusion"/"rf_only"/"camera_only": um único EKF processando as
    detecções de TODAS as estações em sequência (fusão por medição
    sequencial - ekf_fusion.py).
  - "track_to_track_ci": um EKF LOCAL por estação (só com as detecções
    daquela estação) + um combinador GLOBAL por Interseção de
    Covariância (Julier & Uhlmann, 1997) - track_to_track_fusion.py.
  - "baseline_geometric": mínimos quadrados ponderados sem filtragem
    temporal (baseline_estimator.py) - sem estado, sem "arquitetura" de
    fusão sequencial, mantido como referência de comparação.

"""

import os
import time
import numpy as np
import pandas as pd

from config import TRAJECTORY, STATIONS, RF, CAMERA, FILTER_CONFIG, EXPERIMENT
from station_layout_generator import generate_stations
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections
from ekf_fusion import run_fusion_ekf
from fusion_models import thin_rf_detections
from track_to_track_fusion import run_track_to_track_fusion
from baseline_estimator import run_baseline
from telemetry_exporter import export_telemetry
from metrics_evaluator import (
    summarize_run_common_grid, aggregate_runs_common_grid, divergence_rate,
    correlate_gdop_rmse, compare_scenarios_statistical, compare_methods_paired,
    summarize_gdop_correlations, fusion_gain_summary,
)

SCENARIOS = (1, 2, 3)
MODES = ("fusion", "rf_only", "camera_only")


def run_repetition(scenario_id: int, stations_df: pd.DataFrame, repetition: int,
                    rf_decimation_factor: int, save_telemetry: bool, output_dir: str,
                    rf_config_overrides: dict = None, camera_config_overrides: dict = None,
                    noise_reference_stations_df: pd.DataFrame = None):
    """
    Executa uma repetição - todos os modos do EKF global (fusion/rf_only/
    camera_only), a fusão track-to-track (CI) e o baseline geométrico -
    para um cenário.

    rf_config_overrides, camera_config_overrides : dicts opcionais
        mesclados por cima da config RF/CAMERA padrão (após a semente por
        repetição já ter sido aplicada) - usado por
        geometry_noise_experiment.py para escalar o ruído dos sensores
        (`noise_scale`) independentemente da geometria das estações, sem
        alterar o comportamento padrão (None = nenhuma mudança).

    noise_reference_stations_df : DataFrame opcional - se fornecido, o
        RUÍDO dos sensores é calculado usando a distância a ESTE layout
        de referência, não a `stations_df` (usado para a geometria/
        medição real) - isola o efeito puro de geometria, congelando o
        nível de ruído numa distância de referência fixa mesmo quando
        `stations_df` foi escalado/movido (ver rf_sensor_model.py::
        generate_rf_detections e achados_divergencia_fusao.md §14 -
        correção da revisão externa: antes, `noise_scale` sozinho não
        bastava para isolar geometria, porque o ruído de base continuava
        dependendo da distância atual das estações).

    CORREÇÃO DE PROTOCOLO (itens 1, 3 e 4 - ver achados_divergencia_fusao.md):
    os resumos de métrica vêm de summarize_run_common_grid() (grade
    temporal comum + separação de regime permanente -
    'rmse_full_m'/'rmse_steady_m'/'covariance_stabilized'/'empirically_diverged'/'t_conv_s'), e a correlação
    GDOP x erro (item 4) é calculada PARA CADA REPETIÇÃO, reaproveitando
    as trilhas já geradas aqui (sem regenerar detecções RF/câmera).

    Usado tanto pela execução sequencial (run_experiment, abaixo) quanto
    por run_all_parallel.py e run_single_repetition.py, que importam
    esta função diretamente - corrigir aqui corrige as três vias de
    execução de uma vez.

    Retorna
    -------
    (summaries, gdop_results) : summaries é a lista de resumos de
    métrica (um por modo/arquitetura); gdop_results é uma LISTA de
    dicts leves {'scenario_id', 'repetition', 'sensor_mode', 'pearson_r',
    'p_value'} - uma linha para "fusion" (EKF global) e outra para
    "track_to_track_ci".
    """
    rf_cfg = dict(RF)
    rf_cfg["seed"] = RF["seed"] + repetition * 1000 + scenario_id
    rf_cfg.update(rf_config_overrides or {})
    cam_cfg = dict(CAMERA)
    cam_cfg["seed"] = CAMERA["seed"] + repetition * 1000 + scenario_id
    cam_cfg.update(camera_config_overrides or {})

    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations_df, rf_cfg,
                                     noise_reference_stations_df=noise_reference_stations_df)
    rf_det = thin_rf_detections(rf_det, rf_decimation_factor)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations_df, cam_cfg,
                                          noise_reference_stations_df=noise_reference_stations_df)
    merged = associate_detections(rf_det, cam_det)

    summaries = []
    fusion_track = None

    # --- Arquitetura 1: EKF global (fusão por medição sequencial) ---
    for mode in MODES:
        track = run_fusion_ekf(merged, stations_df, FILTER_CONFIG, mode)
        if mode == "fusion":
            fusion_track = track
        summaries.append(summarize_run_common_grid(track, TRAJECTORY, scenario_id, mode, repetition))
        if save_telemetry and repetition == 0:
            export_telemetry(
                track, TRAJECTORY,
                os.path.join(output_dir, f"telemetry_scenario{scenario_id}_{mode}"),
                fmt="csv",
            )

    # --- Arquitetura 2: track-to-track (EKF local por estação + CI global) ---
    t2t_track, _local_tracks = run_track_to_track_fusion(
        scenario_id, repetition, stations_df=stations_df, merged=merged,
    )
    summaries.append(summarize_run_common_grid(
        t2t_track, TRAJECTORY, scenario_id, "track_to_track_ci", repetition))
    if save_telemetry and repetition == 0:
        export_telemetry(
            t2t_track, TRAJECTORY,
            os.path.join(output_dir, f"telemetry_scenario{scenario_id}_track_to_track_ci"),
            fmt="csv",
        )

    # --- Baseline geométrico (sem filtragem temporal) ---
    baseline = run_baseline(merged, stations_df)
    summaries.append(summarize_run_common_grid(baseline, TRAJECTORY, scenario_id, "baseline_geometric", repetition))
    if save_telemetry and repetition == 0:
        export_telemetry(
            baseline, TRAJECTORY,
            os.path.join(output_dir, f"telemetry_scenario{scenario_id}_baseline"),
            fmt="csv",
        )

    # --- Correlação GDOP x erro (item 4) - fusion (EKF global) e track_to_track_ci ---
    gdop_results = []
    for label, track in (("fusion", fusion_track), ("track_to_track_ci", t2t_track)):
        gdop_corr = correlate_gdop_rmse(track, TRAJECTORY, stations_df, scenario_id)
        gdop_results.append({
            "scenario_id": scenario_id,
            "repetition": repetition,
            "sensor_mode": label,
            "pearson_r": gdop_corr["pearson_r"],
            "p_value": gdop_corr["p_value"],
        })

    return summaries, gdop_results


def run_experiment(n_repetitions: int = None, rf_decimation_factor: int = None,
                    output_dir: str = None) -> dict:
    """
    Executa a matriz completa do experimento.

    Retorna
    -------
    dict com: 'raw' (DataFrame de todas as execuções individuais, já no
    esquema de grade comum), 'aggregated' (DataFrame agregado por
    cenário x modo, com rmse_full e rmse_steady lado a lado),
    'divergence' (taxa de divergência por cenário x modo - item 5),
    'gdop_correlations' (DataFrame, uma linha por cenário x repetição -
    item 4), 'statistical_test' (dict).
    """
    n_repetitions = n_repetitions or EXPERIMENT["n_monte_carlo_repetitions"]
    rf_decimation_factor = (rf_decimation_factor
                             if rf_decimation_factor is not None
                             else EXPERIMENT["rf_decimation_factor"])
    output_dir = output_dir or EXPERIMENT["output_dir"]
    os.makedirs(output_dir, exist_ok=True)

    all_summaries = []
    gdop_rows = []

    t_start = time.time()
    for scenario_id in SCENARIOS:
        stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)
        t_scenario_start = time.time()

        for repetition in range(n_repetitions):
            summaries, gdop_results = run_repetition(
                scenario_id, stations_df, repetition, rf_decimation_factor,
                save_telemetry=True, output_dir=output_dir,
            )
            all_summaries.extend(summaries)
            gdop_rows.extend(gdop_results)

        elapsed = time.time() - t_scenario_start
        print(f"Cenário {scenario_id}: {n_repetitions} repetições concluídas em {elapsed:.1f}s")

    total_elapsed = time.time() - t_start
    print(f"\nTempo total do experimento: {total_elapsed:.1f}s")

    aggregated, raw, divergence = aggregate_runs_common_grid(all_summaries)
    gdop_correlations = pd.DataFrame(gdop_rows)

    # Comparação estatística entre cenários, modo fusão (EKF global,
    # métrica principal). Usa rmse_steady_m (regime permanente), não
    # rmse_full_m - comparar cenários pelo RMSE bruto misturaria a
    # duração/severidade do transiente de cada cenário com o efeito de
    # geometria (GDOP) que a hipótese realmente quer testar.
    rmse_by_scenario = {
        sc: raw[(raw.scenario_id == sc) & (raw.sensor_mode == "fusion")].rmse_steady_m.tolist()
        for sc in SCENARIOS
    }
    statistical_test = compare_scenarios_statistical(rmse_by_scenario)

    # Comparação PAREADA entre métodos/arquiteturas, dentro de cada
    # cenário - os métodos usam a MESMA geração de ruído por repetição
    # (ver run_repetition), então são amostras pareadas, não
    # independentes (revisão externa incorporada - ver
    # achados_divergencia_fusao.md). Compara os 4 métodos com filtro
    # (RF-only incluído mesmo sendo estruturalmente ruim - Friedman não
    # exige que os métodos sejam bons, só pareados e balanceados).
    method_comparison_by_scenario = {}
    for sc in SCENARIOS:
        raw_sc = raw[raw.scenario_id == sc]
        values_by_method = {}
        ok = True
        for mode in ("fusion", "camera_only", "rf_only", "track_to_track_ci"):
            vals = raw_sc[raw_sc.sensor_mode == mode].sort_values("repetition").rmse_steady_m.tolist()
            if not vals:
                ok = False
                break
            values_by_method[mode] = vals
        method_comparison_by_scenario[sc] = (
            compare_methods_paired(values_by_method) if ok else
            {"test": None, "note": "Algum modo ausente neste cenário - comparação pulada."}
        )

    aggregated.to_csv(os.path.join(output_dir, "aggregated_metrics.csv"), index=False)
    raw.to_csv(os.path.join(output_dir, "raw_run_summaries.csv"), index=False)
    divergence.to_csv(os.path.join(output_dir, "divergence_rates.csv"), index=False)
    gdop_correlations.to_csv(os.path.join(output_dir, "gdop_correlations.csv"), index=False)

    return {
        "raw": raw, "aggregated": aggregated, "divergence": divergence,
        "gdop_correlations": gdop_correlations,
        "statistical_test": statistical_test,
        "method_comparison_by_scenario": method_comparison_by_scenario,
        "rf_decimation_factor": rf_decimation_factor,
        "n_repetitions": n_repetitions,
    }


if __name__ == "__main__":
    results = run_experiment()

    print("\n=== RESULTADOS AGREGADOS (RMSE médio ± desvio-padrão, por cenário x modo) ===")
    print("    (rmse_full = grade comum incl. transiente | rmse_steady = regime permanente)")
    for _, row in results["aggregated"].iterrows():
        print(f"  Cenário {row.scenario_id} | {row.sensor_mode:20s} | "
              f"full = {row.rmse_full_mean:.3f} ± {row.rmse_full_std:.3f} m | "
              f"steady = {row.rmse_steady_mean:.3f} ± {row.rmse_steady_std:.3f} m | "
              f"n={row.n_repetitions} | divergência(covariância)={row.divergence_rate_pct:.1f}% | "
              f"divergência(erro real)={row.empirical_divergence_rate_pct:.1f}% | "
              f"NEES médio={row.mean_nees_steady_mean:.2f} (esperado≈3,0)")

    print("\n=== VALIDAÇÃO CRUZADA GDOP x RMSE, por cenário x arquitetura (agregado entre repetições) ===")
    print("    (ver docstring de summarize_gdop_correlations para as ressalvas de autocorrelação)")
    gdop_by_scenario = results["gdop_correlations"].groupby(["scenario_id", "sensor_mode"])
    for (sc, mode), g in gdop_by_scenario:
        s = summarize_gdop_correlations(g.pearson_r.tolist())
        print(f"  Cenário {sc} | {mode:20s} | r médio (Fisher)={s['mean_r_fisher']:.3f} "
              f"(dp={s['std_r']:.3f}, n={s['n']}) | Wilcoxon (r≠0) p={s['wilcoxon_p_value']:.4f}")

    print("\n=== TESTE ESTATÍSTICO ENTRE CENÁRIOS (EKF global, RMSE de regime permanente) ===")
    print(f"  {results['statistical_test']}")

    print("\n=== COMPARAÇÃO PAREADA ENTRE MÉTODOS, POR CENÁRIO (Friedman + Wilcoxon pareado) ===")
    for sc, comp in results["method_comparison_by_scenario"].items():
        print(f"  Cenário {sc}: {comp.get('test')} statistic={comp.get('statistic')} "
              f"p={comp.get('p_value')}")
        for ph in comp.get("post_hoc", []):
            sig = "* " if ph["significant_after_correction"] else "  "
            print(f"    {sig}{ph['method_a']:20s} vs {ph['method_b']:20s} | "
                  f"p_holm={ph['p_value_holm_bonferroni']:.4f} | r={ph['effect_size_rank_biserial']:+.2f}")

    import json
    with open(os.path.join(EXPERIMENT["output_dir"], "method_comparison_by_scenario.json"), "w") as f:
        json.dump(results["method_comparison_by_scenario"], f, indent=2, default=str)

    print(f"\nArquivos salvos em: {EXPERIMENT['output_dir']}")

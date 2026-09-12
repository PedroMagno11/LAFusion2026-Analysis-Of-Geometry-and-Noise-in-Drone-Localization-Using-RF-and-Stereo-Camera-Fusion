"""
Roda todas as combinações (cenário, repetição) do experimento em paralelo,
usando multiprocessing nativo do Python - funciona igual em Windows, Mac e
Linux.

Uso:
    python run_all_parallel.py <n_repeticoes> <n_processos>

Exemplo (30 repetições, 4 processos simultâneos):
    python run_all_parallel.py 30 4

Cada combinação salva seu próprio arquivo em
<output_dir>/partial_results/scenario{X}_rep{Y}.csv - seguro para rodar em
paralelo, sem condição de corrida (cada processo escreve só no seu próprio
arquivo). Ao final, agrega tudo automaticamente.
"""

import os
import sys
import glob
import multiprocessing as mp

import pandas as pd

from config import TRAJECTORY, STATIONS, EXPERIMENT
from station_layout_generator import generate_stations
from orchestrator import run_repetition
from metrics_evaluator import (
    aggregate_runs_common_grid, compare_scenarios_statistical,
    compare_methods_paired, summarize_gdop_correlations,
)

PARTIAL_DIR = os.path.join(EXPERIMENT["output_dir"], "partial_results")


def _run_one(args):
    """Executa uma combinação (cenário, repetição) - roda em um processo separado."""
    scenario_id, repetition = args
    stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)

    summaries, gdop_results = run_repetition(
        scenario_id, stations_df, repetition,
        rf_decimation_factor=EXPERIMENT["rf_decimation_factor"],
        save_telemetry=(repetition == 0),
        output_dir=EXPERIMENT["output_dir"],
    )

    out_path = os.path.join(PARTIAL_DIR, f"scenario{scenario_id}_rep{repetition}.csv")
    pd.DataFrame(summaries).to_csv(out_path, index=False)

    gdop_path = os.path.join(PARTIAL_DIR, f"gdop_scenario{scenario_id}_rep{repetition}.csv")
    pd.DataFrame(gdop_results).to_csv(gdop_path, index=False)

    print(f"OK: cenario {scenario_id}, repeticao {repetition} -> {out_path}", flush=True)
    return out_path


def aggregate_and_report():
    """
    Agrega os arquivos parciais (partial_results/*.csv, um por
    combinação cenário x repetição, gerados por _run_one via
    run_repetition() em orchestrator.py - já no esquema de grade
    temporal comum, itens 1 e 3 da correção do protocolo), mais os
    arquivos de correlação GDOP (item 4, um por combinação também,
    prefixo 'gdop_' - mantidos em arquivos separados dos de métrica
    para não misturar dois esquemas de coluna diferentes no mesmo glob).
    """
    metric_files = glob.glob(os.path.join(PARTIAL_DIR, "scenario*_rep*.csv"))
    gdop_files = glob.glob(os.path.join(PARTIAL_DIR, "gdop_scenario*_rep*.csv"))
    print(f"\nArquivos parciais de métrica encontrados: {len(metric_files)}")
    print(f"Arquivos parciais de correlação GDOP encontrados: {len(gdop_files)}")
    if not metric_files:
        print("Nenhum resultado encontrado - nada a agregar.")
        return

    raw = pd.concat([pd.read_csv(f) for f in metric_files], ignore_index=True)
    aggregated, raw_df, divergence = aggregate_runs_common_grid(raw.to_dict("records"))

    aggregated.to_csv(os.path.join(EXPERIMENT["output_dir"], "aggregated_metrics.csv"), index=False)
    raw_df.to_csv(os.path.join(EXPERIMENT["output_dir"], "raw_run_summaries.csv"), index=False)
    divergence.to_csv(os.path.join(EXPERIMENT["output_dir"], "divergence_rates.csv"), index=False)

    print("\n=== RESULTADOS AGREGADOS ===")
    print("    (rmse_full = grade comum incl. transiente | rmse_steady = regime permanente)")
    for _, row in aggregated.sort_values(["scenario_id", "sensor_mode"]).iterrows():
        print(f"  Cenario {row.scenario_id} | {row.sensor_mode:20s} | "
              f"full = {row.rmse_full_mean:.3f} +- {row.rmse_full_std:.3f} m | "
              f"steady = {row.rmse_steady_mean:.3f} +- {row.rmse_steady_std:.3f} m | "
              f"n={row.n_repetitions} | divergencia(cov)={row.divergence_rate_pct:.1f}% | "
              f"divergencia(erro real)={row.empirical_divergence_rate_pct:.1f}%")

    # Comparação estatística entre cenários usa RMSE de regime permanente
    # (rmse_steady_m), não o bruto - ver nota em orchestrator.py::run_experiment.
    rmse_by_scenario = {
        sc: raw_df[(raw_df.scenario_id == sc) & (raw_df.sensor_mode == "fusion")].rmse_steady_m.tolist()
        for sc in (1, 2, 3)
    }
    test = compare_scenarios_statistical(rmse_by_scenario)
    print(f"\nTeste estatistico entre cenarios (fusao, regime permanente): {test}")

    print("\n=== COMPARACAO PAREADA ENTRE METODOS, POR CENARIO (Friedman + Wilcoxon pareado) ===")
    method_comparisons = {}
    for sc in (1, 2, 3):
        raw_sc = raw_df[raw_df.scenario_id == sc]
        values_by_method = {}
        ok = True
        for mode in ("fusion", "camera_only", "rf_only", "track_to_track_ci"):
            vals = raw_sc[raw_sc.sensor_mode == mode].sort_values("repetition").rmse_steady_m.tolist()
            if not vals:
                ok = False
                break
            values_by_method[mode] = vals
        comp = compare_methods_paired(values_by_method) if ok else {
            "test": None, "note": "Algum modo ausente neste cenario - comparacao pulada."}
        method_comparisons[sc] = comp
        print(f"  Cenario {sc}: {comp.get('test')} statistic={comp.get('statistic')} p={comp.get('p_value')}")
        for ph in comp.get("post_hoc", []):
            sig = "* " if ph["significant_after_correction"] else "  "
            print(f"    {sig}{ph['method_a']:20s} vs {ph['method_b']:20s} | "
                  f"p_holm={ph['p_value_holm_bonferroni']:.4f} | r={ph['effect_size_rank_biserial']:+.2f}")

    import json
    with open(os.path.join(EXPERIMENT["output_dir"], "method_comparison_by_scenario.json"), "w") as f:
        json.dump(method_comparisons, f, indent=2, default=str)

    if gdop_files:
        gdop_raw = pd.concat([pd.read_csv(f) for f in gdop_files], ignore_index=True)
        gdop_raw.to_csv(os.path.join(EXPERIMENT["output_dir"], "gdop_correlations.csv"), index=False)
        print("\n=== VALIDACAO CRUZADA GDOP x RMSE, por cenario x arquitetura (agregado entre repeticoes) ===")
        print("    (ver docstring de summarize_gdop_correlations para as ressalvas de autocorrelacao)")
        for (sc, mode), g in gdop_raw.groupby(["scenario_id", "sensor_mode"]):
            s = summarize_gdop_correlations(g.pearson_r.tolist())
            print(f"  Cenario {sc} | {mode:20s} | r medio (Fisher)={s['mean_r_fisher']:.3f} "
                  f"(dp={s['std_r']:.3f}, n={s['n']}) | Wilcoxon (r!=0) p={s['wilcoxon_p_value']:.4f}")
    else:
        print("\nAVISO: nenhum arquivo de correlacao GDOP encontrado - "
              "rode novamente apos a correcao do item 4 para gera-los.")

    print(f"\nSalvo em: {EXPERIMENT['output_dir']}")


if __name__ == "__main__":
    n_reps = int(sys.argv[1]) if len(sys.argv) > 1 else EXPERIMENT["n_monte_carlo_repetitions"]
    n_workers = int(sys.argv[2]) if len(sys.argv) > 2 else 4

    os.makedirs(PARTIAL_DIR, exist_ok=True)
    combos = [(sc, rep) for sc in (1, 2, 3) for rep in range(n_reps)]

    print(f"Rodando {len(combos)} combinacoes (3 cenarios x {n_reps} repeticoes) "
          f"com {n_workers} processos em paralelo...")
    print("Isso ainda pode levar bastante tempo dependendo da sua CPU.\n")

    with mp.Pool(n_workers) as pool:
        pool.map(_run_one, combos)

    print("\nTodas as combinacoes concluidas. Agregando resultados...")
    aggregate_and_report()

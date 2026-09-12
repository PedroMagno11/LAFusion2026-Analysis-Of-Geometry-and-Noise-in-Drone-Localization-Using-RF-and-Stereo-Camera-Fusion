"""
Experimento fatorial: geometria (distance_scale) x ruído do sensor
(noise_scale), variados de forma INDEPENDENTE - responde às perguntas
"como a geometria afeta o tracking" e "como o ruído do sensor afeta o
tracking" separadamente, coisa que a análise observacional (GDOP x erro
ao longo da trajetória real) não consegue fazer nos Cenários 1 e 3, onde
GDOP e ruído-via-distância são quase colineares (r=0,92-0,99 - ver
achados_divergencia_fusao.md §13).

Paralelizado com multiprocessing nativo (mesmo padrão de
run_all_parallel.py) - cada combinação (cenário, distance_scale,
noise_scale, repetição) roda num processo separado, escrevendo seu
próprio arquivo em partial_results/ (sem condição de corrida), agregado
ao final.

Uso:
    python3 geometry_noise_experiment.py [n_repeticoes] [cenarios] [distance_scales] [noise_scales] [n_processos]

    Exemplo (exploratório rápido, 4 processos em paralelo):
        python3 geometry_noise_experiment.py 5 1,3 0.75,1.0,1.5 0.5,1.0,2.0 4
    Exemplo (confirmação, mais repetições):
        python3 geometry_noise_experiment.py 20 1,2,3 1.0,1.5 0.5,1.0,2.0 4

Saída: geometry_noise_experiment_results/
    partial_results/sc{X}_ds{Y}_ns{Z}_rep{W}.csv - um arquivo por combinação
        (cenário, distance_scale, noise_scale, repetição) - seguro para
        paralelismo, sem condição de corrida.
    aggregated_by_factor.csv - uma linha por (cenário, distance_scale,
        noise_scale, modo/arquitetura), com RMSE e divergência - a
        tabela central para analisar os dois efeitos separadamente
        (ex.: fixar noise_scale=1.0 e variar distance_scale = efeito
        de geometria pura nessa taxa de ruído; fixar distance_scale=1.0
        e variar noise_scale = efeito de ruído puro nessa geometria).
"""
import glob
import os
import sys
import time
import multiprocessing as mp

import pandas as pd

from config import TRAJECTORY, STATIONS, EXPERIMENT
from station_layout_generator import generate_stations_scaled
from orchestrator import run_repetition
from metrics_evaluator import aggregate_runs_common_grid

OUTPUT_DIR = "geometry_noise_experiment_results"
PARTIAL_DIR = os.path.join(OUTPUT_DIR, "partial_results")


def _run_one(args):
    """Executa uma combinação (cenário, distance_scale, noise_scale,
    repetição) - roda em um processo separado, escreve seu próprio
    arquivo (sem condição de corrida).

    ISOLAMENTO DE GEOMETRIA (correção da revisão externa - ver
    achados_divergencia_fusao.md §14): o ruído é sempre calculado com
    base na geometria de REFERÊNCIA (distance_scale=1,0), não na
    geometria escalada usada para a medição/estimação. Assim, variar
    distance_scale muda SÓ os ângulos/GDOP (geometria pura) - o nível de
    ruído de base fica congelado; noise_scale continua escalando esse
    nível de referência, independentemente. Antes desta correção,
    fixar noise_scale e variar distance_scale ainda deixava o ruído
    mudar junto (porque a fórmula de ruído usa a distância ATUAL, não
    uma referência fixa) - por isso o experimento antigo não isolava
    geometria de verdade, só reduzia o acoplamento (revisão externa,
    ponto 2).
    """
    scenario_id, distance_scale, noise_scale, repetition = args
    stations_df = generate_stations_scaled(scenario_id, TRAJECTORY, STATIONS, distance_scale)
    noise_reference_df = generate_stations_scaled(scenario_id, TRAJECTORY, STATIONS, 1.0)

    summaries, _gdop = run_repetition(
        scenario_id, stations_df, repetition,
        rf_decimation_factor=EXPERIMENT["rf_decimation_factor"],
        save_telemetry=False,
        output_dir=OUTPUT_DIR,
        rf_config_overrides={"noise_scale": noise_scale},
        camera_config_overrides={"noise_scale": noise_scale},
        noise_reference_stations_df=noise_reference_df,
    )
    for s in summaries:
        s["distance_scale"] = distance_scale
        s["noise_scale"] = noise_scale

    out_path = os.path.join(
        PARTIAL_DIR, f"sc{scenario_id}_ds{distance_scale:.3f}_ns{noise_scale:.3f}_rep{repetition}.csv")
    pd.DataFrame(summaries).to_csv(out_path, index=False)
    print(f"OK: cenario={scenario_id} distance_scale={distance_scale:.2f} "
          f"noise_scale={noise_scale:.2f} rep={repetition} -> {out_path}", flush=True)
    return out_path


def aggregate_and_report():
    files = glob.glob(os.path.join(PARTIAL_DIR, "sc*_ds*_ns*_rep*.csv"))
    print(f"\nArquivos parciais encontrados: {len(files)}")
    if not files:
        print("Nenhum resultado encontrado - nada a agregar.")
        return None

    raw = pd.concat([pd.read_csv(f) for f in files], ignore_index=True)

    agg_rows = []
    for (sc, ds, ns), group in raw.groupby(["scenario_id", "distance_scale", "noise_scale"]):
        aggregated, _raw_df, _divergence = aggregate_runs_common_grid(group.to_dict("records"))
        aggregated["distance_scale"] = ds
        aggregated["noise_scale"] = ns
        agg_rows.append(aggregated)

    result = pd.concat(agg_rows, ignore_index=True)
    result.to_csv(os.path.join(OUTPUT_DIR, "aggregated_by_factor.csv"), index=False)

    print("\n=== RESULTADOS AGREGADOS (fusion, para leitura rápida) ===")
    fusion = result[result.sensor_mode == "fusion"].sort_values(["scenario_id", "distance_scale", "noise_scale"])
    for _, row in fusion.iterrows():
        print(f"  Cenario {row.scenario_id} | distance_scale={row.distance_scale:.2f} | "
              f"noise_scale={row.noise_scale:.2f} | RMSE_steady={row.rmse_steady_mean:8.3f}m | "
              f"divergencia={row.divergence_rate_pct:5.1f}%")

    print(f"\nSalvo em: {OUTPUT_DIR}/aggregated_by_factor.csv")
    return result


if __name__ == "__main__":
    n_reps = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    scenarios = [int(s) for s in sys.argv[2].split(",")] if len(sys.argv) > 2 else [1, 2, 3]
    distance_scales = [float(s) for s in sys.argv[3].split(",")] if len(sys.argv) > 3 else [1.0]
    noise_scales = [float(s) for s in sys.argv[4].split(",")] if len(sys.argv) > 4 else [0.5, 1.0, 2.0]
    n_workers = int(sys.argv[5]) if len(sys.argv) > 5 else 4

    os.makedirs(PARTIAL_DIR, exist_ok=True)
    combos = [(sc, ds, ns, rep)
              for sc in scenarios for ds in distance_scales for ns in noise_scales
              for rep in range(n_reps)]

    print(f"Rodando {len(combos)} combinacoes (cenarios={scenarios} x "
          f"distance_scales={distance_scales} x noise_scales={noise_scales} x "
          f"{n_reps} repeticoes) com {n_workers} processos em paralelo...")
    print("Isso pode levar bastante tempo dependendo do tamanho da grade e da sua CPU.\n")

    t0 = time.time()
    with mp.Pool(n_workers) as pool:
        pool.map(_run_one, combos)
    elapsed = time.time() - t0

    print(f"\nTodas as combinacoes concluidas em {elapsed/60:.1f} min. Agregando resultados...")
    aggregate_and_report()

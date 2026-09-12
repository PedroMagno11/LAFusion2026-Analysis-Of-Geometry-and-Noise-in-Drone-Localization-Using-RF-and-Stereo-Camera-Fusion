"""
Experimento de sensibilidade ao afastamento das estações (distance_scale).

Investiga se afastar as estações do centro da trajetória, melhora a cobertura de FOV da câmera nas bordas
da trajetória e, por consequência, reduz a taxa de divergência da fusão
nos Cenários 2 e 3 - ver achados_divergencia_fusao.md para o contexto
completo da investigação que motivou este experimento.

REPRODUTIBILIDADE: as sementes de ruído (RF/câmera) dependem só de
scenario_id e repetition, não de distance_scale, então, para uma mesma 
repetição, a única coisa que muda entre valores de
distance_scale é a geometria das estações, isolando exatamente o efeito
que este experimento quer medir.

Uso:
    python3 spacing_experiment.py [n_repetitions] [scenarios] [scales]

    n_repetitions : int, default 10 (mais rápido que as 30 do experimento
        principal - é uma varredura exploratória, não o resultado final;
        subir para 30 depois de escolher os valores de distance_scale
        que valem a pena reportar no artigo)
    scenarios     : lista separada por vírgula, default "2,3" (Cenário 1
        já tem 0% de divergência na distância original - só entra na
        varredura se explicitamente pedido)
    scales        : lista separada por vírgula, default
        "0.5,0.75,1.0,1.25,1.5,2.0"

Exemplo:
    python3 spacing_experiment.py 10 2,3 0.5,1.0,1.5,2.0
    python3 spacing_experiment.py 30 1,2,3 1.0,1.5   # confirmação final

Saída: spacing_experiment_results/
    raw_<scenario>_<scale>.csv        - uma linha por repetição x modo
    aggregated_by_scale.csv           - resumo por cenário x escala x modo
    coverage_by_scale.csv             - fração de tempo com 3 estações em
                                         FOV, por cenário x escala (não
                                         depende de repetição - é
                                         determinístico pela geometria)
"""
import os
import sys
import time
import numpy as np
import pandas as pd

from config import TRAJECTORY, STATIONS, CAMERA, EXPERIMENT
from station_layout_generator import generate_stations_scaled
from camera_sensor_model import generate_camera_detections
from orchestrator import run_repetition
from metrics_evaluator import aggregate_runs_common_grid

OUTPUT_DIR = "./spacing_experiment_results"


def three_station_coverage_fraction(scenario_id: int, distance_scale: float) -> float:
    """
    Fração do tempo de voo em que as 3 estações têm o drone dentro do FOV
    da câmera simultaneamente - métrica determinística (não depende de
    ruído/repetição), calculada com a mesma lógica de geração de detecção
    de câmera já usada no resto do pipeline (generate_camera_detections,
    campo in_fov antes do filtro que descarta linhas fora do FOV - aqui
    recalculado de propósito para inspecionar esse campo diretamente).
    """
    stations_df = generate_stations_scaled(scenario_id, TRAJECTORY, STATIONS, distance_scale)
    cam_cfg = dict(CAMERA)
    cam_cfg["seed"] = CAMERA["seed"]  # cobertura de FOV é geométrica, não depende do ruído
    cam = generate_camera_detections(scenario_id, TRAJECTORY, stations_df, cam_cfg)
    bins = np.arange(0, TRAJECTORY["duration_s"] + 0.5, 1.0)
    cam = cam.copy()
    cam["bin"] = np.digitize(cam.timestamp, bins)
    per_bin = cam.groupby("bin").station_id.nunique()
    return float((per_bin == 3).sum() / (len(bins) - 1))


def run_spacing_experiment(n_repetitions: int, scenarios: list, scales: list):
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    coverage_rows = []
    agg_rows = []

    for scenario_id in scenarios:
        for scale in scales:
            t0 = time.time()
            stations_df = generate_stations_scaled(scenario_id, TRAJECTORY, STATIONS, scale)

            all_summaries = []
            gdop_rows = []
            for repetition in range(n_repetitions):
                summaries, gdop_results = run_repetition(
                    scenario_id, stations_df, repetition,
                    rf_decimation_factor=EXPERIMENT["rf_decimation_factor"],
                    save_telemetry=False,  # experimento exploratório - não salva telemetria pesada
                    output_dir=OUTPUT_DIR,
                )
                all_summaries.extend(summaries)
                gdop_rows.extend(gdop_results)

            raw_df = pd.DataFrame(all_summaries)
            raw_df.to_csv(os.path.join(OUTPUT_DIR, f"raw_scenario{scenario_id}_scale{scale}.csv"), index=False)

            aggregated, _, divergence = aggregate_runs_common_grid(all_summaries)
            aggregated["distance_scale"] = scale
            agg_rows.append(aggregated)

            coverage = three_station_coverage_fraction(scenario_id, scale)
            coverage_rows.append({"scenario_id": scenario_id, "distance_scale": scale,
                                   "coverage_3_stations_frac": coverage})

            elapsed = time.time() - t0
            fusion_div = divergence[divergence.sensor_mode == "fusion"]
            div_pct = fusion_div.divergence_rate_pct.iloc[0] if len(fusion_div) else float("nan")
            print(f"Cenario {scenario_id} | escala {scale:.2f} | cobertura_3_estacoes={coverage*100:.0f}% | "
                  f"divergencia_fusao={div_pct:.1f}% | {elapsed:.1f}s ({n_repetitions} reps)", flush=True)

    pd.DataFrame(coverage_rows).to_csv(os.path.join(OUTPUT_DIR, "coverage_by_scale.csv"), index=False)
    pd.concat(agg_rows, ignore_index=True).to_csv(
        os.path.join(OUTPUT_DIR, "aggregated_by_scale.csv"), index=False)
    print(f"\nSalvo em: {OUTPUT_DIR}/")


if __name__ == "__main__":
    n_reps = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    scenarios = [int(s) for s in sys.argv[2].split(",")] if len(sys.argv) > 2 else [2, 3]
    scales = [float(s) for s in sys.argv[3].split(",")] if len(sys.argv) > 3 else \
        [0.5, 0.75, 1.0, 1.25, 1.5, 2.0]

    print(f"Rodando experimento de espacamento: cenarios={scenarios}, escalas={scales}, "
          f"{n_reps} repeticoes cada")
    run_spacing_experiment(n_reps, scenarios, scales)

"""
Executa uma combinação (cenário, repetição) do experimento e salva o
resultado em um arquivo próprio (não compartilhado) - seguro para rodar
em paralelo (múltiplos processos simultâneos), já que cada processo só
escreve no seu próprio arquivo, sem condição de corrida.

Uso: python3 run_single_repetition.py <scenario_id> <repetition>
"""
import sys
import os
import pandas as pd

from config import TRAJECTORY, STATIONS, EXPERIMENT
from station_layout_generator import generate_stations
from orchestrator import run_repetition

PARTIAL_DIR = os.path.join(EXPERIMENT["output_dir"], "partial_results")


def main():
    scenario_id = int(sys.argv[1])
    repetition = int(sys.argv[2])

    os.makedirs(PARTIAL_DIR, exist_ok=True)
    stations_df = generate_stations(scenario_id, TRAJECTORY, STATIONS)

    # CORREÇÃO DE PROTOCOLO (itens 1, 3 e 4): run_repetition() roda o
    # EKF global (fusion/rf_only/camera_only) e a fusão track-to-track
    # (CI), e retorna (summaries, gdop_results) - ver orchestrator.py.
    # summaries traz 'rmse_full_m'/'rmse_steady_m'/'covariance_stabilized'/'empirically_diverged'/
    # 't_conv_s' por modo/arquitetura; gdop_results é uma LISTA (uma
    # linha para "fusion", outra para "track_to_track_ci"), salva em arquivo próprio (prefixo 'gdop_'), no mesmo
    # padrão de run_all_parallel.py, para que aggregate_and_report() (ou
    # o bloco equivalente em run_all_parallel.sh) consiga juntar tudo
    # depois.
    summaries, gdop_results = run_repetition(
        scenario_id, stations_df, repetition,
        rf_decimation_factor=EXPERIMENT["rf_decimation_factor"],
        save_telemetry=(repetition == 0),
        output_dir=EXPERIMENT["output_dir"],
    )

    # Arquivo próprio desta combinação - sem disputa com outros processos
    out_path = os.path.join(PARTIAL_DIR, f"scenario{scenario_id}_rep{repetition}.csv")
    pd.DataFrame(summaries).to_csv(out_path, index=False)

    gdop_path = os.path.join(PARTIAL_DIR, f"gdop_scenario{scenario_id}_rep{repetition}.csv")
    pd.DataFrame(gdop_results).to_csv(gdop_path, index=False)

    print(f"Cenário {scenario_id}, repetição {repetition} concluída -> {out_path}")
    for s in summaries:
        print(f"  {s['sensor_mode']:20s} cov_estabilizada={s['covariance_stabilized']!s:5s} "
              f"divergiu_empiric={s['empirically_diverged']!s:5s} "
              f"RMSE_full={s['rmse_full_m']:.3f}m RMSE_steady={s['rmse_steady_m']:.3f}m")
    for g in gdop_results:
        print(f"  GDOP x erro ({g['sensor_mode']}): r={g['pearson_r']:.3f}, p={g['p_value']:.4f}")


if __name__ == "__main__":
    main()

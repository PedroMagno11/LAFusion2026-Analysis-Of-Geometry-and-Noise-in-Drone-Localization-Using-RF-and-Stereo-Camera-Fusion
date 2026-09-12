"""
Roda o estudo completo em um único comando:
  1. Experimento principal (run_all_parallel.py) - N repetições x 3
     cenários x 5 modos/arquiteturas (fusion, rf_only, camera_only,
     track_to_track_ci, baseline_geometric).
  2. Experimento fatorial geometria x ruído (geometry_noise_experiment.py)
     - distance_scale x noise_scale, independentes.
  3. Gráficos finais (plot_experiment_results.py).
  4. Um resumo consolidado dos dois experimentos, impresso no final e
     salvo em resumo_estudo_completo.txt.

Uso:
    python3 run_full_study.py [opções]

Exemplos:
    # Padrão (recomendado para o resultado final do artigo):
    python3 run_full_study.py

    # Mais rápido (exploratório, menos repetições, menos combinações):
    python3 run_full_study.py --n-reps-principal 10 --n-reps-fatorial 5 \\
        --distance-scales 1.0,1.5 --noise-scales 0.5,1.0,2.0

    # Só o experimento principal (pula o fatorial):
    python3 run_full_study.py --pular-fatorial

    # Só o fatorial (pula o principal, ex.: já rodou antes):
    python3 run_full_study.py --pular-principal

"""
import argparse
import glob
import os
import subprocess
import sys
import time

import pandas as pd

EXPERIMENT_DIR = "experiment_results"
FACTORIAL_DIR = "geometry_noise_experiment_results"


def clear_stale_partials(directory: str, label: str):
    """
    Remove CSVs de partial_results/ antigos ANTES de uma nova rodada.

    ACHADO (durante o teste deste script): run_all_parallel.py não limpa
    partial_results/ sozinho - se a pasta já tiver arquivos de uma rodada
    anterior com N diferente, a agregação final mistura o novo com o
    velho silenciosamente (glob pega todos os scenarioX_repY.csv, não só
    os desta execução). Limpar antes garante que o resultado agregado
    corresponde exatamente à rodada que você acabou de pedir.
    """
    partial_dir = os.path.join(directory, "partial_results")
    if not os.path.isdir(partial_dir):
        return
    files = glob.glob(os.path.join(partial_dir, "*.csv"))
    if files:
        print(f"[AVISO] {label}: removendo {len(files)} arquivo(s) de uma rodada anterior "
              f"em {partial_dir}/ antes de começar (evita misturar resultados antigos com os novos).")
        for f in files:
            os.remove(f)


def run_step(cmd: list, label: str) -> float:
    print(f"\n{'=' * 78}\n{label}\n{'=' * 78}", flush=True)
    print(f"Comando: {' '.join(cmd)}\n", flush=True)
    t0 = time.time()
    result = subprocess.run([sys.executable] + cmd)
    elapsed = time.time() - t0
    if result.returncode != 0:
        print(f"\n[ERRO] '{label}' terminou com código {result.returncode} "
              f"após {elapsed / 60:.1f} min - abortando as próximas etapas.")
        sys.exit(result.returncode)
    print(f"\n[OK] '{label}' concluído em {elapsed / 60:.1f} min")
    return elapsed


def build_summary() -> str:
    """Lê os CSVs dos dois experimentos (se existirem) e monta um resumo
    legível único - não recalcula nada, só reorganiza o que já foi salvo."""
    lines = []
    lines.append("=" * 78)
    lines.append("RESUMO DO ESTUDO COMPLETO")
    lines.append("=" * 78)

    agg_path = os.path.join(EXPERIMENT_DIR, "aggregated_metrics.csv")
    if os.path.exists(agg_path):
        lines.append("\n--- Experimento principal: RMSE de regime permanente e divergência ---\n")
        df = pd.read_csv(agg_path)
        for _, row in df.sort_values(["scenario_id", "sensor_mode"]).iterrows():
            lines.append(
                f"  Cenário {row.scenario_id} | {row.sensor_mode:20s} | "
                f"RMSE_steady={row.rmse_steady_mean:10.3f}m ± {row.rmse_steady_std:8.3f} | "
                f"divergência={row.divergence_rate_pct:5.1f}% | n={row.n_repetitions}"
            )
    else:
        lines.append("\n(Experimento principal não encontrado - pulado ou ainda não rodou.)")

    gdop_path = os.path.join(EXPERIMENT_DIR, "gdop_correlations.csv")
    if os.path.exists(gdop_path):
        lines.append("\n--- Correlação GDOP x erro (média por cenário x arquitetura) ---\n")
        g = pd.read_csv(gdop_path)
        summary = g.groupby(["scenario_id", "sensor_mode"]).agg(
            r_medio=("pearson_r", "mean"), n_sig=("p_value", lambda s: (s < 0.05).sum()),
            n=("p_value", "count"),
        ).reset_index()
        for _, row in summary.iterrows():
            lines.append(f"  Cenário {row.scenario_id} | {row.sensor_mode:20s} | "
                          f"r médio={row.r_medio:6.3f} | significativo em {row.n_sig}/{row.n}")

    fact_path = os.path.join(FACTORIAL_DIR, "aggregated_by_factor.csv")
    if os.path.exists(fact_path):
        lines.append("\n--- Experimento fatorial: geometria (distance_scale) x ruído (noise_scale) ---\n")
        lines.append("    (fusion apenas, para leitura rápida - ver o CSV completo para todos os modos)\n")
        f = pd.read_csv(fact_path)
        f_fusion = f[f.sensor_mode == "fusion"].sort_values(["scenario_id", "distance_scale", "noise_scale"])
        for _, row in f_fusion.iterrows():
            lines.append(
                f"  Cenário {row.scenario_id} | distance_scale={row.distance_scale:.2f} | "
                f"noise_scale={row.noise_scale:.2f} | "
                f"RMSE_steady={row.rmse_steady_mean:8.3f}m | divergência={row.divergence_rate_pct:5.1f}%"
            )
    else:
        lines.append("\n(Experimento fatorial não encontrado - pulado ou ainda não rodou.)")

    lines.append("\n" + "=" * 78)
    lines.append("Arquivos completos: experiment_results/*.csv e "
                  "geometry_noise_experiment_results/aggregated_by_factor.csv")
    lines.append("Documentação da metodologia: achados_divergencia_fusao.md")
    lines.append("=" * 78)
    return "\n".join(lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n-reps-principal", type=int, default=30,
                         help="Repetições do experimento principal (padrão: 30)")
    parser.add_argument("--n-workers", type=int, default=4,
                         help="Processos paralelos para o experimento principal (padrão: 4)")
    parser.add_argument("--n-reps-fatorial", type=int, default=20,
                         help="Repetições por combinação do experimento fatorial (padrão: 20)")
    parser.add_argument("--cenarios-fatorial", type=str, default="1,2,3",
                         help="Cenários no fatorial, separados por vírgula (padrão: 1,2,3)")
    parser.add_argument("--distance-scales", type=str, default="0.75,1.0,1.5",
                         help="Escalas de distância no fatorial (padrão: 0.75,1.0,1.5)")
    parser.add_argument("--noise-scales", type=str, default="0.5,1.0,2.0",
                         help="Escalas de ruído no fatorial (padrão: 0.5,1.0,2.0)")
    parser.add_argument("--pular-principal", action="store_true",
                         help="Não roda o experimento principal (usa resultados já existentes)")
    parser.add_argument("--pular-fatorial", action="store_true",
                         help="Não roda o experimento fatorial")
    parser.add_argument("--pular-graficos", action="store_true",
                         help="Não gera os gráficos finais")
    args = parser.parse_args()

    t_total_start = time.time()
    tempos = {}

    if not args.pular_principal:
        clear_stale_partials(EXPERIMENT_DIR, "Experimento principal")
        tempos["principal"] = run_step(
            ["run_all_parallel.py", str(args.n_reps_principal), str(args.n_workers)],
            f"ETAPA 1/3 - Experimento principal (N={args.n_reps_principal}, "
            f"{args.n_workers} processos)",
        )
    else:
        print("\n[PULADO] Experimento principal (--pular-principal)")

    if not args.pular_fatorial:
        clear_stale_partials(FACTORIAL_DIR, "Experimento fatorial")
        tempos["fatorial"] = run_step(
            ["geometry_noise_experiment.py", str(args.n_reps_fatorial),
             args.cenarios_fatorial, args.distance_scales, args.noise_scales, str(args.n_workers)],
            f"ETAPA 2/3 - Experimento fatorial geometria x ruído "
            f"(N={args.n_reps_fatorial}, cenários={args.cenarios_fatorial}, "
            f"distance_scales={args.distance_scales}, noise_scales={args.noise_scales}, "
            f"{args.n_workers} processos)",
        )
    else:
        print("\n[PULADO] Experimento fatorial (--pular-fatorial)")

    if not args.pular_graficos and not args.pular_principal:
        tempos["graficos"] = run_step(["plot_experiment_results.py"], "ETAPA 3/3 - Gráficos finais")
    else:
        print("\n[PULADO] Geração de gráficos")

    total_elapsed = time.time() - t_total_start
    print(f"\n\nTempo total do estudo: {total_elapsed / 60:.1f} min "
          f"({', '.join(f'{k}={v/60:.1f}min' for k, v in tempos.items())})")

    summary = build_summary()
    print("\n\n" + summary)
    with open("resumo_estudo_completo.txt", "w", encoding="utf-8") as f:
        f.write(summary)
    print("\nResumo salvo em resumo_estudo_completo.txt")

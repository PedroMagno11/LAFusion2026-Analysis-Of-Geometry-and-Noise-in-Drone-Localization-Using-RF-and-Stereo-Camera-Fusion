"""
Visualização dos resultados agregados do experimento completo.

CORRIGIDO (revisão externa incorporada - ver achados_divergencia_fusao.md):
este script estava desatualizado desde a correção de protocolo (grade
temporal comum + separação de regime permanente) e a remoção do UKF -
usava colunas antigas (rmse_mean/rmse_std/rmse_m), não incluía
track_to_track_ci, rotulava a fusão como "(UKF)", e o título afirmava
"3 repetições" (o N real vem do experimento, não é fixo). OUT_DIR também
estava com um caminho errado ("./outputs/experiment_results", que nunca
existiu) - agora lido de config.py, para não voltar a divergir.

Gera:
  - experiment_rmse_by_scenario.png: RMSE de regime permanente médio ±
    desvio-padrão, por cenário e modo/arquitetura, com os pontos de cada
    repetição individual sobrepostos (para mostrar a variância real, não
    só a média).
  - experiment_rmse_by_mode.png: mesma informação, reorganizada por modo
    (facilita comparar os 3 cenários dentro de cada modo).
  - experiment_divergence_rates.png: taxa de divergência por
    covariância (o próprio filtro) e por erro real (ground truth) lado
    a lado - ver metrics_evaluator.py::summarize_run_common_grid para a
    distinção entre as duas.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from config import EXPERIMENT

OUT_DIR = EXPERIMENT["output_dir"]

MODES = ["fusion", "camera_only", "rf_only", "track_to_track_ci", "baseline_geometric"]
MODE_LABELS = {
    "fusion": "Fusão\n(EKF global)", "camera_only": "Câmera\n-only",
    "rf_only": "RF\n-only", "track_to_track_ci": "Track-to-\ntrack (CI)",
    "baseline_geometric": "Baseline\ngeométrico",
}
MODE_COLORS = {
    "fusion": "tab:blue", "camera_only": "tab:orange",
    "rf_only": "tab:red", "track_to_track_ci": "tab:purple",
    "baseline_geometric": "tab:green",
}


def _modes_present(df: pd.DataFrame) -> list:
    """Só plota os modos que de fato existem nos dados (robusto a rodadas
    parciais, ex.: --pular-fatorial ou execuções antigas sem alguma arquitetura)."""
    return [m for m in MODES if m in df.sensor_mode.unique()]


if __name__ == "__main__":
    raw = pd.read_csv(f"{OUT_DIR}/raw_run_summaries.csv")
    aggregated = pd.read_csv(f"{OUT_DIR}/aggregated_metrics.csv")
    modes = _modes_present(aggregated)
    n_reps = int(aggregated.n_repetitions.max())

    # --- RMSE de regime permanente por cenário (barras + pontos individuais) ---
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=False)
    for ax, scenario_id in zip(axes, (1, 2, 3)):
        agg_sc = aggregated[aggregated.scenario_id == scenario_id].set_index("sensor_mode")
        raw_sc = raw[raw.scenario_id == scenario_id]

        means = [agg_sc.loc[m, "rmse_steady_mean"] if m in agg_sc.index else np.nan for m in modes]
        stds = [agg_sc.loc[m, "rmse_steady_std"] if m in agg_sc.index else np.nan for m in modes]
        colors = [MODE_COLORS[m] for m in modes]

        ax.bar(range(len(modes)), means, yerr=stds, capsize=5,
               color=colors, alpha=0.7, edgecolor="black")

        for i, m in enumerate(modes):
            pts = raw_sc[raw_sc.sensor_mode == m].rmse_steady_m.values
            jitter = np.random.uniform(-0.12, 0.12, size=len(pts))
            ax.scatter(np.full(len(pts), i) + jitter, pts, color="black",
                       s=25, zorder=5, alpha=0.8)

        ax.set_xticks(range(len(modes)))
        ax.set_xticklabels([MODE_LABELS[m] for m in modes], fontsize=9)
        ax.set_yscale("log")  # RF-only pode ser ordens de magnitude maior - log evita esmagar o resto
        ax.set_title(f"Cenário {scenario_id}")
        ax.grid(alpha=0.3, axis="y")
        if scenario_id == 1:
            ax.set_ylabel("RMSE de regime permanente (m, escala log)")

    fig.suptitle(f"RMSE de regime permanente por cenário e modo/arquitetura "
                 f"(barras = média±desvio-padrão de {n_reps} repetições; "
                 f"pontos = repetições individuais)", fontsize=11)
    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/experiment_rmse_by_scenario.png", dpi=150)
    plt.close(fig)
    print("Gerado: experiment_rmse_by_scenario.png")

    # --- RMSE de regime permanente por modo, comparando os 3 cenários ---
    fig, ax = plt.subplots(figsize=(13, 6))
    x = np.arange(len(modes))
    width = 0.25
    scenario_colors = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}

    for i, scenario_id in enumerate((1, 2, 3)):
        agg_sc = aggregated[aggregated.scenario_id == scenario_id].set_index("sensor_mode")
        means = [agg_sc.loc[m, "rmse_steady_mean"] if m in agg_sc.index else np.nan for m in modes]
        stds = [agg_sc.loc[m, "rmse_steady_std"] if m in agg_sc.index else np.nan for m in modes]
        ax.bar(x + (i - 1) * width, means, width, yerr=stds, capsize=4,
               label=f"Cenário {scenario_id}", color=scenario_colors[scenario_id], alpha=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels([MODE_LABELS[m] for m in modes])
    ax.set_yscale("log")
    ax.set_ylabel("RMSE de regime permanente (m, escala log)")
    ax.set_title(f"RMSE por modo/arquitetura, comparando os 3 cenários geométricos (N={n_reps})")
    ax.legend()
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/experiment_rmse_by_mode.png", dpi=150)
    plt.close(fig)
    print("Gerado: experiment_rmse_by_mode.png")

    # --- Taxas de divergência (covariância vs. erro real) ---
    if "divergence_rate_pct" in aggregated.columns and "empirical_divergence_rate_pct" in aggregated.columns:
        fig, axes = plt.subplots(1, 2, figsize=(15, 6), sharey=True)
        for ax, col, title in zip(
            axes, ("divergence_rate_pct", "empirical_divergence_rate_pct"),
            ("Divergência por covariância\n(o que o próprio filtro reporta)",
             "Divergência empírica\n(erro real > limiar, sustentado - só em simulação)"),
        ):
            xw = np.arange(len(modes))
            for i, scenario_id in enumerate((1, 2, 3)):
                agg_sc = aggregated[aggregated.scenario_id == scenario_id].set_index("sensor_mode")
                vals = [agg_sc.loc[m, col] if m in agg_sc.index else np.nan for m in modes]
                ax.bar(xw + (i - 1) * width, vals, width, label=f"Cenário {scenario_id}",
                       color=scenario_colors[scenario_id], alpha=0.8)
            ax.set_xticks(xw)
            ax.set_xticklabels([MODE_LABELS[m] for m in modes], fontsize=9)
            ax.set_title(title, fontsize=10)
            ax.grid(alpha=0.3, axis="y")
        axes[0].set_ylabel("Taxa de divergência (%)")
        axes[0].legend()
        fig.suptitle(f"Taxas de divergência - covariância vs. erro real (N={n_reps})", fontsize=11)
        fig.tight_layout()
        fig.savefig(f"{OUT_DIR}/experiment_divergence_rates.png", dpi=150)
        plt.close(fig)
        print("Gerado: experiment_divergence_rates.png")
    else:
        print("AVISO: colunas de divergência não encontradas em aggregated_metrics.csv - "
              "pulei experiment_divergence_rates.png (rode a versão atual do experimento).")

    print(f"\n=== Resumo: fusão vs. câmera-only vs. track-to-track, por cenário (regime permanente) ===")
    for scenario_id in (1, 2, 3):
        parts = []
        for m in ("fusion", "camera_only", "track_to_track_ci"):
            if m in modes:
                vals = raw[(raw.scenario_id == scenario_id) & (raw.sensor_mode == m)].rmse_steady_m.values
                parts.append(f"{MODE_LABELS[m].replace(chr(10), ' ')}={sorted(np.round(vals, 3))}")
        print(f"  Cenário {scenario_id}: " + " | ".join(parts))

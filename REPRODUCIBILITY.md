# Reproducibility Guide

This document is written for reviewers who want to independently reproduce
the experiments reported in the paper.

---

## 1. What this reproduces

A simulation study of RF+camera sensor fusion for 3D drone localization,
comparing:

- **Two fusion architectures**, both built on an Extended Kalman Filter
  (EKF): (1) a single global EKF processing all stations' detections
  sequentially ("fusion"), and (2) a distributed track-to-track scheme
  where each station runs its own local EKF and a global estimate is
  produced via Covariance Intersection ("track_to_track_ci").
- **Two single-sensor baselines** ("rf_only", "camera_only") and a
  **filterless geometric baseline** ("baseline_geometric").
- **Three station-geometry topologies** (linear, circular, asymmetric)
  over the same synthetic trajectory.
- A **factorial experiment** that varies station distance and sensor noise
  level independently, to separate geometric effects from noise effects.

All experiments are Monte Carlo studies (repeated with independent random
seeds) over a fixed, deterministic ground-truth trajectory.

## 2. Requirements

- Python 3.10+ (developed and tested on 3.12).
- No GPU required. A single full run (N=30) takes on the order of hours on
  a modern multi-core desktop/laptop — see Section 5 for scaling advice.

Install dependencies:

```bash
pip install -r requirements.txt
```

(if you are using a virtual environment,
which we recommend: `python -m venv venv`, then activate it before
installing.)


On Windows, use `python` instead of `python3` in the commands below.

## 3. Quick sanity check (< 2 minutes)

Before committing to a full run, confirm the environment is set up
correctly:

```bash
python3 trajectory_generator.py
python3 ekf_fusion.py
python3 track_to_track_fusion.py
```

Each should run in a few seconds and print a short numerical summary with
no errors. If any of these fail, the issue is almost certainly a missing
dependency or Python version, not the experiment itself.

## 4. Reproducing the reported results

Everything is orchestrated by a single script, `run_full_study.py`, which
runs the main experiment, the factorial experiment, generates the summary
figures, and writes a consolidated text summary at the end.

**Full reproduction (the numbers reported in the paper):**

```bash
python3 run_full_study.py
```

This uses the defaults: N=30 repetitions for the main experiment (3
scenarios × 5 modes/architectures), N=20 repetitions × 27 combinations
for the factorial experiment (3 scenarios × 3 station-distance scales ×
3 noise scales), and 4 parallel worker processes.

**Recommended first step — a fast partial run**, to confirm your machine
reproduces sane numbers before committing hours of compute:

```bash
python3 run_full_study.py --n-reps-principal 5 --n-reps-fatorial 3
```

**Adjusting parallelism** to your machine's core count (check with
`python3 -c "import os; print(os.cpu_count())"`):

```bash
python3 run_full_study.py --n-workers 8
```

**Running only part of the study** (e.g., if you already have one part and
only need to re-check the other):

```bash
python3 run_full_study.py --pular-principal   # skip the main experiment
python3 run_full_study.py --pular-fatorial    # skip the factorial experiment
```

Run `python3 run_full_study.py --help` for the full list of options
(repetition counts, worker count, scenario list, station-distance and
noise-scale grids).

## 5. Expected runtime

The main experiment now runs **two full fusion architectures per
repetition** (global EKF and track-to-track), which roughly doubles the
runtime relative to a single-architecture pipeline. As a rough guide from
our own runs: a single (scenario, repetition) combination at full RF
fidelity (833 Hz, no decimation — see `config.py`, `EXPERIMENT
["rf_decimation_factor"]`, which must stay at `1` for the reported
results) takes on the order of tens of seconds to a couple of minutes,
depending on hardware; multiply by 90 combinations (3 scenarios × 30
repetitions) and divide by your worker count for a rough estimate. The
factorial experiment (27 geometry×noise combinations × 20 repetitions =
540 runs) is the more expensive stage — budget accordingly, or reduce
`--n-reps-fatorial` / the scale grids for an initial check.

## 6. Where results land, and how they map to the paper

All main-experiment output goes to `experiment_results/` (path
configurable via `config.py`, key `EXPERIMENT["output_dir"]`); factorial
output goes to `geometry_noise_experiment_results/`.

| File | Contents | Used for |
|---|---|---|
| `experiment_results/raw_run_summaries.csv` | One row per (scenario, mode/architecture, repetition): steady-state and full-trajectory RMSE (3D and horizontal), convergence/divergence flags, NEES, etc. | Underlies every RMSE/divergence table in the paper |
| `experiment_results/aggregated_metrics.csv` | Mean ± std of the above, aggregated per (scenario, mode) | Main results table |
| `experiment_results/divergence_rates.csv` | Divergence rate by two distinct criteria (filter-reported covariance vs. ground-truth error) — see `achados_divergencia_fusao.md` §12 for why these differ | Divergence/robustness discussion |
| `experiment_results/gdop_correlations.csv` | Per-repetition Pearson correlation between theoretical GDOP and empirical error | H3 (geometry predicts error) evidence |
| `experiment_results/method_comparison_by_scenario.json` | Friedman test + paired Wilcoxon post-hoc (Holm-Bonferroni corrected) comparing architectures within each scenario | Statistical significance claims for architecture comparisons |
| `experiment_results/experiment_rmse_by_scenario.png`, `experiment_rmse_by_mode.png`, `experiment_divergence_rates.png` | Summary figures | Paper figures |
| `geometry_noise_experiment_results/aggregated_by_factor.csv` | RMSE/divergence as a function of station-distance scale and noise scale, independently varied | Geometry-vs-noise factorial results |
| `resumo_estudo_completo.txt` | Plain-text digest of the two tables above, generated automatically at the end of `run_full_study.py` | Quick cross-check without opening CSVs |

## 7. Determinism

All Monte Carlo repetitions use a fixed seed formula
(`base_seed + repetition * 1000 + scenario_id`, see `orchestrator.py`), so
a full re-run with the same `N` and scenario/mode set reproduces the
**same** raw numbers bit-for-bit (up to floating-point non-associativity
across different CPU core counts / parallel scheduling, which does not
affect any reported statistic at the precision used in the paper).
**Important:** if you re-run with a different repetition count `N` after a
previous run, `run_full_study.py` clears stale partial-result files
before starting, to avoid silently mixing results from different `N`
across runs.

## 8. Repository structure

- `trajectory_generator.py`, `station_layout_generator.py`: deterministic
  ground-truth trajectory and the three station geometries.
- `rf_sensor_model.py`, `camera_sensor_model.py`: sensor noise models
  (calibrated to literature — see `metodologia_completa.md` §3).
- `fusion_models.py`: shared motion/measurement models and filter
  initialization (used by both architectures).
- `ekf_fusion.py`: the global EKF (fusion/rf_only/camera_only modes).
- `track_to_track_fusion.py`: the distributed (local EKF + Covariance
  Intersection) architecture.
- `baseline_estimator.py`: the filterless geometric baseline.
- `gdop_calculator.py`: the bearing-only geometric dilution index.
- `metrics_evaluator.py`: all evaluation metrics (common time grid, RMSE,
  NEES, statistical tests).
- `orchestrator.py`: runs one (scenario, repetition) combination across
  all modes/architectures — the core unit reused by every entry point
  below.
- `run_all_parallel.py`, `run_single_repetition.py` (+ `run_all_parallel.sh`),
  `run_full_study.py`: three ways to run the main experiment (parallel
  Python, one-process-per-combination, and the all-in-one orchestrator,
  respectively).
- `geometry_noise_experiment.py`: the factorial experiment.
- `plot_experiment_results.py`: summary figures.
- `validate_module_*.py`, `validate_modules_1_2.py`: per-module sanity
  checks with visual output (not required for reproducing the numerical
  results, but useful for inspecting individual pipeline stages).

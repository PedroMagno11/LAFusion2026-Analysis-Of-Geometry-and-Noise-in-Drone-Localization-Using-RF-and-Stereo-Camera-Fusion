# Simulador de Fusão RF+Câmera para Localização de Drones

Guia rápido para rodar o projeto do zero. Para entender o que cada arquivo faz e a fundamentação na literatura, veja `guia_arquivos_e_literatura.md`. Para o desenho
completo da pesquisa, veja `metodologia_fusao_rf_camera_drone.md`. Para instruções
em inglês voltadas a revisores (reprodução dos resultados do artigo), veja
`REPRODUCIBILITY.md`.



## 1. Preparar o ambiente

```bash
pip install -r requirements.txt
```
---

## 2. Testar rapidamente que tudo está funcionando

Rode qualquer módulo isolado — deve terminar em segundos e imprimir um resumo numérico:

```bash
python3 trajectory_generator.py
python3 ekf_fusion.py
python3 track_to_track_fusion.py
```

Se os três rodarem sem erro, o ambiente está pronto.
---

## 3. Rodar uma validação visual (opcional, mas recomendado)

Cada `validate_module_N.py` gera os gráficos daquele módulo específico:

```bash
python3 validate_modules_1_2.py   # trajetória + layout dos 3 cenários
python3 validate_module_3.py      # sensor RF
python3 validate_module_4.py      # sensor câmera
python3 validate_module_5.py      # linha do tempo intercalada
python3 validate_module_6.py      # GDOP
python3 validate_module_7.py      # EKF (fusão global), semente padrão
python3 validate_module_8.py      # baseline vs. EKF
```

Os PNGs são salvos em `./outputs/` — ajuste o `OUT_DIR` no topo de cada
script se quiser salvar em outro lugar na sua máquina.

---

## 4. Rodar o experimento completo (o resultado principal do trabalho)

### 4.1 Ajustar o número de repetições

Edite `config.py`, bloco `EXPERIMENT`:

```python
EXPERIMENT = {
    "n_monte_carlo_repetitions": 30,   # recomendado (ver análise de poder estatístico)
    "rf_decimation_factor": 1,          # NÃO altere - fidelidade total é obrigatória
    "output_dir": "/caminho/onde/salvar",
}
```

### 4.2 Rodar — três formas

**Sequencial** (mais simples, mais lento, funciona em qualquer sistema):
```bash
python orchestrator.py
```
Cada repetição agora roda o EKF global (3 modos) + track-to-track + baseline -
mais lento que a versão só-UKF/só-um-filtro de antes. Tempo estimado para N=30:
reserve bastante mais que as ~2h15min da versão anterior; meça numa repetição
só antes de disparar o experimento completo.

**Paralela — Linux/Mac (bash):**
```bash
chmod +x run_all_parallel.sh
./run_all_parallel.sh 30 4
```

**Paralela — Windows (ou qualquer sistema, script Python puro):**
```bash
python run_all_parallel.py 30 4
```
Faz a mesma coisa que o script bash (multiprocessing, um arquivo por combinação,
sem condição de corrida), mas usa só bibliotecas nativas do Python — não precisa de
WSL, Git Bash nem nada além do Python já instalado. Ajuste o segundo número (`4`)
para o número de núcleos da sua máquina (no Windows: Gerenciador de Tarefas → Desempenho
→ CPU → "Núcleos", ou rode `python -c "import os; print(os.cpu_count())"` no terminal).

Em ambos os scripts paralelos, cada combinação salva seu próprio arquivo em
`experiment_results/partial_results/scenarioX_repY.csv` (mais um
`gdop_scenarioX_repY.csv`), e a agregação final (`aggregated_metrics.csv`,
`raw_run_summaries.csv`, `divergence_rates.csv`, `gdop_correlations.csv`) é
feita automaticamente ao final.

### 4.3 Gerar os gráficos finais

```bash
python3 plot_experiment_results.py
```

---

## 5. Onde ficam os resultados

Dentro de `experiment_results/` (ou o `output_dir` que você configurou):

| Arquivo | Conteúdo |
|---|---|
| `raw_run_summaries.csv` | Cada execução individual (cenário x modo/arquitetura x repetição) |
| `aggregated_metrics.csv` | Médias e desvios-padrão agregados, com RMSE completo e de regime permanente lado a lado |
| `divergence_rates.csv` | Taxa de divergência por cenário x modo/arquitetura |
| `gdop_correlations.csv` | Correlação GDOP x erro, por repetição, para `fusion` e `track_to_track_ci` |
| `telemetry_scenario{1,2,3}_{fusion,rf_only,camera_only,track_to_track_ci,baseline}.csv` | Telemetria completa (posição, velocidade, incerteza) da repetição 0 |
| `experiment_rmse_by_scenario.png` / `experiment_rmse_by_mode.png` | Gráficos comparativos finais |

---

## 6. Ferramentas auxiliares (fora do fluxo principal)

- `local_track_exporter.py` — tabela de tracks locais por estação (station_id,
  local_track_id, matriz de covariância). Ferramenta avulsa, não faz parte da
  metodologia formal.
- `visualize_fov_and_camera_positions.py` — mapas de FOV + posição estimada só por
  câmera, por estação.
- `spacing_experiment.py` — varredura do afastamento das estações vs. taxa de
  divergência.
- `compare_architectures.py` — EKF global vs. track-to-track, casos críticos.

---

## 7. Ordem recomendada se algo der errado

1. Confirme as dependências (`pip list | grep -E "scipy|statsmodels"`).
2. Rode `python3 config.py` — se der erro de sintaxe, o problema está lá.
3. Rode os módulos na ordem: trajectory_generator -> station_layout_generator ->
   rf_sensor_model -> camera_sensor_model -> detection_association -> gdop_calculator ->
   fusion_models -> ekf_fusion -> track_to_track_fusion -> baseline_estimator ->
   telemetry_exporter -> metrics_evaluator -> orchestrator - cada um depende só
   dos anteriores, então isso ajuda a isolar onde algo quebrou.
4. Se o EKF divergir com alguma semente nova (RMSE de centenas/milhares de
   metros), NÃO é necessariamente bug - veja achados_divergencia_fusao.md §3
   e §8, que documentam esse comportamento e como foi tratado (gate de
   inovação + inicialização por diferenciação de dois pontos). O
   track_to_track_ci costuma ser mais robusto nesses casos (§10.3), mas não é
   imune.

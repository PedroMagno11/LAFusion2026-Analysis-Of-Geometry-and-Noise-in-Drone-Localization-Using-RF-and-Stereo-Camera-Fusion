# Guia de Auditoria — Arquivos do Simulador × Literatura × Comportamento Esperado

## Como usar este documento

Para cada arquivo do projeto, este guia traz: **(1)** a qual módulo/seção da metodologia ele corresponde, **(2)** quais fontes da literatura fundamentam os números que ele usa, **(3)** o que você deve observar ao rodá-lo para confirmar que o comportamento bate com o esperado, e **(4)** como rodá-lo isoladamente. A ideia é que você consiga, arquivo por arquivo, checar "isso que o código faz é mesmo o que o artigo X diz?".

Todas as referências completas (com localização exata dentro da fonte) estão no **Item 12** de `metodologia_fusao_rf_camera_drone.md` — este guia cita só o essencial para conferência rápida.

---

## `config.py` — Parâmetros centrais

Não é um módulo funcional, é onde **todos os números calibrados pela literatura** ficam concentrados, com comentários explicando a origem de cada um. Se você quiser conferir "de onde veio esse valor", comece sempre aqui — cada bloco (`TRAJECTORY`, `STATIONS`, `RF`, `CAMERA`, `UKF`, `EXPERIMENT`) tem comentários apontando a fonte.

**O que verificar:** todo parâmetro numérico deve ter, no comentário acima dele, ou uma citação de artigo, ou a frase "ASSUNÇÃO DECLARADA"/"ASSUNÇÃO DE PROJETO" explicando que não vem de fonte específica. Se encontrar um número sem nenhum dos dois, é um problema de rastreabilidade a reportar.

---

## Módulo 1 — `trajectory_generator.py`

**Corresponde a:** Item 3 da metodologia (trajetória/ground truth).

**Literatura:** nenhuma — é geometria pura, sem calibração empírica. A forma da curva (arco vertical + S lateral) é escolha de projeto sua, documentada como tal.

**O que verificar:**
- `generate_trajectory()` retorna posição em arco: `z` sobe de `z_base` até `z_base+z_amplitude` exatamente na metade do tempo (`s=0.5`) e volta a `z_base` no final.
- Se `y_curve_cycles=2`, a trajetória em `y` deve formar um "S": afasta, volta à rota no meio, afasta do outro lado, volta no fim.
- `trajectory_state_at(t, ...)` deve bater com `generate_trajectory()` em qualquer `t` amostrado (é a mesma fórmula, só que avaliável em instante arbitrário — usada pelos sensores).

**Como rodar:** `python3 trajectory_generator.py` (imprime resumo numérico) ou `python3 validate_modules_1_2.py` (gera `trajectory_views.png`).

---

## Módulo 2 — `station_layout_generator.py`

**Corresponde a:** Item 4 da metodologia (os 3 cenários geométricos).

**Literatura:** a *forma* de cada cenário é desenhada para testar as hipóteses de:
- Herath & Pathirana, *Sensors* 2013 (array linear bearing-only) → Cenário 1.
- Resultado clássico de GDOP mínimo no centro de polígono regular, citado via *Sensors* 16(6):934 (2016) → Cenário 2.

**O que verificar:**
- Cenário 1: as 3 estações devem estar em linha reta (`y` constante), espaçadas ao longo de `x`.
- Cenário 2: as 3 estações devem formar um triângulo equilátero (120° entre si) ao redor do centro da área de voo.
- Cenário 3: posições agora são **proporcionais** à extensão da trajetória (`positions_fraction` no config) — se você mudar `x_start`/`x_end`, as estações do Cenário 3 devem se mover proporcionalmente, não ficar paradas (isso foi uma correção que fizemos no meio do projeto).

**Como rodar:** `python3 validate_modules_1_2.py` → gera `scenario_{1,2,3}_layout.png`.

---

## Módulo 3 — `rf_sensor_model.py`

**Corresponde a:** Item 5 da metodologia (sensor RF).

**Literatura e valores a conferir:**

| O que checar | Valor esperado | Fonte |
|---|---|---|
| Erro angular (`error_std_deg`) perto de 30 m de distância | ≈ 5° (piso) | Han & Jang, *Sensors* 2025 |
| RF mede **só azimute**, nunca elevação | `meas_elevation_deg` não deve existir na saída do RF | Han & Jang 2025 (arranjo circular horizontal de 6 antenas) |
| Banda do sinal usada no cálculo de ruído térmico | 10 MHz | Han & Jang 2025 (vídeo DJI Phantom 4 Pro) |
| Ganho de antena de recepção | 9 dBi | Han & Jang 2025 (antena PM-PP09) |
| Potência de transmissão do drone | -23 dBm | Jeong et al., *WCMC* 2018 |
| Taxa de atualização | ~833 Hz (ciclo de 1,2 ms) | Han & Jang 2025 |
| Erro angular cresce com a distância | Deve ser aproximadamente **linear** com `d` (não constante) | Derivado de erro∝1/√SNR [patente US12560669] + FSPL |

**O que verificar rodando o código:**
- `angular_error_std_deg(30, RF)` deve retornar ≈5°.
- O erro deve **crescer** conforme a distância aumenta (gráfico "Modelo de degradação do erro AoA" em `rf_module_validation.png` deve ser uma reta crescente).
- Sanidade estatística: gerando muitas amostras de ruído numa distância fixa, o desvio-padrão amostral deve bater com o teórico (diferença <1%).

**Como rodar:** `python3 rf_sensor_model.py` (resumo numérico) ou `python3 validate_module_3.py` (gera `rf_module_validation.png` com 4 painéis, incluindo a checagem estatística).

---

## Módulo 4 — `camera_sensor_model.py`

**Corresponde a:** Item 6 da metodologia (sensor de câmera).

**Literatura e valores a conferir:**

| O que checar | Valor esperado | Fonte |
|---|---|---|
| Erro de bearing (azimute/elevação) | ≈ 0,08° (bem mais preciso que o RF) | Calibrado via resolução/FOV da câmera (VIODE dataset) |
| Erro de profundidade cresce com `distância²` | Curva quadrática, não linear | Fórmula clássica ΔZ=(z²/(B·f))·Δd |
| Δd (erro de disparidade) | ≈ 0,54 px | Retro-calculado de Sharma, Jain & Kothari 2022 (23% de erro a 8 m) |
| Taxa de quadros | 30 fps | Relatório Counter-UAS 101 (EO/IR) |
| Campo de visão (FOV) | 90°, apontado para o centro da trajetória | Config. de câmera estéreo padrão AirSim (VIODE) |
| Câmera fornece bearing **e** profundidade (ao contrário do RF) | `meas_distance_m` deve estar preenchido nas detecções de câmera | Pipeline estéreo padrão |

**O que verificar rodando o código:**
- `bearing_error_std_deg(CAMERA)` deve retornar ≈0,08°.
- `depth_error_std_m(d, CAMERA)` deve crescer com `d²` — a 100 m o erro deve ser ~4x maior que a 50 m (não 2x).
- Nem todo instante de tempo deve ter detecção de todas as estações — algumas ficam fora do FOV (visível no gráfico de "instantes com drone dentro do FOV", que mostra lacunas).
- No gráfico de FOV (`scenario_1_fov.png`), os setores de 90° desenhados sobre o mapa devem coincidir com onde cada estação realmente detecta o drone.

**Como rodar:** `python3 camera_sensor_model.py` ou `python3 validate_module_4.py` (gera `camera_module_validation.png`), ou `python3 visualize_fov_and_camera_positions.py` (gera os mapas de FOV e exporta `camera_position_estimates_scenario_*.csv`).

---

## Módulo 5 — `detection_association.py`

**Corresponde a:** Item 13.1 (Módulo 5) e Item 13.3 (estruturas de dados) da metodologia.

**Literatura:** nenhuma — é só padronização de esquema de dados, sem parâmetro numérico novo.

**O que verificar:**
- No fluxo unificado (`associate_detections()`), linhas com `sensor_type='RF'` devem ter `meas_distance_m` e `meas_elevation_deg` como `NaN`.
- Linhas com `sensor_type='camera'` devem ter todos os campos preenchidos.
- O fluxo deve estar ordenado por `timestamp` (`merged.timestamp.is_monotonic_increasing` deve ser `True`).
- Proporção de detecções RF:câmera deve refletir a proporção das taxas de amostragem (~833:30 ≈ 27,7:1).

**Como rodar:** `python3 detection_association.py` ou `python3 validate_module_5.py` (gera `detection_timeline.png`).

---

## Módulo 6 — `gdop_calculator.py`

**Corresponde a:** Item 4 (fundamentação dos 3 cenários) e Item 9 (métricas) da metodologia.

**Literatura e valores a conferir:**

| O que checar | Valor esperado | Fonte |
|---|---|---|
| Fórmula de GDOP bearing-only | GDOP = √(traço((HᵀH)⁻¹)) | Herath & Pathirana, *Sensors* 2013 |
| Cenário 2 (circular): GDOP mínimo | Exatamente no **centro geométrico** do círculo formado pelas estações | *Sensors* 16(6):934 (2016) |
| Cenário 1 (linear): degradação nas bordas | GDOP alto ao longo do eixo das estações (efeito "endfire") | arXiv:2608.10784 (broadside vs. endfire) |

**O que verificar rodando o código:**
- No mapa de calor do Cenário 2 (`gdop_heatmap_scenario_2.png`), a região mais clara (GDOP baixo) deve estar centrada exatamente no meio do triângulo formado pelas 3 estações.
- No mapa de calor do Cenário 1 (`gdop_heatmap_scenario_1.png`), devem aparecer "asas" escuras (GDOP alto) se estendendo ao longo do eixo `y=0` (onde as estações estão), fora da faixa onde a trajetória passa.
- O GDOP do Cenário 2 deve ter **menor variação** ao longo do tempo que o Cenário 1 (compare os desvios em `gdop_along_trajectory.png`).

**Como rodar:** `python3 gdop_calculator.py` (resumo numérico) ou `python3 validate_module_6.py` (gera os 4 gráficos: trajetória temporal + 3 mapas de calor).

---

## Módulo 7 — `ekf_fusion.py` (antes `ukf_fusion.py` - UKF removido, ver achados_divergencia_fusao.md §11)

**Corresponde a:** Item 8 da metodologia (arquitetura de fusão) e Item 8c (resultado do estudo Monte Carlo).

**Literatura:**
- UKF: formulação padrão de Julier/Uhlmann via biblioteca `filterpy`.
- Correção de dimensionalidade: RF mede só azimute (1D); câmera mede azimute+elevação+distância (3D) — consequência direta da correção documentada no Módulo 3.
- Densidade de ruído de processo: calibrada pela **própria aceleração de pico da trajetória sintética** (~1,02 m/s²), não de um artigo externo.

**O que verificar:**
- `hx_rf()` deve retornar um array de **1 elemento** (só azimute); `hx_camera()` deve retornar **3 elementos** (azimute, elevação, distância).
- No diagnóstico de NIS (`ukf_nis_diagnostic.png`, gerado por `validate_module_7.py`), a média do NIS do RF deve ficar próxima de 1 (graus de liberdade=1); a da câmera próxima de 3 (mas pode estar um pouco acima, ~4,3, indicando leve otimismo do filtro quanto à câmera — já documentado como limitação conhecida).
- **Importante:** ao rodar com sementes de ruído diferentes (não só a padrão), o RMSE da fusão pode variar bastante (ver Item 8c) — isso é um comportamento **esperado e documentado**, não um bug. Não estranhe se uma repetição específica mostrar fusão pior que câmera-only.
- O campo `gate_rejected` deve ficar em torno de 0-2% das detecções na maioria dos casos; se você ver taxas de rejeição muito altas (>20%), é sinal de que o filtro está tendo dificuldade de convergir naquela execução específica.

**Como rodar:** `python3 ekf_fusion.py` (resumo numérico) ou `python3 validate_module_7.py` (gera os gráficos de validação do EKF).

**Nota de transparência:** este é o módulo com a história de depuração mais longa do projeto — teve divergência catastrófica com certas sementes de ruído, corrigida via gate de inovação. Veja o Item 8c da metodologia para o relato completo, incluindo o que foi tentado e não funcionou.

---

## Módulo 8 — `baseline_estimator.py`

**Corresponde a:** Item 8 da metodologia (baseline comparativo).

**Literatura:** multilateração por mínimos quadrados ponderados — método clássico, sem calibração empírica própria (usa os mesmos ruídos já calibrados nos Módulos 3 e 4).

**O que verificar:**
- Cada "época" (ancorada em cada instante de detecção de câmera) deve combinar as detecções de câmera daquele instante **e** as detecções RF mais próximas no tempo.
- O baseline **não deve ter campos de velocidade** (é um método sem estado/memória) — isso deve aparecer como `NaN` na exportação (Módulo 9).
- No estudo Monte Carlo (Item 8c), o baseline deve ser o método com **menor variância entre repetições** (desvio-padrão pequeno, ~0,04-0,3 m) — precisamente por não depender de inicialização.

**Como rodar:** `python3 baseline_estimator.py` ou `python3 validate_module_8.py` (gera `baseline_vs_ukf_comparison.png`).

---

## Módulo 9 — `telemetry_exporter.py`

**Corresponde a:** Item 8b da metodologia (esquema de saída de tracking).

**Literatura:** esquema de campos inspirado no padrão MAVLink (`GLOBAL_POSITION_INT`, `HIGH_LATENCY2`).

**O que verificar:**
- O CSV/JSON exportado deve ter exatamente as colunas listadas no Item 8b: `timestamp, x, y, z, vx, vy, vz, speed, course_over_ground, vertical_speed, pos_uncertainty, vel_uncertainty, scenario_id, sensor_mode` + campos `ground_truth_*`.
- Para o baseline geométrico, os campos de velocidade/incerteza devem estar como `NaN` (documentado, não é erro).
- `course_over_ground` é o rumo do **deslocamento** (derivado da velocidade), não o heading do drone — não confundir os dois (distinção que fizemos explicitamente porque não há sensor de atitude no simulador).

**Como rodar:** `python3 telemetry_exporter.py` (gera CSVs de exemplo em `/mnt/user-data/outputs/`).

---

## Módulo 10 — `metrics_evaluator.py`

**Corresponde a:** Item 9 da metodologia (métricas de avaliação).

**Literatura:** RMSE e testes estatísticos (Kruskal-Wallis) são métodos padrão, sem calibração específica.

**O que verificar:**
- `correlate_gdop_rmse()` deve retornar uma correlação de Pearson **positiva e significativa** entre o GDOP teórico (Módulo 6) e o erro empírico — no Cenário 1, obtivemos r=0,60 (p=0,005). Essa é a validação central do trabalho: confirma que a teoria de GDOP prediz corretamente onde o erro será maior/menor.
- `fusion_gain_summary()` decompõe o ganho total em "ganho multi-sensor" (baseline vs. sensor isolado) e "ganho de filtragem temporal" (UKF vs. baseline) — no Cenário 1 (execução única), ficou ~17%/83% respectivamente.

**Como rodar:** `python3 metrics_evaluator.py` (roda um exemplo completo com a semente padrão).

---

## Módulo 11 — `orchestrator.py`, `run_single_repetition.py`, `plot_experiment_results.py`

**Corresponde a:** Item 13.2 (fluxo de execução) e Item 8c (resultado do experimento Monte Carlo) da metodologia.

**O que verificar:**
- `orchestrator.py` roda a matriz completa (3 cenários × N repetições × 4 modos) e salva `aggregated_metrics.csv` + `raw_run_summaries.csv`.
- **Checagem importante:** `EXPERIMENT["rf_decimation_factor"]` deve estar em `1` (fidelidade total) — testamos decimar para acelerar e isso causou instabilidade adicional no filtro (documentado no Item 8c); não decimar a menos que você entenda esse risco.
- O resultado agregado deve mostrar: RF-only consistentemente pior que todos; baseline geométrico com o menor desvio-padrão entre repetições; fusão e câmera-only com variância alta e sobreposta (ver tabela no Item 8c).

**Como rodar:** `python3 orchestrator.py` (tudo de uma vez, ~13-15 min) ou `python3 run_single_repetition.py <cenário> <repetição>` (uma combinação por vez, para rodar aos poucos).

---

## Scripts auxiliares de visualização

| Arquivo | O que gera | Módulo/Item correspondente |
|---|---|---|
| `validate_modules_1_2.py` | Trajetória + layout dos 3 cenários | Módulos 1-2, plano de validação passos 1-2 |
| `validate_module_3.py` | Validação estatística e visual do RF | Módulo 3, passo 3 |
| `validate_module_4.py` | Validação estatística e visual da câmera | Módulo 4, passo 4 |
| `validate_module_5.py` | Linha do tempo intercalada RF+câmera | Módulo 5 |
| `validate_module_6.py` | GDOP ao longo do tempo + mapas de calor | Módulo 6, passo 5 |
| `validate_module_7.py` | Trajetória estimada, erro, diagnóstico NIS | Módulo 7, passo 7 |
| `validate_module_8.py` | Comparação baseline vs. UKF vs. sensores isolados | Módulo 8, passo 6 |
| `visualize_fov_and_camera_positions.py` | Mapas de FOV + posições estimadas só por câmera | Extensão do Módulo 4 |
| `plot_experiment_results.py` | Gráficos finais do experimento Monte Carlo | Módulo 11 |

---

## Checklist rápido de conferência (resumo executivo)

Se você só tiver tempo de checar alguns números-chave contra a literatura, priorize estes:

- [ ] Erro angular RF ≈ 5° perto de 30 m, crescendo com a distância (`rf_sensor_model.py`) — Han & Jang 2025
- [ ] RF só mede azimute, nunca elevação (`rf_sensor_model.py`, `detection_association.py`) — Han & Jang 2025
- [ ] Erro de profundidade da câmera cresce com `distância²` (`camera_sensor_model.py`) — fórmula estéreo clássica
- [ ] GDOP mínimo no centro do círculo, Cenário 2 (`gdop_calculator.py`) — *Sensors* 2016
- [ ] Correlação GDOP × erro empírico positiva e significativa (`metrics_evaluator.py`) — validação própria do trabalho
- [ ] RF-only consistentemente o pior método no estudo Monte Carlo (`orchestrator.py`) — esperado, sensor menos informativo
- [ ] Todo parâmetro em `config.py` tem citação ou "ASSUNÇÃO DECLARADA" no comentário — princípio metodológico geral

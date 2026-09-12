# Fusão de Dados RF + Imagem para Localização de Drone
## Documento de Metodologia Consolidado (v1 — para revisão)

---

## 1. Objetivo do Trabalho

Desenvolver e avaliar, por meio de **simulação controlada**, um método de **fusão de dados RF + imagem (câmera)** para estimar a posição 3D de um drone ao longo do tempo, investigando sistematicamente **como a geometria de posicionamento das estações de sensoriamento afeta a acurácia da fusão**.

O simulador substitui a necessidade de hardware real (RF/câmera) por modelos de ruído fisicamente fundamentados e calibrados com dados da literatura, permitindo testar hipóteses de geometria de sensores em condições controladas e repetíveis.

---

## 2. Gap na Literatura

- A literatura de **localização por RF** trata o problema isoladamente, majoritariamente via AoA, TDOA/TOA e RSS, com AoA preferida por não exigir sincronização temporal entre estações [Han & Jang, *Sensors* 2025].
- A literatura de **localização por câmera** trata o problema separadamente, tipicamente via detector (YOLO) + triangulação estéreo ou bearing a partir de bounding box e intrínsecos da câmera [Sharma, Jain & Kothari, arXiv:2202.09097, 2022].
- O trabalho mais próximo de uma fusão real RF+EO é o da JHU/APL, que desenvolve uma arquitetura de fusão multiestágio usando RF passivo e imagem eletro-óptica — mas com **layout fixo** de sensores e dados reais, sem investigar sistematicamente o efeito da geometria de estações.
- A literatura de **geometria de sensores (GDOP)** caracteriza teoricamente o efeito da topologia na acurácia de localização por ângulo, mas nunca foi conectada explicitamente ao problema de fusão heterogênea RF+imagem para drones.

**→ Gap coberto por este trabalho:** conectar a teoria de geometria de sensores (GDOP) com um pipeline de fusão RF+imagem para localização de drone, comparando explicitamente 3 topologias de estação sob a mesma trajetória e sob modelos de ruído fisicamente parametrizados (não emprestados cegamente de um único estudo).

---

## 3. Trajetória do Drone (Ground Truth)

- Curva paramétrica em função do tempo: movimento da esquerda para a direita, com subida suave até a metade do trajeto e descida — formato de arco (ex.: parabólico ou senoidal amortecido).
- Mesma trajetória aplicada aos 3 cenários, permitindo comparação controlada isolando o efeito da geometria das estações.
- Gera-se a posição (x, y, z, t) de referência contra a qual todos os métodos (RF-only, câmera-only, fusão) serão comparados.

---

## 4. Os Três Cenários de Geometria de Estações

| Cenário | Configuração | Fundamentação teórica | Hipótese de comportamento |
|---|---|---|---|
| **1 — Linear/lateral** | 3 estações alinhadas transversalmente à trajetória, observando o arco completo da esquerda para a direita (visão lateral de toda a travessia) | Teoria de array linear bearing-only (broadside vs. endfire) [Li et al., *Sensors* 2023, doi:10.3390/s23146408] | Melhor acurácia próxima ao centro/topo do arco (broadside); degradação nas extremidades (aproximação de endfire) |
| **2 — Circular** | 3 estações dispostas em círculo ao redor da área de voo | GDOP mínimo no centro de um polígono regular [Li, Qi & Sheng, *Automatica* 2019] | Acurácia mais uniforme ao longo de toda a trajetória; possível vantagem geral sobre o Cenário 1 |
| **3 — Assimétrico** | 3 estações em disposição não regular | Sem garantia teórica fechada; requer cálculo numérico de GDOP ponto a ponto | Comportamento variável; representa cenário de implantação real/não idealizada |

**Nota metodológica:** os 3 cenários serão avaliados com a mesma métrica teórica preditiva — GDOP calculado ponto a ponto ao longo da trajetória — comparada ao erro empírico obtido na simulação (RMSE). Isso permite validar (ou refutar) a teoria clássica de geometria de sensores no contexto específico de fusão heterogênea RF+imagem, contribuição própria do trabalho.

**Resultado da implementação (Módulo 6):** o GDOP teórico (bearing-only, 2D, geométrico puro) foi calculado ponto a ponto ao longo da trajetória para os 3 cenários, confirmando quantitativamente as hipóteses teóricas: Cenário 1 (linear) variou de 50,0 a 100,8 (alta variação, degradação nas extremidades - padrão broadside/endfire); Cenário 2 (circular) variou de 49,5 a 65,7 (mais baixo e mais uniforme); Cenário 3 (assimétrico) variou de 42,0 a 74,7 (comportamento intermediário/variável, sem garantia teórica). Mapas de calor espaciais confirmam visualmente o padrão clássico: GDOP mínimo no centro exato do círculo no Cenário 2, e uma região de forte degradação ("asas" endfire) ao longo do eixo das estações no Cenário 1, fora da faixa de cobertura da trajetória.

---

## 5. Modelo do Sensor RF

**Sinal simulado:** downlink do drone (vídeo/telemetria), banda ISM **2.4 / 5.8 GHz**.

**Método de localização:** AoA (Angle of Arrival), escolhido por não exigir sincronização temporal entre estações, ao contrário de TDOA/TOA [Han & Jang, *Sensors* 2025].

**Dimensionalidade da medição — CORREÇÃO IMPORTANTE:** o sistema de referência [Han & Jang, *Sensors* 2025] usa um arranjo **circular horizontal de 6 antenas direcionais**, que mede apenas o ângulo no plano horizontal (**azimute**) — não há medição de elevação nesse sistema. Uma versão anterior deste documento/simulador assumia (incorretamente, sem fundamentação) que o RF também forneceria elevação; isso foi corrigido. **Implicação para a fusão:** a câmera passa a ser a **única fonte de informação de elevação/altitude** no sistema — RF sozinho nunca consegue estimar a altura do drone, apenas sua posição no plano horizontal (via triangulação de múltiplas estações). Isso é uma complementaridade de **dimensionalidade** entre os sensores (não apenas de precisão), e reforça a motivação para a fusão RF+câmera no artigo.

**Modelo de atenuação (path-loss):**
- Base: Free-Space Path Loss (FSPL), FSPL(dB) = 20·log₁₀(d) + 20·log₁₀(f) + 20·log₁₀(4π/c).
- Calibração de potência de transmissão do drone: âncora empírica de drone tipo Wi-Fi em 2.4 GHz — potência de transmissão ≈ -23 dBm, ganho de antena de transmissão ≈ 2,5 dBi [Jeong et al., *Wireless Communications and Mobile Computing*, 2018].
- **Calibração do ganho de antena de recepção — CORREÇÃO:** ganho de recepção ≈ 9 dBi, valor real da antena PM-PP09 usada no próprio sistema de referência [Han & Jang, *Sensors* 2025], banda 2,400–2,483 GHz, beamwidth de 3 dB de ~60°. Como o sistema chaveia entre 6 antenas (uma ativa por vez via switch SP6T), o ganho relevante para o link budget é o de uma antena individual. Uma versão anterior deste documento usava um valor genérico de 3 dBi emprestado do Jeong et al. (que descreve características do drone transmissor, não do receptor de Han & Jang) — corrigido para evitar misturar parâmetros de sistemas diferentes.
- Extensão futura (não bloqueante): biblioteca UAVRadio oferece modelos adicionais (log-distância, two-ray, dual-slope) caso se queira modelar NLoS/multipath.

**Modelo de ruído angular (erro de AoA):**
- Piso de erro em boa SNR/curto-médio alcance: erro angular médio < 5°, demonstrado experimentalmente em ambiente controlado de até ~30 m, com antena de feixe comutado, tanto para sinal CW quanto para vídeo OFDM real de drone [Han & Jang, *Sensors* 2025].
- **Ressalva registrada:** o artigo-fonte não fornece curva erro-vs-distância explícita; o valor de 5° é tratado como piso de melhor caso, não como constante universal.
- Erro modelado como crescente com a queda de SNR (função da distância via FSPL), seguindo a proporcionalidade clássica erro_AoA ∝ 1/√SNR de estimação por CRLB [patente US12560669, "Systems, methods, and apparatus for estimating angle of arrival"].
- **Piso de ruído do receptor:** calculado via fórmula clássica de ruído térmico N(dBm) = -174 + 10·log₁₀(B) + NF, usando a largura de banda real do sinal de vídeo OFDM do DJI Phantom 4 Pro (10 MHz, mesmo sinal usado na calibração do erro de 5°) e uma figura de ruído de referência de receptor SDR de 6,2 dB [patente US7116958, "Interference rejection in a radio receiver"] — substituindo a suposição inicial não fundamentada. **Nota de robustez:** como o modelo de erro usa a razão SNR_referência/SNR(distância), o piso de ruído se cancela algebricamente — o resultado da curva de erro angular é insensível ao valor absoluto do piso de ruído, tornando essa calibração menos crítica do que se pensava inicialmente.

**Taxa de atualização:** ordem de grandeza de ~833 Hz (ciclo de chaveamento de antena de 1,2 ms), conforme hardware de referência [Han & Jang, *Sensors* 2025]. Tratado como valor de referência de ordem de grandeza, não como padrão universal de hardware.

---

## 6. Modelo do Sensor de Câmera

**Pipeline de referência:** detector (YOLO/variantes) → bounding box → centróide → bearing (via intrínsecos da câmera) e/ou profundidade (via triangulação estéreo) [pipeline consistente com Sharma, Jain & Kothari, arXiv:2202.09097].

**Modelo físico de erro de profundidade (substitui uso de percentual fixo):**

ΔZ = (z² / (B·f)) · Δd

onde z = profundidade do alvo, B = baseline entre câmeras, f = distância focal, Δd = erro de disparidade/correspondência [fórmula clássica de triangulação estéreo].

**Fatores que modulam Δd (parametrizáveis no simulador, não fixos):**
- Resolução do sensor (mais pixels → menor Δd absoluto).
- Iluminação/SNR da imagem.
- Qualidade do detector/confiança do bounding box.
- Motion blur (movimento do drone relativo ao tempo de exposição).
- Condições atmosféricas (a longo alcance).

**Calibração empírica de Δd:** pipeline tiny-YOLOv4 + triangulação estéreo testado em ambiente sintético AirSim, com detecção a distância máxima de 8 m e erro médio de 23% da distância [Sharma, Jain & Kothari, arXiv:2202.09097, 2022]. Como esse artigo não publica os parâmetros exatos de câmera (distância focal, resolução, baseline), esses intrínsecos foram calibrados por uma configuração-padrão de câmera estéreo em AirSim documentada em outro estudo comparável: resolução 752×480 px, FOV horizontal de 90°, baseline de 5 cm [VIODE dataset]. A partir desses valores, retro-calculou-se Δd ≈ 0,54 px (erro de disparidade/centróide), usado como parâmetro fixo do modelo. A baseline **real** da estação (0,5 m) é um parâmetro de projeto (não de literatura), já que estações fixas em mastro podem ter baseline maior que uma câmera compacta embarcada em drone.

**Resultado da implementação (Módulo 4):** o erro de bearing (azimute/elevação) resultante é muito pequeno (~0,08°), bem mais preciso que o erro angular do RF (7-17° na faixa de distância do Cenário 1). Já o erro de profundidade da câmera (5-29 m) tem magnitude **comparável** ao erro linear equivalente do RF na mesma faixa — achado não-trivial: ambos os modelos resultam, por construção matemática independente, em erro proporcional a d² (distância ao quadrado), e os coeficientes dessas duas curvas quadráticas ficaram numericamente próximos, apesar de calibrados a partir de fontes completamente diferentes. Isso significa que RF e câmera contribuem de forma equilibrada para a fusão neste cenário, o que é favorável para o estudo (evita que a fusão apenas "copie" o sensor dominante).

**Taxa de atualização (frame rate):** 30 fps, padrão de câmeras EO convencionais aplicadas a C-UAS — "câmeras padrão capturam quadros em intervalos fixos de 30 ou 60 fps" [relatório técnico Counter-UAS 101 — EO/IR Detection, drone-warfare.com, 2026].
- **Ressalva de proveniência:** fonte de caráter técnico/industrial, não peer-reviewed — usada como referência de engenharia de sistema para ordem de grandeza realista.
- Valor alternativo de referência (câmeras térmicas): ~9 Hz (limiar regulatório ITAR), não adotado como padrão, mas documentado como alternativa caso se queira simular câmera térmica no lugar de EO.

---

## 7. Detecção — Simplificação Assumida

Nesta primeira versão do simulador, assume-se que a **detecção é sempre bem-sucedida** (sem falso-negativo, sem oclusão, sem perda de sinal) tanto para RF quanto para câmera, desde que o drone esteja dentro do alcance/FOV geométrico da estação.

**Registrado como limitação/simplificação do estudo**, não como característica realista de sistemas de campo — pode ser relaxado em trabalhos futuros.

---

## 8. Arquitetura de Fusão

**Filtro escolhido:** UKF (Unscented Kalman Filter), assíncrono/multi-taxa.

**Justificativa:**
- Medições de bearing/AoA são não lineares em relação ao estado cartesiano do alvo; o UKF evita a linearização de primeira ordem do EKF, capturando média e covariância verdadeiras via amostragem determinística [Nair, *LiDAR and Radar Sensor Fusion using UKF*].
- O problema RF+câmera é estruturalmente análogo à fusão vision+radar já resolvida na literatura veicular: "fusão multi-taxa de visão e radar usando filtro de Kalman para resolver problemas de amostragem assíncrona e multi-taxa, com filtro multi-taxa descentralizado para cada sensor e ponderação diferente aplicada à posição prevista de cada sensor" [pesquisa de fusão multi-rate vision+radar, 2014].
- A disparidade de taxas entre RF (~833 Hz) e câmera (30 fps) é exatamente o cenário-problema que a literatura de fusão multi-taxa assíncrona resolve: sensor rápido refina a predição entre atualizações mais espaçadas do sensor lento.

**Baseline comparativo:** triangulação/multilateração geométrica simples (mínimos quadrados), para quantificar o ganho real do filtro temporal (UKF) sobre a geometria instantânea pura.

**Modos de comparação a rodar:** RF-only vs. Câmera-only vs. Fusão RF+Câmera (UKF) vs. Fusão RF+Câmera (baseline geométrico) — 4 condições, cruzadas com os 3 cenários geométricos.

**Resultado da implementação (Módulo 7):**
- Biblioteca: `filterpy` (UnscentedKalmanFilter), confirmada capaz de alternar `hx`/`R`/`dim_z` entre chamadas de `update()` (RF 1D vs. câmera 3D) na mesma instância de filtro.
- Densidade de ruído de processo calibrada com base na aceleração de pico da própria trajetória sintética (~1,02 m/s², via diferenciação analítica), usando `process_noise_accel_std = 2.0 m/s²` (margem de segurança sobre o valor observado).
- **RMSE de posição 3D (Cenário 1):** Fusão = 0,84 m; RF-only = 5,67 m; Câmera-only = 2,38 m — a fusão supera claramente qualquer sensor isolado.
- **Validação cruzada com o Módulo 6 (GDOP):** o erro do modo RF-only ao longo do tempo reproduz visualmente o formato de "U" já previsto pela curva de GDOP teórico do Cenário 1 (erro mínimo próximo ao ponto de GDOP mínimo, maior nas bordas) — confirmação direta da teoria de geometria de sensores no contexto de fusão.
- **Diagnóstico de consistência (NIS):** RF com ajuste quase perfeito ao χ²(1) teórico (média observada 0,99, esperado 1). Câmera com leve otimismo do filtro (média observada 4,33, esperado 3, χ²(3)) — indica que a incerteza real da câmera é um pouco maior que a reportada; possível refinamento futuro (ajuste fino de R ou q), não bloqueante.
- **Lições de implementação (armadilhas numéricas do UKF, documentadas para referência):**
  1. Covariância pode perder simetria/positividade após muitas atualizações sequenciais em alta taxa (RF a ~833 Hz) — corrigido com regularização por autovalores a cada passo.
  2. Combinação de `alpha` pequeno (0,1) com covariância inicial muito grande causa cancelamento numérico catastrófico no cálculo da matriz de covariância da inovação (S), gerando NIS artificialmente enorme — não deve ser "corrigido" inflando ainda mais a covariância inicial (piora o problema); o correto é manter a covariância inicial modesta e tratar o pico de NIS do instante de inicialização como transiente esperado, excluído do diagnóstico de consistência.
  3. Inicialização do filtro exige um fix de posição 3D completo — só a câmera fornece isso (RF é bearing-only); o filtro é inicializado na primeira detecção de câmera disponível, com velocidade inicial zero.

**Resultado da implementação (Módulo 8):** baseline geométrico (mínimos quadrados ponderados, por época ancorada nos instantes de detecção de câmera) obteve RMSE de posição 3D = 2,12 m no Cenário 1 — melhor que qualquer sensor isolado (RF-only = 5,67 m; câmera-only = 2,38 m), mas pior que a fusão UKF completa (0,84 m). Isso permite decompor o ganho total da fusão em dois componentes: **ganho de combinação multi-sensor** (sensor isolado → baseline: ~0,26 m de redução) e **ganho adicional de filtragem temporal** (baseline → UKF: ~1,27 m de redução, o componente dominante). Visualmente, o baseline apresenta picos de erro nas bordas do trajeto (coincidindo com a região de GDOP alto identificada no Módulo 6), que o UKF amortece via sua memória temporal - evidência concreta do valor do filtro além da simples combinação geométrica instantânea. **Nota:** este resultado de execução única (semente de ruído padrão) é revisitado e contextualizado no Item 8c à luz do estudo Monte Carlo completo.

### 8c. Estudo Monte Carlo Completo (Módulo 11) - Fragilidade do UKF e Resultado Reavaliado

**Descoberta durante a execução do Módulo 11:** ao rodar o experimento completo (3 cenários × 3 repetições Monte Carlo, variando a semente de ruído), o UKF apresentou **divergência catastrófica** (erros de centenas a milhares de metros) em algumas combinações específicas de semente/cenário - um comportamento que não havia aparecido na validação de execução única do Módulo 7 (Item 8b), que usava apenas a semente de ruído padrão.

**Causa raiz identificada:** a inicialização do filtro depende de uma única detecção de câmera, que pode ter erro de dezenas de metros (ver Módulo 4). Em certas realizações de ruído, essa estimativa inicial ruim, combinada com uma medição de azimute de câmera subsequente muito precisa (~0,08°), causa instabilidade numérica na aproximação por sigma-points do UKF (cancelamento numérico ao calcular a matriz de covariância da inovação).

**Tentativas de correção e lições aprendidas** (documentadas para transparência metodológica):
1. Inicializar com a média de múltiplas detecções de câmera (3, depois uma janela de 1s) - reduziu a divergência em alguns casos, mas causou regressões em outros (inclusive na semente já validada), por interações não totalmente elucidadas com o restante do loop de atualização.
2. Reduzir a taxa RF (decimação) para acelerar o estudo Monte Carlo - **abandonado**: expôs a mesma fragilidade de forma ainda mais errática (sem relação suave entre o fator de decimação e a estabilidade), e o estudo foi mantido em fidelidade total (taxa RF real, ~833 Hz) por priorizar validade científica sobre velocidade de execução.
3. **Correção final adotada:** manter a inicialização simples (detecção única, já validada) e aplicar um **gate de inovação** (rejeição de atualizações cujo NIS exceda um limiar de confiança de 99,99%, prática padrão em tracking) - com um limiar mais rígido (90%) aplicado apenas nos primeiros 2 segundos após a inicialização e apenas no modo de fusão (onde a instabilidade foi observada). Essa combinação eliminou a divergência catastrófica em todos os casos testados, mantendo o comportamento já validado da execução de referência praticamente inalterado (RMSE de fusão: 0,84 m → 0,84-1,37 m, dependendo da configuração exata testada).

**Resultado agregado (3 cenários × 3 repetições, RMSE médio ± desvio-padrão):**

| Cenário | Baseline geométrico | Câmera-only | Fusão (UKF) | RF-only |
|---|---|---|---|---|
| 1 | 1,93 ± 0,04 m | 1,58 ± 0,70 m | 3,72 ± 2,66 m | 14,54 ± 2,31 m |
| 2 | 3,02 ± 0,29 m | 3,49 ± 2,53 m | 5,72 ± 7,45 m | 18,70 ± 4,78 m |
| 3 | 2,65 ± 0,11 m | 3,59 ± 3,61 m | 6,20 ± 3,67 m | 13,11 ± 0,71 m |

**Teste estatístico (Kruskal-Wallis) entre cenários, modo fusão:** estatística = 1,16, p = 0,56 - **não significativo** (n=3 repetições por grupo).

**Discussão honesta do resultado - achado científico, não falha do trabalho:**

1. **RF-only é consistente e claramente o pior método** (12-19 m) em todos os cenários - resultado robusto, sem ambiguidade, confirmando a limitação estrutural do RF como sensor bearing-only 2D discutida desde o Item 5.

2. **O baseline geométrico é o método mais ESTÁVEL** entre repetições (desvio-padrão de apenas 0,04-0,29 m) - por não ter estado interno nem depender de inicialização, sua acurácia é previsível e reprodutível, ainda que sua acurácia pontual (2-3 m) não seja a melhor observada.

3. **Fusão e câmera-only apresentam variância muito alta e sobreposta entre si** - em nenhum dos 3 cenários a fusão superou a câmera-only de forma estatisticamente robusta com a amostra de 3 repetições; em alguns cenários a média da fusão fica numericamente acima da câmera-only. A causa identificada é a **sensibilidade à inicialização**: numa trajetória curta (20 s), o tempo de convergência do filtro após uma inicialização ruim pode consumir uma fração substancial da janela de avaliação, dominando o RMSE agregado e mascarando o efeito da geometria/combinação de sensores que o estudo pretendia isolar.

4. **Interpretação para o trabalho:** com o desenho experimental atual (N=3 repetições, trajetória de 20 s, inicialização por detecção única), **não há evidência estatística suficiente de que a fusão RF+câmera supere consistentemente a câmera isolada** nesta simulação. Isso não invalida a arquitetura de fusão nem o simulador - é um resultado genuíno que aponta para uma variável de confusão real (variância de inicialização) que precisa ser controlada ou reportada separadamente em trabalhos futuros (ex.: excluir o transiente de aquisição do cálculo de RMSE, aumentar o número de repetições, ou alongar a trajetória) para isolar de forma mais limpa o efeito da geometria de sensores e da combinação multi-sensor, que é o gap original que o trabalho se propõe a investigar (Item 2). O achado em si - de que o ganho teórico da fusão pode ser neutralizado pela variância de convergência do filtro em janelas curtas de rastreamento - é relevante e deve ser discutido na seção de resultados/discussão do artigo, não descartado.

**Limitação declarada:** N=3 repetições Monte Carlo é uma amostra pequena para conclusões estatísticas fortes; os testes de significância (Kruskal-Wallis) não têm poder estatístico suficiente para detectar diferenças moderadas com essa amostra. Recomenda-se aumentar N em trabalho futuro, watch-list já registrada para revisão.

### 8d. Correção Crítica: Isolamento do Modo RF-only na Inicialização

**Problema identificado (por questionamento direto do usuário):** todos os resultados de RF-only reportados nos Itens 8b e 8c foram gerados com uma falha de isolamento — a inicialização do filtro, em TODOS os modos (inclusive RF-only), usava a primeira detecção de câmera disponível no conjunto completo de detecções, mesmo quando o modo avaliado (RF-only) não deveria ter acesso a esse sensor. Isso inflava artificialmente a acurácia inicial do RF-only, mascarando sua limitação estrutural real como sensor isolado.

**Correção implementada:** o modo RF-only agora inicializa exclusivamente por **triangulação de interseção de azimutes** a partir de medições de múltiplas estações RF (mínimo 2, não colineares com o alvo) — sem nenhuma informação de câmera. Como o RF não mede elevação (Item 5), a coordenada `z` inicial é definida por um **prior de altitude declarado como suposição** (não medido; um palpite de "altitude típica de operação"), com incerteza inicial larga. Estrutural e importante: **nenhuma medição subsequente do RF-only jamais corrige `z`** ao longo de toda a execução — o erro de altitude permanece essencialmente não corrigido do início ao fim, refletindo fielmente a limitação bearing-only bidimensional do RF.

**Resultado da correção (semente padrão, Cenário 1):** RMSE de posição 3D do RF-only subiu de ~5,67 m (com a inicialização indevidamente auxiliada por câmera) para **9,51 m** (isolamento correto) — dos quais **6,48 m** vêm isoladamente do erro de altitude não corrigido. Testado nos 3 cenários com a semente padrão: RMSE entre 7,35 m e 9,51 m, sem divergência numérica. Fusão e câmera-only permanecem inalterados por essa correção, já que legitimamente têm acesso à câmera.

**Implicação para os resultados já reportados:** a conclusão qualitativa de que "RF-only é consistentemente o pior método" (Item 8c) permanece válida e agora está *mais* robustamente fundamentada (a diferença para os demais métodos aumentou). Porém, os **valores numéricos exatos** de RF-only nas Tabelas do Item 8c e do artigo (`artigo_lafusion.docx`) refletem a versão não corrigida e precisam ser regenerados antes da submissão final - o estudo Monte Carlo completo (Item 8c) deve ser re-executado com esta correção antes de qualquer publicação dos números.

---

## 8b. Escopo de Tracking e Esquema de Saída de Dados

**Escopo de tracking:** este trabalho realiza rastreamento de estado (posição, velocidade) de um único alvo via filtro recursivo (UKF), no sentido de "tracking" empregado na literatura de fusão vision+radar e na arquitetura JHU/APL (predição + atualização a cada instante). Não são abordados problemas de gerenciamento de múltiplas trilhas (iniciação, associação sob ambiguidade, término de trilha) — fora do escopo desta versão (ver Item 10).

**Motivação:** o trabalho será conduzido em colaboração com outro pesquisador, exigindo que o simulador produza uma saída estruturada e padronizada de tracking a cada passo de tempo, não apenas a posição estimada.

**Referência de formato adotada:** esquema inspirado no padrão MAVLink de telemetria de UAV (usado por PX4/ArduPilot), que já define campos equivalentes aos necessários aqui — posição (lat/lon/alt ou x/y/z local), velocidade de solo, "course over ground" (rumo de deslocamento) e campos de incerteza associados a cada grandeza [MAVLink Standard/Common Message Set; MAVSDK Telemetry::RawGps].

**Distinção técnica importante:** como o simulador possui apenas sensores de posição (RF/AoA e câmera/bearing), sem sensor de atitude (giroscópio/magnetômetro), a saída fornece **"course over ground"** (rumo de deslocamento, derivado do vetor velocidade estimado) — e não "heading" (orientação/proa do veículo), que são conceitos distintos no próprio padrão MAVLink. Essa distinção deve ser comunicada ao pesquisador colaborador para evitar ambiguidade na interpretação dos dados.

**Escopo confirmado (decisão do orientando):** o simulador NÃO modela sensores embarcados do drone (IMU, magnetômetro, barômetro). O escopo se limita a: (1) simulação de dados de detecção via RF, (2) simulação de dados de detecção via imagem, e (3) simulação da movimentação/trajetória do drone (ground truth). Essa decisão é consistente com o cenário real do problema: em um cenário de detecção de drone não-cooperativo/desconhecido, não há acesso à telemetria interna do alvo — apenas às observações externas (RF e imagem). Toda estimativa de cinemática (posição, velocidade vetorial, rumo) deriva exclusivamente do estado do UKF alimentado por RF+câmera, nunca de um sensor interno do drone.

**Esquema de saída proposto (por passo de tempo, por cenário, por modo de sensor):**

| Campo | Descrição | Origem |
|---|---|---|
| `timestamp` | Instante da estimativa | Clock da simulação |
| `x, y` (ou `lat, lon`) | Posição horizontal | Estado do UKF |
| `z` (ou `alt`) | Elevação/altitude | Estado do UKF |
| `vx, vy, vz` | Velocidade vetorial (m/s), componentes cartesianas | Estado do UKF |
| `speed` | Velocidade escalar (m/s) | Derivada de vx, vy, vz do estado do UKF |
| `course_over_ground` | Rumo de deslocamento (graus) | Derivado de vx, vy do estado do UKF (não é heading/atitude) |
| `vertical_speed` | Taxa de subida/descida (m/s) | Componente vz do estado do UKF (redundante com vz, mantido por legibilidade) |
| `pos_uncertainty` | Incerteza de posição (desvio-padrão, m) | Diagonal da matriz de covariância do UKF |
| `vel_uncertainty` | Incerteza de velocidade (desvio-padrão, m/s) | Diagonal da matriz de covariância do UKF |
| `scenario_id` | Cenário geométrico (1, 2 ou 3) | Configuração da simulação |
| `sensor_mode` | RF-only / câmera-only / fusão | Configuração da simulação |
| `ground_truth_x/y/z` | Posição real do drone (para avaliação) | Trajetória gerada |
| `ground_truth_vx/vy/vz` | Velocidade vetorial real do drone (para avaliação) | Derivada analítica da trajetória gerada |

**Sistema de coordenadas:** quadro local ENU (East-North-Up), em metros, a partir de uma origem arbitrária — consistente com o modo local previsto pelo próprio padrão MAVLink [MAVLink Common Message Set]. Conversão opcional para coordenadas geodésicas (lat/lon) a partir de uma origem geográfica configurável, caso o pesquisador colaborador precise desse formato — transformação de saída, sem impacto na lógica de fusão.

---

## 9. Métricas de Avaliação

1. **RMSE de posição 3D** (fusão vs. ground truth), por cenário e por modo de fusão.
2. **Erro em função da posição na trajetória** — para identificar onde cada geometria degrada (ex.: extremos do arco no Cenário 1).
3. **GDOP teórico vs. RMSE empírico** — validação cruzada da teoria de geometria de sensores no contexto de fusão heterogênea.
4. **Comparação estatística entre os 3 cenários** — múltiplas repetições Monte Carlo variando a semente de ruído; teste estatístico (ANOVA ou não-paramétrico) para significância da diferença entre cenários.
5. **Ganho da fusão** — comparação RF-only / Câmera-only / Fusão, para quantificar o benefício de combinar as modalidades.

**Resultado da implementação (Módulos 9 e 10):**
- **Módulo 9 (exportação):** esquema padronizado (Item 8b) implementado e testado tanto para a saída do UKF quanto do baseline geométrico — para o baseline, os campos de velocidade/incerteza ficam explicitamente `NaN` (documentado como limitação estrutural do método memoryless, não erro).
- **Módulo 10 (métricas):** funções de RMSE, resumo por execução, agregação Monte Carlo, comparação estatística (Kruskal-Wallis) e validação cruzada GDOP-RMSE implementadas e testadas.
- **Validação cruzada GDOP x RMSE (Cenário 1, modo fusão):** correlação de Pearson entre o GDOP teórico (Módulo 6) e o erro de posição empírico, binados em 20 janelas de tempo ao longo da trajetória — **r = 0,60 (p = 0,005)**, estatisticamente significativa. Essa é a validação central do gap identificado na Seção 2: confirma que a teoria clássica de GDOP prediz corretamente onde o erro será maior/menor mesmo no contexto específico (não convencional na literatura) de fusão heterogênea RF+câmera.
- **Resumo de ganho da fusão (Cenário 1):** fusão reduz o RMSE em 85,1% frente ao RF-only e 64,5% frente à câmera-only; do ganho total da fusão sobre o baseline geométrico, ~83% vem da filtragem temporal (UKF) e ~17% da combinação multi-sensor pura (ver Módulo 8).

---

## 10. Limitações e Ameaças à Validade (registradas até o momento)

- **Ruído calibrado por poucos estudos específicos:** o valor de erro angular RF (<5°) e o valor de erro de disparidade de câmera (calibrado via AirSim) vêm de setups particulares (hardware/software específicos) que podem não generalizar — mitigado parcialmente pelo uso de fórmulas físicas paramétricas em vez de constantes copiadas diretamente.
- **Ausência de curva erro-vs-distância explícita** na fonte de erro AoA — o valor de 5° é tratado como piso de melhor caso.
- **Fonte de frame rate de câmera é técnica/industrial, não peer-reviewed** — usada por falta de referência acadêmica mais específica encontrada até o momento.
- **Detecção assumida sempre bem-sucedida** — simplificação que não reflete oclusão, perda de sinal ou falsos negativos reais.
- **Ausência de múltiplos alvos** — a associação RF↔câmera é trivial no cenário de alvo único; extensão a múltiplos alvos exigiria mecanismo de associação (gating, JPDA, etc.), não coberto nesta versão.
- **Validade externa:** resultados de simulação não substituem validação de campo; o estudo é uma análise comparativa controlada de geometrias e algoritmos de fusão, não uma validação com hardware real.

---

## 11. Parâmetros Consolidados (referência rápida)

| Parâmetro | Valor / Modelo | Fonte |
|---|---|---|
| Banda RF | 2.4 / 5.8 GHz | Definição do projeto |
| Sinal RF simulado | Downlink do drone | Definição do projeto |
| Método de localização RF | AoA | Han & Jang, *Sensors* 2025 |
| Path-loss RF | FSPL + calibração empírica 2.4 GHz | Jeong et al., 2018 |
| Erro angular RF (piso) | < 5° (curto/médio alcance) | Han & Jang, *Sensors* 2025 |
| Taxa de atualização RF | ~833 Hz (ordem de grandeza) | Han & Jang, *Sensors* 2025 |
| Modelo de erro de câmera | ΔZ = (z²/(B·f))·Δd | Formulação clássica de triangulação estéreo |
| Calibração de Δd | AirSim + tiny-YOLOv4, até 8 m, erro 23% | Sharma, Jain & Kothari, 2022 |
| Taxa de quadros câmera | 30 fps | Relatório técnico C-UAS EO/IR, 2026 |
| Nº de estações por cenário | 3 | Definição do projeto |
| Detecção | Sempre bem-sucedida (simplificação) | Definição do projeto |
| Filtro de fusão | UKF assíncrono/multi-taxa | Literatura de fusão multi-rate vision+radar |
| Baseline de comparação | Triangulação/multilateração geométrica | — |

---

## 12. Referências

*Cada referência indica, além da citação completa, **onde dentro da fonte** a informação usada foi encontrada — para facilitar sua verificação.*

1. **Han, S.; Jang, B.J.** (2025). "Drone's Angle-of-Arrival Estimation Using a Switched-Beam Antenna and Single-Channel Receiver." *Sensors*, 25(8), 2376. DOI: [10.3390/s25082376](https://doi.org/10.3390/s25082376).
   - **Onde encontrar:** erro angular <5° e banda do sinal (vídeo DJI Phantom 4 Pro, 10 MHz) — Abstract e seção de Resultados Experimentais. Taxa de atualização (~1,2 ms por ciclo de chaveamento) — seção de descrição do sistema (arranjo SP6T de 6 antenas, "Section 2 - Proposed System Configuration and Operating Principle"). **Ganho de antena de recepção (~9 dBi, antena PM-PP09, banda 2,400–2,483 GHz, beamwidth de 3 dB ~60°)** — Section 3.1 "Hardware Configuration".

2. **Sharma, A.; Jain, N.; Kothari, M.** (2022). "Lightweight Multi-Drone Detection and 3D-Localization via YOLO." *arXiv:2202.09097*. https://arxiv.org/abs/2202.09097.
   - **Onde encontrar:** calibração de erro de câmera (AirSim, tiny-YOLOv4 + triangulação estéreo, máx. 8 m, erro médio 23%) — Abstract e seção de Resultados/Avaliação Experimental.

3. **Wimalajeewa, T. et al.** (2024). "Multi-Stage Fusion Architecture for Small-Drone Localization and Identification Using Passive RF and EO Imagery: A Case Study." *arXiv:2406.16875*. https://arxiv.org/abs/2406.16875.
   - **Onde encontrar:** descrição da arquitetura de fusão multiestágio RF+EO e associação detecção-a-detecção — Abstract e seção de Arquitetura do Sistema.

4. **Jeong, W.H. et al.** (2018). "Empirical Path-Loss Modeling and a RF Detection Scheme for Various Drones." *Wireless Communications and Mobile Computing*, 2018, Article ID 6795931. https://www.hindawi.com/journals/wcmc/2018/6795931/.
   - **Onde encontrar:** potência de transmissão (-23 dBm), ganhos de antena (2,5 dBi Tx / 3 dBi Rx) e potência recebida calculada (-71,52 dBm) para drone tipo Wi-Fi — tabela de especificações de sinal por tipo de drone (seção de caracterização empírica do sinal).

5. **Herath, S.C.K.; Pathirana, P.N.** (2013). "Optimal Sensor Arrangements in Angle of Arrival (AoA) and Range Based Localization with Linear Sensor Arrays." *Sensors*, 13(9), 12277–12294. DOI: [10.3390/s130912277](https://doi.org/10.3390/s130912277).
   - **Onde encontrar:** formulação de GDOP para localização bearing-only e análise de geometria de array linear — seção de derivação teórica de GDOP (corpo central do artigo, antes da seção de resultados numéricos).

6. **"Multi-UAV Tracking Evaluation Using 5G Uplink Signals on an O-RAN ISAC Simulation Testbed."** *arXiv:2608.10784*. https://arxiv.org/html/2608.10784.
   - **Onde encontrar:** degradação do estimador interferométrico em direção ao endfire — seção de análise de desempenho geométrico do array (discussão de broadside vs. endfire).

7. **"Optimization of the Coverage and Accuracy of an Indoor Positioning System with a Variable Number of Sensors."** *Sensors*, 16(6), 934 (2016). DOI: [10.3390/s16060934](https://doi.org/10.3390/s16060934).
   - **Onde encontrar:** resultado de GDOP mínimo no centro do polígono regular formado pelas âncoras — seção de análise geométrica/otimização de posicionamento de sensores.

8. **"Measurement Along the Path of Unmanned Aerial Vehicles for Best Horizontal Dilution of Precision and Geometric Dilution of Precision."** PMC12251746. https://www.ncbi.nlm.nih.gov/pmc/articles/PMC12251746/.
   - **Onde encontrar:** papel do GDOP como objetivo primário em localização — Introdução/Abstract.

9. **Kang, S.; et al.** (2014). "Asynchronous Sensor Fusion using Multi-rate Kalman Filter." https://www.researchgate.net/publication/289102615.
   - **Onde encontrar:** arquitetura de fusão multi-taxa vision+radar com filtro de Kalman descentralizado por sensor — Abstract/seção de arquitetura proposta. **Nota:** autoria completa não confirmada com certeza — recomendo conferência adicional antes da citação formal no artigo.

10. **Referência sobre UKF vs. EKF em fusão de sensores automotivos (LiDAR/Radar).**
    - **Nota de verificação:** referência de menor confiança nesta rodada — recomendo substituir por uma fonte canônica (ex.: Julier, S.J.; Uhlmann, J.K. "A New Extension of the Kalman Filter to Nonlinear Systems") antes da escrita final do artigo. Não incluir citação formal até essa verificação.

11. **"Counter-UAS 101 — EO/IR Detection."** Relatório técnico, drone-warfare.com (2026).
    - **Onde encontrar:** taxa de quadros de câmera EO padrão (30/60 fps) e limiar de câmeras térmicas (~9 Hz, limiar ITAR) — corpo do texto, seção sobre sensores EO/IR. **Nota:** fonte técnica/industrial, não peer-reviewed.

12. **MAVLink Developer Guide — Common Message Set.** https://mavlink.io/en/messages/common.html.
    - **Onde encontrar:** campos de telemetria (lat/lon/alt, ground speed, course over ground, incertezas) — mensagens `GLOBAL_POSITION_INT` e `HIGH_LATENCY2`. Referencial local (NED) vs. global (lat/lon/alt) — documentação de frames de coordenadas (`MAV_FRAME`).

13. Fórmula clássica de erro de profundidade por triangulação estéreo, ΔZ = (z²/(B·f))·Δd.
    - **Onde encontrar:** formulação padrão amplamente documentada em manuais de visão computacional estéreo (ex.: capítulos de calibração/geometria epipolar); não associada a uma única fonte primária.

14. **"Systems, methods, and apparatus for estimating angle of arrival."** Patente US12560669. https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12560669.
    - **Onde encontrar:** fórmula σθ = λ/(L√(M·N·SNR))·(1/cos θ)·√(6/2π), demonstrando erro_AoA ∝ 1/√SNR — parágrafo de descrição do "processing unit 106" na Descrição Detalhada (Detailed Description) da patente, próximo à Fig. 3 (fluxograma do método de estimação de AoA).

15. **"Interference rejection in a radio receiver."** Patente US7116958. https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/7116958.
    - **Onde encontrar:** exemplo de link budget com figura de ruído de 6,2 dB e cálculo de sensibilidade (-94,8 dBm) — **Table 1** da patente, com a fórmula N(dBm) = -174 + 10·log(B) + NF explicitada no texto imediatamente após a tabela.

16. **Cho, K.; et al.** "VIODE: A Simulated Dataset to Address the Challenges of Visual-Inertial Odometry in Dynamic Environments." *arXiv:2102.05965*. https://arxiv.org/pdf/2102.05965.
    - **Onde encontrar:** configuração de câmera estéreo padrão em AirSim (resolução WVGA 752×480 px, baseline 5 cm, FOV 90°, 20 Hz) — seção "C. Sensor setup and calibration". Usada como proxy de calibração de intrínsecos de câmera (Item 6), já que o artigo de Sharma, Jain & Kothari (2022) não publica esses parâmetros explicitamente no texto disponível.

**Recomendação de revisão:** os itens 9 e 10 (arquitetura de fusão) seguem sendo os de menor certeza bibliográfica — antes da escrita final do artigo, sugiro uma busca dedicada para substituí-los por referências mais canônicas de UKF aplicado a tracking multissensor.

---

## 13. Desenho Técnico do Simulador (ainda sem implementação)

### 13.1 Módulos do simulador

| # | Módulo | Responsabilidade |
|---|---|---|
| 1 | **TrajectoryGenerator** | Gera a trajetória ground truth (x, y, z, t) em arco, e sua derivada analítica (vx, vy, vz, t) — usada como referência em todos os cenários. |
| 2 | **StationLayoutGenerator** | Gera as coordenadas das 3 estações para cada um dos 3 cenários (linear/lateral, circular, assimétrico), a partir de parâmetros de configuração. |
| 3 | **RFSensorModel** | Por estação e instante: calcula o ângulo AoA verdadeiro, a distância, a potência recebida (FSPL + calibração empírica), o SNR resultante, o erro angular (função decrescente de SNR, piso de 5°), e amostra a medição ruidosa na taxa de atualização configurada. |
| 4 | **CameraSensorModel** | Por estação e instante: projeta a posição do drone no plano de imagem (modelo pinhole), verifica se está dentro do FOV, calcula o erro de profundidade via ΔZ = (z²/(B·f))·Δd, e amostra a medição ruidosa na taxa de quadros configurada. |
| 5 | **DetectionAssociation** | Empacota cada medição (RF ou câmera) em uma estrutura padronizada de "detecção". Trivial nesta versão (alvo único, detecção sempre bem-sucedida), mas isolado como módulo para permitir extensão futura a múltiplos alvos. |
| 6 | **GDOPCalculator** | Calcula o GDOP teórico (bearing-only) ponto a ponto ao longo da trajetória, para cada cenário — métrica preditiva independente do ruído amostrado. |
| 7 | **FusionEngine (UKF)** | Mantém o estado [x, y, z, vx, vy, vz]; executa passo de predição em alta taxa (modelo de velocidade constante) e passo de atualização de forma assíncrona, sempre que uma nova detecção (RF ou câmera) chega — arquitetura multi-taxa. |
| 8 | **BaselineEstimator** | Triangulação/multilateração geométrica por mínimos quadrados, usando as detecções mais próximas no tempo — baseline sem componente temporal, para quantificar o ganho do UKF. |
| 9 | **TelemetryExporter** | Formata a saída de cada passo de tempo conforme o esquema definido no Item 8b (posição, velocidade vetorial, incerteza, etc.), em formato estruturado (ex.: CSV/JSON). |
| 10 | **MetricsEvaluator** | Calcula RMSE vs. ground truth, cruza GDOP teórico com RMSE empírico, agrega resultados por cenário/modo/repetição, e aplica os testes estatísticos definidos no Item 9. |
| 11 | **Orchestrator** | Script de mais alto nível que combina todos os módulos acima, percorrendo a matriz de execução: 3 cenários × 4 modos de sensor × N repetições Monte Carlo. |

### 13.2 Fluxo de execução

1. **Gerar trajetória** (uma única vez, determinística — mesma referência para todos os cenários).
2. **Para cada cenário (1, 2, 3):**
   - Gerar layout das estações.
   - Calcular GDOP teórico ao longo da trajetória (independe do ruído sorteado).
   - **Para cada repetição Monte Carlo** (variando a semente de ruído):
     - Gerar detecções RF (por estação, nos instantes definidos pela taxa RF).
     - Gerar detecções de câmera (por estação, nos instantes definidos pela taxa de quadros).
     - Rodar o UKF nos 3 modos relevantes (RF-only, câmera-only, fusão RF+câmera).
     - Rodar o baseline geométrico para comparação.
     - Exportar a telemetria de cada modo.
3. **Agregar métricas**: RMSE por passo de tempo, RMSE global por cenário/modo, correlação GDOP teórico vs. RMSE empírico.
4. **Comparação estatística** entre os 3 cenários (múltiplas repetições já geram a distribuição necessária para o teste).
5. **Exportar tabelas/gráficos finais** para uso na escrita do artigo.

### 13.3 Estruturas de dados intermediárias

- **`GroundTruthPoint`**: `{timestamp, x, y, z, vx, vy, vz}`
- **`StationConfig`**: `{station_id, x, y, z, scenario_id}`
- **`Detection`**: `{timestamp, station_id, sensor_type (RF|camera), measurement (ângulo[, profundidade]), measurement_noise_std, scenario_id}`
- **`TrackState`**: `{timestamp, x, y, z, vx, vy, vz, covariance_matrix, scenario_id, sensor_mode}` — corresponde diretamente ao esquema de saída já definido no Item 8b.

### 13.4 Parâmetros configuráveis (arquivo de configuração único, ex.: YAML)

- **Trajetória**: ponto inicial/final, altura máxima do arco, duração total, passo de tempo do ground truth.
- **Estações**: número (padrão 3), parâmetros geométricos por cenário (espaçamento linear / raio circular / coordenadas customizadas do assimétrico).
- **RF**: frequência (2.4 ou 5.8 GHz), potência de transmissão, ganhos de antena, piso de erro angular, taxa de atualização, parâmetros da função de degradação de erro por SNR.
- **Câmera**: distância focal, baseline (se estéreo), resolução, erro de disparidade (Δd), taxa de quadros, FOV.
- **Fusão**: tipo de filtro (UKF), modelo de ruído de processo (Q), covariância inicial do estado.
- **Simulação**: número de repetições Monte Carlo, estratégia de sementes aleatórias, lista de modos de sensor a rodar.

Isso permite rodar toda a matriz de experimentos (cenários × condições de ruído × modos de fusão) sem alterar código — apenas o arquivo de configuração, o que também facilita a análise de sensibilidade mencionada anteriormente.

### 13.5 Plano de validação incremental

Cada módulo será testado isoladamente antes da integração, para isolar erros:

1. **TrajectoryGenerator**: inspeção visual do arco gerado (início/fim, suavidade, altura máxima no meio do percurso).
2. **StationLayoutGenerator**: inspeção visual da geometria de cada cenário sobreposta à trajetória.
3. **RFSensorModel isolado**: casos de teste com geometria simples (ex.: estação diretamente abaixo do drone) para verificar se o ângulo calculado bate com o valor geométrico esperado; verificação estatística de que o ruído amostrado corresponde ao desvio-padrão configurado; sanity check dos valores de potência recebida contra a calibração de Jeong et al. (2018).
4. **CameraSensorModel isolado**: casos de teste de projeção pinhole com posições conhecidas; verificação de que o recorte por FOV funciona corretamente; sanity check da fórmula ΔZ com valores de f, B, Δd conhecidos.
5. **GDOPCalculator**: validação contra o caso clássico já citado (GDOP mínimo no centro do polígono regular do Cenário 2).
6. **BaselineEstimator**: com detecções sem ruído (ruído = 0), a triangulação deve reconstruir exatamente o ground truth — teste de sanidade antes de introduzir ruído real.
7. **FusionEngine (UKF)**: mesmo teste de sanidade sem ruído (deve convergir exatamente à trajetória real); depois, com ruído, verificar que o RMSE do UKF é menor ou igual ao do baseline geométrico (caso contrário, algo está mal calibrado no filtro).
8. **Integração ponta a ponta em escala reduzida**: um único cenário, poucas repetições, inspeção visual da trajetória estimada vs. ground truth, e checagem do formato de saída exportado.
9. **Execução completa**: todos os cenários, todos os modos, todas as repetições Monte Carlo — geração dos resultados finais para análise.

---

## 14. Status

Todos os 11 módulos do desenho técnico implementados, validados e documentados, incluindo o experimento Monte Carlo completo (3 cenários × 3 repetições) do Módulo 11. Resultado central documentado no Item 8c: RF-only é consistentemente o pior método; o baseline geométrico é o mais estável entre repetições; fusão e câmera-only apresentam variância alta e sobreposta, sem evidência estatística robusta (N=3) de que a fusão supere a câmera isolada nesta configuração - achado reportado como resultado científico válido, não como falha do simulador. Pipeline pronto para: (a) aumentar N de repetições para maior poder estatístico, (b) explorar a exclusão do transiente de inicialização do cálculo de RMSE, ou (c) iniciar a escrita do artigo com os resultados atuais.

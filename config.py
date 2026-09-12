TRAJECTORY = {
    # Sistema de coordenadas local ENU (East-North-Up), em metros.
    # x = East (direção do movimento esquerda -> direita)
    # y = North (profundidade/afastamento em relação à linha de estações)
    # z = Up (altitude)
    "x_start": 0.0,       # posição x inicial do drone (m)
    "x_end": 100.0,       # posição x final do drone (m) - mantido próximo à
                           # ordem de grandeza do experimento de calibração
                           # do erro AoA (~30 m de referência), evitando
                           # extrapolar o modelo de SNR para distâncias não
                           # validadas
    "y_offset": 40.0,     # afastamento constante (profundidade) da linha de
                           # base das estações (m) - ver Item 4 (Cenário 1)
    "z_base": 20.0,       # altitude de base (m)
    "z_amplitude": 15.0,  # amplitude do arco de subida/descida (m) - o drone
                           # sobe de z_base até z_base + z_amplitude no meio
                           # do trajeto e volta a z_base no final
    "y_curve_amplitude": 10.0,  # amplitude da curvatura lateral (m)
    "y_curve_cycles": 2.0, # formato em "S" (ver Item 3 da metodologia)
    "duration_s": 20.0,   # duração total do trajeto (s)
    "dt_s": 0.05,         # passo de tempo do ground truth (s) - 20 Hz,
                           # suficientemente fino para servir de referência
                           # a RF (~833 Hz, reamostrado) e câmera (30 fps)
   
}

STATIONS = {
    "n_stations": 3,          # fixo em 3, conforme decisão do orientando
    "station_height_m": 2.0,  # altura do mastro/estação acima do solo (m)

    # Cenário 1 - Linear/lateral: estações alinhadas em x, na linha de base
    # (y=0), observando toda a travessia lateral do arco.
    "scenario_1": {
        "y": 0.0,
        "margin_fraction": 0.1,  # margem em relação a [x_start, x_end]
    },

    # Cenário 2 - Circular: estações em círculo centrado na área de voo.
    "scenario_2": {
        "center_x": None,   # None -> calculado automaticamente como x_mid
        "center_y": None,   # None -> calculado automaticamente como y_offset/2
        "radius_m": 35.0,
        "start_angle_deg": 90.0,  # ângulo da primeira estação (topo do círculo)
    },

    # Cenário 3 - Assimétrico: coordenadas explícitas, não regulares,
    # representando uma implantação de campo não idealizada.
    "scenario_3": {
        "positions_fraction": [
            (0.10, 0.125),
            (0.55, 0.75),
            (0.85, -0.125),
        ]
    },
}

RF = {
    "frequency_hz": 2.4e9,      # banda ISM 2.4 GHz (downlink do drone)
    "update_rate_hz": 833.0,    # ~1,2 ms/estimativa - Han & Jang, Sensors 2025
 
    # Calibração de link budget - potência de transmissão do drone: Jeong
    # et al., WCMC 2018 (drone tipo Wi-Fi). Ganho de antena de RECEPÇÃO:
    # antena PM-PP09 usada no próprio sistema de referência [Han & Jang,
    # Sensors 2025] - ganho direcional de ~9 dBi, banda 2.400-2.483 GHz,
    # beamwidth de 3 dB de ~60°. Como o sistema chaveia entre 6 antenas
    # (uma ativa por vez), o ganho relevante para o link budget é o de UMA
    # antena individual, não um ganho combinado do arranjo - valor
    # substituído do genérico de Jeong et al. (3 dBi) pelo valor real do
    # hardware da fonte de calibração do erro angular, evitando misturar
    # parâmetros de sistemas diferentes.
    "tx_power_dbm": -23.0,
    "tx_antenna_gain_dbi": 2.5,
    "rx_antenna_gain_dbi": 9.0,

    
    "signal_bandwidth_hz": 10.0e6,
    "receiver_noise_figure_db": 6.2,
 
    # Calibração do modelo de erro angular - Han & Jang, Sensors 2025:
    # erro médio < 5° foi demonstrado em ambiente de até ~30 m (piso de
    # melhor caso / boa SNR). A partir daqui, o erro é escalado com a
    # distância via a relação teórica erro ~ 1/sqrt(SNR), fundamentada em
    # formulação clássica de CRLB para estimação de AoA (ver documentação
    # do rf_sensor_model.py).
    "error_floor_deg": 5.0,
    "reference_distance_m": 30.0,
 
    "seed": 42,
}

# ---------------------------------------------------------------------------
# SENSOR DE CÂMERA - Item 6 da metodologia
# ---------------------------------------------------------------------------
CAMERA = {
    "frame_rate_hz": 30.0,   # padrão EO em C-UAS - Counter-UAS 101 (drone-warfare.com)

    # Intrínsecos de referência - configuração padrão de câmera estéreo em
    # AirSim [VIODE dataset, Cho et al.]: resolução WVGA, FOV 90°, usada
    # como proxy de calibração já que o artigo de referência do erro de
    # câmera (Sharma, Jain & Kothari, 2022) não publica os parâmetros
    # exatos de câmera no texto disponível.
    "resolution_px": (752, 480),
    "fov_horizontal_deg": 90.0,

    # Calibração do ruído de detecção (Δd, erro de disparidade/centróide em
    # pixels), retro-calculado a partir do resultado empírico de Sharma,
    # Jain & Kothari (2022): erro de profundidade de 23% a 8 m, usando a
    # fórmula ΔZ=(z²/(B·f))·Δd com f derivado do FOV/resolução acima e
    # calibration_baseline_m como baseline assumida do estudo de origem
    # (não publicada explicitamente - usamos o valor de referência do VIODE,
    # 5 cm, como proxy). Resultado: Δd ≈ 0.54 px (ver camera_sensor_model.py).
    "calibration_baseline_m": 0.05,

    # ASSUNÇÃO DE PROJETO (não vem da literatura): baseline real da estação
    # de câmera do nosso simulador. Como as estações são fixas em mastro
    # (não embarcadas em drone), podem ter baseline maior que uma câmera
    # estéreo compacta - escolhido 0.5 m como valor de projeto razoável
    # para uma estação terrestre. Deve ser revisado se uma referência mais
    # específica for encontrada.
    "station_baseline_m": 0.5,

    "seed": 7,
}

# ---------------------------------------------------------------------------
# FUSÃO - FILTER_CONFIG (Módulo 7) - Item 8 da metodologia
# ---------------------------------------------------------------------------
FILTER_CONFIG = {
    # Densidade espectral de ruído de processo (aceleração "surpresa", m²/s³).
    "process_noise_accel_std": 2.0,  # desvio-padrão de aceleração (m/s²); q = std²


    "initial_pos_std_m": 15.0,
    "initial_vel_std_mps": 5.0,

    # Janela de tempo (s) usada para inicializar o estado, com a média de
    # todas as detecções de câmera nesse intervalo (ver nota de
    # diagnóstico no ekf_fusion.py sobre velocidade de convergência).
    "init_window_s": 1.0,

    
    "rf_only_z_prior_m": 25.0,
    "rf_only_z_prior_std_m": 25.0,
}

# ---------------------------------------------------------------------------
# EXPERIMENTO (Módulo 11 - Orquestrador)
# ---------------------------------------------------------------------------
EXPERIMENT = {
    "n_monte_carlo_repetitions": 30,

    # DECISÃO REGISTRADA: uma tentativa inicial de subamostrar a taxa RF
    # (fator 20) para acelerar o experimento expôs uma fragilidade real do
    # FILTER_CONFIG - com correções esparsas, o filtro pode divergir de forma
    # praticamente caótica dependendo da sequência específica de ruído
    # que sobra após a decimação (testado: fator 5 funcionou, fatores 3 e
    # 2 pioraram - não há relação suave com o fator). Um gate de inovação
    # (rejeição por NIS) foi adicionado ao FILTER_CONFIG como robustez geral, mas
    # não resolve esse problema sozinho (uma vez divergente, o filtro
    # passa a rejeitar também as medições corretas). DECISÃO: priorizar
    # validade científica sobre velocidade - o experimento roda em
    # FIDELIDADE TOTAL (fator=1, taxa RF real ~833 Hz), configuração já
    # validada nos Módulos 3, 5, 7 e 8. Repetições reduzidas para 3 (em
    # vez de 5) para manter o tempo de execução viável (~13 min para os
    # 3 cenários).
    "rf_decimation_factor": 1,

    # Caminho relativo (funciona em Windows, Mac e Linux) - a pasta é
    # criada automaticamente dentro de onde você rodar o script. Troque
    # para um caminho absoluto se preferir salvar em outro lugar (ex.:
    # Windows: "C:/Users/seu_usuario/drone-localization/experiment_results").
    "output_dir": "./experiment_results",
}
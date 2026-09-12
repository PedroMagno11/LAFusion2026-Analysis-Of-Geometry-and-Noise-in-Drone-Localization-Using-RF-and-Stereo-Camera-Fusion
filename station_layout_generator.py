"""
Módulo 2 - StationLayoutGenerator

Gera as coordenadas das 3 estações (RF + câmera colocalizados) para cada um
dos 3 cenários geométricos.

Cenário 1 (linear/lateral): estações alinhadas ao longo de x, na linha de
    base (y=0), observando a travessia lateral completa do arco - ver
    fundamentação de array linear bearing-only (broadside vs. endfire).
Cenário 2 (circular): estações em círculo centrado na área de voo - GDOP
    mínimo no centro do polígono regular.
Cenário 3 (assimétrico): coordenadas explícitas e não regulares.
"""

import numpy as np
import pandas as pd


def _scenario_1_linear(traj_config: dict, sc1_config: dict, station_h: float) -> pd.DataFrame:
    x0, x1 = traj_config["x_start"], traj_config["x_end"]
    span = x1 - x0
    margin = sc1_config["margin_fraction"] * span
    xs = np.linspace(x0 + margin, x1 - margin, 3)
    y = sc1_config["y"]
    rows = [
        {"station_id": i + 1, "x": xs[i], "y": y, "z": station_h, "scenario_id": 1}
        for i in range(3)
    ]
    return pd.DataFrame(rows)


def _scenario_2_circular(traj_config: dict, sc2_config: dict, station_h: float) -> pd.DataFrame:
    x_mid = sc2_config["center_x"]
    if x_mid is None:
        x_mid = (traj_config["x_start"] + traj_config["x_end"]) / 2.0
    y_center = sc2_config["center_y"]
    if y_center is None:
        y_center = traj_config["y_offset"] / 2.0

    R = sc2_config["radius_m"]
    start_angle = np.radians(sc2_config["start_angle_deg"])
    angles = start_angle + np.array([0.0, 2 * np.pi / 3, 4 * np.pi / 3])

    rows = []
    for i, ang in enumerate(angles):
        x = x_mid + R * np.cos(ang)
        y = y_center + R * np.sin(ang)
        rows.append({"station_id": i + 1, "x": x, "y": y, "z": station_h, "scenario_id": 2})
    return pd.DataFrame(rows)


def _scenario_3_asymmetric(traj_config: dict, sc3_config: dict, station_h: float) -> pd.DataFrame:
    x0, x1 = traj_config["x_start"], traj_config["x_end"]
    y_off = traj_config["y_offset"]
    rows = []
    for i, (fx, fy) in enumerate(sc3_config["positions_fraction"]):
        x = x0 + fx * (x1 - x0)
        y = fy * y_off
        rows.append({"station_id": i + 1, "x": x, "y": y, "z": station_h, "scenario_id": 3})
    return pd.DataFrame(rows)


def generate_stations(scenario_id: int, traj_config: dict, stations_config: dict) -> pd.DataFrame:
    """
    Gera as coordenadas das estações para um cenário específico.

    Parâmetros
    ----------
    scenario_id : int (1, 2 ou 3)
    traj_config : dict - bloco TRAJECTORY do config.py (necessário para
        posicionar as estações em relação à trajetória)
    stations_config : dict - bloco STATIONS do config.py

    Retorna
    -------
    pd.DataFrame com colunas: station_id, x, y, z, scenario_id
    """
    h = stations_config["station_height_m"]

    if scenario_id == 1:
        return _scenario_1_linear(traj_config, stations_config["scenario_1"], h)
    elif scenario_id == 2:
        return _scenario_2_circular(traj_config, stations_config["scenario_2"], h)
    elif scenario_id == 3:
        return _scenario_3_asymmetric(traj_config, stations_config["scenario_3"], h)
    else:
        raise ValueError(f"scenario_id inválido: {scenario_id} (esperado 1, 2 ou 3)")


def generate_stations_scaled(scenario_id: int, traj_config: dict, stations_config: dict,
                              distance_scale: float = 1.0) -> pd.DataFrame:
    """
    Igual a generate_stations(), mas escala o afastamento de cada estação
    em relação ao centro geométrico da trajetória (mesmo ponto usado como
    boresight da câmera - ver camera_sensor_model.py::_station_boresight),
    preservando a FORMA relativa de cada topologia (linear/circular/
    assimétrica) - só a distância muda, não o arranjo.

    Criada para o experimento reprodutível de sensibilidade ao
    espaçamento das estações (ver spacing_experiment.py) - investigação
    da hipótese de que espaçar mais as estações melhora a cobertura de
    FOV da câmera nas bordas da trajetória e, por consequência, reduz a
    taxa de divergência da fusão (ver documento de achados da divergência
    de fusão, seção sobre cobertura de FOV).

    distance_scale=1.0 reproduz EXATAMENTE generate_stations() (mesmas
    coordenadas usadas no experimento original de N=30) - usado aqui como
    ponto de partida/baseline para a varredura, não uma nova baseline
    diferente.

    Parâmetros
    ----------
    distance_scale : float - fator multiplicativo da distância de cada
        estação ao centro da trajetória. 1.0 = baseline original: <1.0
        aproxima as estações do centro; >1.0 afasta.

    Retorna
    -------
    pd.DataFrame no mesmo esquema de generate_stations() (station_id, x,
    y, z, scenario_id).
    """
    base = generate_stations(scenario_id, traj_config, stations_config)

    x_mid = (traj_config["x_start"] + traj_config["x_end"]) / 2.0
    y_mid = traj_config["y_offset"]

    scaled = base.copy()
    scaled["x"] = x_mid + distance_scale * (base["x"] - x_mid)
    scaled["y"] = y_mid + distance_scale * (base["y"] - y_mid)
    # altura (z) não é afastamento horizontal - mantida como está
    return scaled
    from config import TRAJECTORY, STATIONS
    for sc in (1, 2, 3):
        df = generate_stations(sc, TRAJECTORY, STATIONS)
        print(f"\n--- Cenário {sc} ---")
        print(df)

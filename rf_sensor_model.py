"""
Módulo 3 - RFSensorModel

Simula as detecções RF (AoA) de cada estação ao longo da trajetória do
drone.

Cadeia de cálculo, por estação e instante de amostragem:
  1. Geometria verdadeira: vetor estação->drone -> azimute e elevação
     verdadeiros (graus).
  2. Distância 3D estação-drone.
  3. Path-loss (FSPL) na frequência configurada -> potência recebida (dBm),
     calibrada com a potência de transmissão/ganhos de antena empíricos de
     drone Wi-Fi-type em 2.4 GHz [Jeong et al., WCMC 2018].
  4. SNR (dB) = potência recebida - piso de ruído (ASSUNÇÃO DECLARADA, ver
     config.py).
  5. Erro angular (desvio-padrão, graus): escalado a partir do piso empírico
     de 5° [Han & Jang, Sensors 2025], calibrado à distância de referência
     do experimento original (~30 m), usando a relação teórica clássica
     erro_AoA ~ 1/sqrt(SNR) de estimação por CRLB (ver patente US12560669,
     "Systems, methods, and apparatus for estimating angle of arrival":
     sigma_theta = lambda / (L*sqrt(M*N*SNR)) * ... - a forma funcional
     exata depende de parâmetros de array não disponíveis aqui; usamos
     apenas a proporcionalidade 1/sqrt(SNR), sem replicar a fórmula
     completa).
  6. Amostragem da medição ruidosa: azimute verdadeiro + ruído gaussiano
     com o desvio-padrão calculado no passo 5.

"""

import numpy as np
import pandas as pd

from trajectory_generator import trajectory_state_at

C_LIGHT = 299_792_458.0  # m/s


def fspl_db(distance_m: np.ndarray, frequency_hz: float) -> np.ndarray:
    """Free-Space Path Loss, em dB. FSPL = 20log10(d) + 20log10(f) + 20log10(4*pi/c)."""
    return (
        20 * np.log10(distance_m)
        + 20 * np.log10(frequency_hz)
        + 20 * np.log10(4 * np.pi / C_LIGHT)
    )


def received_power_dbm(distance_m: np.ndarray, rf_config: dict) -> np.ndarray:
    """Potência recebida (dBm), via link budget: Ptx + Gtx + Grx - FSPL."""
    fspl = fspl_db(distance_m, rf_config["frequency_hz"])
    return (
        rf_config["tx_power_dbm"]
        + rf_config["tx_antenna_gain_dbi"]
        + rf_config["rx_antenna_gain_dbi"]
        - fspl
    )


def thermal_noise_floor_dbm(rf_config: dict) -> float:
    """
    Piso de ruído térmico do receptor (dBm), via fórmula clássica:
        N(dBm) = -174 + 10*log10(B_Hz) + NF_dB
    Parâmetros calibrados com a largura de banda real do sinal de vídeo do
    DJI Phantom 4 Pro [Han & Jang, Sensors 2025] e uma figura de ruído de
    referência de receptor SDR [patente US7116958].
    """
    B = rf_config["signal_bandwidth_hz"]
    NF = rf_config["receiver_noise_figure_db"]
    return -174.0 + 10 * np.log10(B) + NF


def angular_error_std_deg(distance_m: np.ndarray, rf_config: dict) -> np.ndarray:
    """
    Desvio-padrão do erro angular (graus), escalado a partir do piso
    empírico via a relação erro ~ 1/sqrt(SNR).
    """
    noise_floor = thermal_noise_floor_dbm(rf_config)

    pr = received_power_dbm(distance_m, rf_config)
    snr_db = pr - noise_floor
    snr_linear = 10 ** (snr_db / 10.0)

    pr_ref = received_power_dbm(np.array([rf_config["reference_distance_m"]]), rf_config)
    snr_db_ref = pr_ref - noise_floor
    snr_linear_ref = 10 ** (snr_db_ref / 10.0)

    ratio = np.sqrt(snr_linear_ref / snr_linear)
    return rf_config["error_floor_deg"] * ratio


def _true_bearing(station_xyz, drone_xyz):
    """Azimute (0-360, a partir de y=Norte, sentido horário) e elevação (graus)."""
    dx = drone_xyz[0] - station_xyz[0]
    dy = drone_xyz[1] - station_xyz[1]
    dz = drone_xyz[2] - station_xyz[2]
    horiz_dist = np.sqrt(dx**2 + dy**2)
    azimuth = np.degrees(np.arctan2(dx, dy)) % 360.0
    elevation = np.degrees(np.arctan2(dz, horiz_dist))
    dist_3d = np.sqrt(dx**2 + dy**2 + dz**2)
    return azimuth, elevation, dist_3d


def generate_rf_detections(scenario_id: int, traj_config: dict,
                            stations_df: pd.DataFrame, rf_config: dict,
                            noise_reference_stations_df: pd.DataFrame = None) -> pd.DataFrame:
    """
    Gera as detecções RF (uma realização de ruído) para um cenário.

    noise_reference_stations_df : opcional. Se fornecido, a GEOMETRIA
        (azimute verdadeiro, usado para a medição) continua vindo de
        `stations_df` (a posição REAL das estações), mas o RUÍDO
        (`angular_error_std_deg`) é calculado usando a distância às
        estações DESTE layout de referência, não das reais - permite
        mover as estações (mudando geometria/GDOP) SEM que o ruído
        mude junto (que é o que acontecia antes: `noise_scale` só
        multiplicava um ruído que ainda dependia da distância atual,
        deixando geometria e ruído emaranhados mesmo com noise_scale
        fixo - ver achados_divergencia_fusao.md §14, revisão externa
        incorporada). Os station_id devem corresponder 1:1 entre os
        dois DataFrames (ex.: mesmo layout gerado por
        generate_stations_scaled() em duas escalas diferentes).

    Retorna
    -------
    pd.DataFrame com colunas:
        timestamp, station_id, sensor_type, distance_m, received_power_dbm,
        snr_db, error_std_deg, true_azimuth_deg, true_elevation_deg,
        meas_azimuth_deg, scenario_id
        (true_elevation_deg é informativa - o RF não mede elevação, ver
        correção no docstring do módulo)
    """
    rng = np.random.default_rng(rf_config["seed"])
    noise_floor = thermal_noise_floor_dbm(rf_config)

    dt_rf = 1.0 / rf_config["update_rate_hz"]
    t = np.arange(0.0, traj_config["duration_s"] + dt_rf / 2, dt_rf)
    drone_state = trajectory_state_at(t, traj_config)
    drone_xyz = (drone_state["x"], drone_state["y"], drone_state["z"])

    if noise_reference_stations_df is not None:
        noise_ref_lookup = noise_reference_stations_df.set_index("station_id")

    rows = []
    for _, station in stations_df.iterrows():
        station_xyz = (station.x, station.y, station.z)
        az, el, dist = _true_bearing(station_xyz, drone_xyz)

        if noise_reference_stations_df is not None:
            ref = noise_ref_lookup.loc[int(station.station_id)]
            ref_xyz = (ref.x, ref.y, ref.z)
            _, _, noise_dist = _true_bearing(ref_xyz, drone_xyz)
        else:
            noise_dist = dist

        pr = received_power_dbm(dist, rf_config)
        snr = pr - noise_floor
        err_std = angular_error_std_deg(noise_dist, rf_config) * rf_config.get("noise_scale", 1.0)
        # noise_scale (padrão 1.0, não usado no experimento principal) permite
        # escalar o ruído RF INDEPENDENTEMENTE da distância - usado em
        # geometry_noise_experiment.py para separar efeito de geometria de
        # efeito de ruído (ver achados_divergencia_fusao.md §13/§14: GDOP e
        # ruído-via-distância são quase colineares nos Cenários 1/3,
        # impossível separar sem variar um dos dois artificialmente).

        az_meas = (az + rng.normal(0.0, err_std)) % 360.0
        # NOTA: elevação NÃO é medida pelo RF nesta versão (ver correção no
        # docstring do módulo) - true_elevation_deg é mantida apenas como
        # informação geométrica de referência/depuração, não como medição.

        rows.append(pd.DataFrame({
            "timestamp": t,
            "station_id": int(station.station_id),
            "sensor_type": "RF",
            "distance_m": dist,
            "received_power_dbm": pr,
            "snr_db": snr,
            "error_std_deg": err_std,
            "true_azimuth_deg": az,
            "true_elevation_deg": el,  # informativo apenas - RF não mede elevação
            "meas_azimuth_deg": az_meas,
            "scenario_id": scenario_id,
        }))

    return pd.concat(rows, ignore_index=True)


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF
    from station_layout_generator import generate_stations

    stations = generate_stations(1, TRAJECTORY, STATIONS)
    det = generate_rf_detections(1, TRAJECTORY, stations, RF)
    print(det.head(10))
    print(f"\nTotal de detecções (cenário 1, 3 estações): {len(det)}")
    print(f"Distância: {det.distance_m.min():.1f} - {det.distance_m.max():.1f} m")
    print(f"SNR: {det.snr_db.min():.1f} - {det.snr_db.max():.1f} dB")
    print(f"Erro angular (std): {det.error_std_deg.min():.2f} - {det.error_std_deg.max():.2f} graus")
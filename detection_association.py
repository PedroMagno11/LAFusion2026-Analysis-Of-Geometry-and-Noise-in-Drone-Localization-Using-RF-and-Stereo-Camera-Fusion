"""
Módulo 5 - DetectionAssociation

Padroniza as saídas heterogêneas do RFSensorModel (Módulo 3) e do
CameraSensorModel (Módulo 4) em uma estrutura comum de "Detection", 
e intercala os dois fluxos em uma única linha
do tempo ordenada - pronta para consumo pelo motor de fusão assíncrono/
multi-taxa (FILTER_CONFIG, Módulo 7).

Nesta versão (alvo único, detecção sempre bem-sucedida - Item 7 da
metodologia), a "associação" propriamente dita é trivial: não há
ambiguidade de qual detecção pertence a qual alvo, então o trabalho deste
módulo se resume a padronizar o esquema e ordenar no tempo. O módulo é
mantido isolado (em vez de fundir essa lógica em outro lugar) para permitir
extensão futura a múltiplos alvos, quando a associação deixar de ser
trivial (gating, JPDA, etc. - fora do escopo desta versão, ver Item 10).

Esquema comum de Detection (colunas de saída):
    timestamp, station_id, sensor_type, scenario_id,
    meas_azimuth_deg, azimuth_noise_std_deg,
    meas_elevation_deg, elevation_noise_std_deg,
    meas_distance_m, distance_noise_std_m,
    true_azimuth_deg, true_elevation_deg, true_distance_m
(campos true_* mantidos apenas para fins de validação/depuração do
simulador - não fazem parte da medição que um sistema real forneceria ao
filtro de fusão.)
"""

import numpy as np
import pandas as pd

_COMMON_COLUMNS = [
    "timestamp", "station_id", "sensor_type", "scenario_id",
    "meas_azimuth_deg", "azimuth_noise_std_deg",
    "meas_elevation_deg", "elevation_noise_std_deg",
    "meas_distance_m", "distance_noise_std_m",
    "true_azimuth_deg", "true_elevation_deg", "true_distance_m",
]


def standardize_rf_detections(rf_df: pd.DataFrame) -> pd.DataFrame:
    """
    Converte a saída do RFSensorModel (Módulo 3) para o esquema comum de
    Detection. RF não fornece profundidade/alcance - campos de distância
    ficam como NaN.

    CORREÇÃO: RF também não mede elevação (arranjo circular horizontal de
    6 antenas, Han & Jang 2025, mede apenas azimute) - campos de elevação
    MEDIDA ficam como NaN. A elevação verdadeira é mantida apenas como
    informação de referência/depuração (true_elevation_deg), não como
    medição.
    """
    out = pd.DataFrame({
        "timestamp": rf_df["timestamp"],
        "station_id": rf_df["station_id"],
        "sensor_type": "RF",
        "scenario_id": rf_df["scenario_id"],
        "meas_azimuth_deg": rf_df["meas_azimuth_deg"],
        "azimuth_noise_std_deg": rf_df["error_std_deg"],
        "meas_elevation_deg": np.nan,
        "elevation_noise_std_deg": np.nan,
        "meas_distance_m": np.nan,
        "distance_noise_std_m": np.nan,
        "true_azimuth_deg": rf_df["true_azimuth_deg"],
        "true_elevation_deg": rf_df["true_elevation_deg"],
        "true_distance_m": rf_df["distance_m"],
    })
    return out[_COMMON_COLUMNS]


def standardize_camera_detections(camera_df: pd.DataFrame) -> pd.DataFrame:
    """
    Converte a saída do CameraSensorModel (Módulo 4) para o esquema comum
    de Detection. Câmera fornece bearing (azimute/elevação) E profundidade
    (distância), ao contrário do RF, que só fornece bearing.
    """
    out = pd.DataFrame({
        "timestamp": camera_df["timestamp"],
        "station_id": camera_df["station_id"],
        "sensor_type": "camera",
        "scenario_id": camera_df["scenario_id"],
        "meas_azimuth_deg": camera_df["meas_azimuth_deg"],
        "azimuth_noise_std_deg": camera_df["bearing_error_std_deg"],
        "meas_elevation_deg": camera_df["meas_elevation_deg"],
        "elevation_noise_std_deg": camera_df["bearing_error_std_deg"],
        "meas_distance_m": camera_df["meas_distance_m"],
        "distance_noise_std_m": camera_df["depth_error_std_m"],
        "true_azimuth_deg": camera_df["true_azimuth_deg"],
        "true_elevation_deg": camera_df["true_elevation_deg"],
        "true_distance_m": camera_df["distance_m"],
    })
    return out[_COMMON_COLUMNS]


def associate_detections(rf_df: pd.DataFrame, camera_df: pd.DataFrame) -> pd.DataFrame:
    """
    Padroniza e intercala as detecções RF e câmera em uma única linha do
    tempo ordenada, pronta para o motor de fusão assíncrono (Módulo 7).

    Associação trivial (Item 7): alvo único, detecção sempre bem-sucedida.
    Não há etapa de gating/matching - apenas padronização + ordenação.

    Parâmetros
    ----------
    rf_df : pd.DataFrame - saída de generate_rf_detections() (Módulo 3)
    camera_df : pd.DataFrame - saída de generate_camera_detections() (Módulo 4)

    Retorna
    -------
    pd.DataFrame no esquema comum de Detection, ordenado por timestamp,
    com uma coluna adicional 'detection_id' (índice sequencial único).
    """
    rf_std = standardize_rf_detections(rf_df)
    camera_std = standardize_camera_detections(camera_df)

    merged = pd.concat([rf_std, camera_std], ignore_index=True)
    merged = merged.sort_values("timestamp", kind="mergesort").reset_index(drop=True)
    merged.insert(0, "detection_id", np.arange(len(merged)))
    return merged


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS, RF, CAMERA
    from station_layout_generator import generate_stations
    from rf_sensor_model import generate_rf_detections
    from camera_sensor_model import generate_camera_detections

    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)

    merged = associate_detections(rf_det, cam_det)

    print(f"Detecções RF: {len(rf_det)}")
    print(f"Detecções câmera: {len(cam_det)}")
    print(f"Total no fluxo unificado: {len(merged)} "
          f"(esperado: {len(rf_det) + len(cam_det)})")
    print(f"\nOrdenação temporal correta: {merged.timestamp.is_monotonic_increasing}")
    print(f"\nPrimeiras 10 linhas do fluxo unificado:")
    print(merged[["detection_id", "timestamp", "station_id", "sensor_type",
                   "meas_azimuth_deg", "meas_distance_m"]].head(10))
    print(f"\nContagem por sensor_type:")
    print(merged.sensor_type.value_counts())

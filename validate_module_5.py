"""
Validação do Módulo 5 (DetectionAssociation).

Gera:
  - detection_timeline.png: 2 painéis -
      (a) linha do tempo (janela de 1s) mostrando os instantes de detecção
          de RF e câmera intercalados, por estação;
      (b) contagem cumulativa de detecções ao longo do tempo, por tipo de
          sensor, confirmando a taxa relativa esperada (RF >> câmera).
"""

import matplotlib.pyplot as plt

from config import TRAJECTORY, STATIONS, RF, CAMERA
from station_layout_generator import generate_stations
from rf_sensor_model import generate_rf_detections
from camera_sensor_model import generate_camera_detections
from detection_association import associate_detections

OUT_DIR = "./outputs"

if __name__ == "__main__":
    scenario_id = 1
    stations = generate_stations(scenario_id, TRAJECTORY, STATIONS)
    rf_det = generate_rf_detections(scenario_id, TRAJECTORY, stations, RF)
    cam_det = generate_camera_detections(scenario_id, TRAJECTORY, stations, CAMERA)
    merged = associate_detections(rf_det, cam_det)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7))

    # (a) janela de 1s, mostrando instantes de detecção por estação/sensor
    ax = axes[0]
    window = merged[merged.timestamp <= 1.5]
    colors = {"RF": "tab:red", "camera": "tab:blue"}
    for stype in ("RF", "camera"):
        d = window[window.sensor_type == stype]
        for sid in sorted(d.station_id.unique()):
            ds = d[d.station_id == sid]
            y = sid + (0.1 if stype == "camera" else -0.1)
            ax.scatter(ds.timestamp, [y] * len(ds), s=18 if stype == "camera" else 6,
                       color=colors[stype], alpha=0.7,
                       label=stype if sid == d.station_id.min() else None)
    ax.set_yticks([1, 2, 3])
    ax.set_yticklabels(["E1", "E2", "E3"])
    ax.set_xlabel("t (s)")
    ax.set_title("Linha do tempo intercalada RF (vermelho) + câmera (azul) - janela de 1.5s")
    ax.legend()
    ax.grid(alpha=0.3)

    # (b) contagem cumulativa por tipo de sensor
    ax = axes[1]
    for stype in ("RF", "camera"):
        d = merged[merged.sensor_type == stype].sort_values("timestamp")
        ax.plot(d.timestamp, range(1, len(d) + 1), label=stype, color=colors[stype])
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Contagem cumulativa de detecções")
    ax.set_title("Acúmulo de detecções ao longo do tempo, por sensor")
    ax.legend()
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/detection_timeline.png", dpi=150)
    plt.close(fig)
    print("Gerado: detection_timeline.png")

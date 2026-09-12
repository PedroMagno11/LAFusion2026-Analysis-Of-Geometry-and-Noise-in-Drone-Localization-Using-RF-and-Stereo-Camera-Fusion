"""
Validação do Módulo 4 (CameraSensorModel).

Gera:
  - camera_module_validation.png: 4 painéis -
      (a) azimute verdadeiro vs. medido ao longo do tempo, por estação,
          incluindo lacunas onde o drone sai do FOV;
      (b) comparação do erro de profundidade da câmera vs. erro angular
          convertido em erro linear equivalente do RF, vs. distância;
      (c) cobertura de FOV por estação ao longo do tempo;
      (d) verificação estatística do ruído de bearing e de profundidade.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

from config import TRAJECTORY, STATIONS, CAMERA, RF
from station_layout_generator import generate_stations
from camera_sensor_model import (
    generate_camera_detections, depth_error_std_m, bearing_error_std_deg,
)
from rf_sensor_model import angular_error_std_deg

OUT_DIR = "./outputs"

if __name__ == "__main__":
    stations = generate_stations(1, TRAJECTORY, STATIONS)
    det = generate_camera_detections(1, TRAJECTORY, stations, CAMERA)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # (a) azimute verdadeiro vs. medido, por estação (mostrando lacunas de FOV)
    ax = axes[0, 0]
    for sid in sorted(det.station_id.unique()):
        d = det[det.station_id == sid].sort_values("timestamp")
        ax.plot(d.timestamp, d.true_azimuth_deg, label=f"E{sid} verdadeiro", lw=1.5)
        ax.scatter(d.timestamp[::15], d.meas_azimuth_deg[::15], s=8, alpha=0.5)
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Azimute (graus)")
    ax.set_title("Azimute verdadeiro vs. medido (Cenário 1) - lacunas = fora do FOV")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (b) erro de profundidade da câmera vs. erro linear equivalente do RF
    ax = axes[0, 1]
    d_range = np.linspace(10, 150, 200)
    cam_err = depth_error_std_m(d_range, CAMERA)
    rf_ang_err_rad = np.radians(angular_error_std_deg(d_range, RF))
    rf_linear_err = d_range * rf_ang_err_rad  # erro linear equivalente (arco)
    ax.plot(d_range, cam_err, color="tab:orange", label="Câmera: erro de profundidade")
    ax.plot(d_range, rf_linear_err, color="tab:red", label="RF: erro linear equiv. (d·erro_rad)")
    ax.set_xlabel("Distância (m)")
    ax.set_ylabel("Erro (m)")
    ax.set_title("Câmera (profundidade) vs. RF (linear equivalente)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (c) cobertura de FOV por estação
    ax = axes[1, 0]
    for i, sid in enumerate(sorted(det.station_id.unique())):
        d = det[det.station_id == sid]
        ax.scatter(d.timestamp, [i] * len(d), s=3, label=f"E{sid}")
    ax.set_yticks(range(len(det.station_id.unique())))
    ax.set_yticklabels([f"E{sid}" for sid in sorted(det.station_id.unique())])
    ax.set_xlabel("t (s)")
    ax.set_title("Instantes com drone dentro do FOV, por estação")
    ax.grid(alpha=0.3)

    # (d) verificação estatística (bearing e profundidade, em d=50m)
    ax = axes[1, 1]
    rng = np.random.default_rng(CAMERA["seed"] + 1)
    bearing_std = bearing_error_std_deg(CAMERA)
    samples = rng.normal(0.0, bearing_std, 20000)
    emp_std = samples.std(ddof=1)
    ax.hist(samples, bins=60, density=True, alpha=0.6, color="tab:blue",
            label="amostras de ruído de bearing")
    xs = np.linspace(-4 * bearing_std, 4 * bearing_std, 200)
    ax.plot(xs, norm.pdf(xs, 0, bearing_std), color="black", lw=2,
            label=f"N(0, {bearing_std:.3f}°) teórico")
    ax.set_title(f"Sanidade estatística (bearing): teórico={bearing_std:.4f}° | "
                 f"amostral={emp_std:.4f}°")
    ax.set_xlabel("Erro de bearing (graus)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/camera_module_validation.png", dpi=150)
    plt.close(fig)

    print(f"Verificação estatística (bearing): teórico={bearing_std:.4f}°, "
          f"amostral(n=20000)={emp_std:.4f}°, "
          f"diferença relativa={abs(bearing_std-emp_std)/bearing_std*100:.2f}%")
    print("\nGerado: camera_module_validation.png")

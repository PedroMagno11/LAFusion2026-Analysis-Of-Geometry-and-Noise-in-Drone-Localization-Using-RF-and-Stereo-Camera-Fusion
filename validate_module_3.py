"""
Validação do Módulo 3 (RFSensorModel).

Gera:
  - rf_module_validation.png: 4 painéis -
      (a) azimute verdadeiro vs. medido (ruidoso) ao longo do tempo, por
          estação, cenário 1;
      (b) erro angular teórico (desvio-padrão) vs. distância;
      (c) SNR vs. distância;
      (d) verificação estatística: para uma distância fixa, comparação
          entre o desvio-padrão configurado (teórico) e o desvio-padrão
          amostral medido em muitas repetições de ruído.
"""

import numpy as np
import matplotlib.pyplot as plt

from config import TRAJECTORY, STATIONS, RF
from station_layout_generator import generate_stations
from rf_sensor_model import (
    generate_rf_detections, angular_error_std_deg, received_power_dbm,
    thermal_noise_floor_dbm,
)

OUT_DIR = "./outputs"


def statistical_sanity_check(rf_config, distance_m=50.0, n_samples=20000):
    """Amostra n_samples de ruído em uma distância fixa e compara o desvio-
    padrão amostral com o valor teórico configurado."""
    rng = np.random.default_rng(rf_config["seed"] + 1)
    theoretical_std = angular_error_std_deg(np.array([distance_m]), rf_config)[0]
    samples = rng.normal(0.0, theoretical_std, n_samples)
    empirical_std = samples.std(ddof=1)
    return theoretical_std, empirical_std


if __name__ == "__main__":
    stations = generate_stations(1, TRAJECTORY, STATIONS)
    det = generate_rf_detections(1, TRAJECTORY, stations, RF)

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))

    # (a) azimute verdadeiro vs. medido, por estação
    ax = axes[0, 0]
    for sid in sorted(det.station_id.unique()):
        d = det[det.station_id == sid]
        ax.plot(d.timestamp, d.true_azimuth_deg, label=f"E{sid} verdadeiro", lw=1.5)
        ax.scatter(d.timestamp[::200], d.meas_azimuth_deg[::200], s=6,
                   alpha=0.5, label=f"E{sid} medido (amostrado)")
    ax.set_xlabel("t (s)")
    ax.set_ylabel("Azimute (graus)")
    ax.set_title("Azimute verdadeiro vs. medido (Cenário 1)")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)

    # (b) erro angular teórico vs. distância
    ax = axes[0, 1]
    d_range = np.linspace(10, 150, 200)
    err = angular_error_std_deg(d_range, RF)
    ax.plot(d_range, err, color="tab:red")
    ax.axvline(RF["reference_distance_m"], color="gray", ls="--",
               label=f"distância de referência ({RF['reference_distance_m']:.0f} m)")
    ax.axhline(RF["error_floor_deg"], color="gray", ls=":",
               label=f"piso empírico ({RF['error_floor_deg']:.0f}°)")
    ax.set_xlabel("Distância (m)")
    ax.set_ylabel("Desvio-padrão do erro angular (graus)")
    ax.set_title("Modelo de degradação do erro AoA com a distância")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (c) SNR vs. distância
    ax = axes[1, 0]
    pr = received_power_dbm(d_range, RF)
    noise_floor = thermal_noise_floor_dbm(RF)
    snr = pr - noise_floor
    ax.plot(d_range, snr, color="tab:purple")
    ax.axhline(0, color="gray", ls=":", lw=1)
    ax.set_xlabel("Distância (m)")
    ax.set_ylabel("SNR (dB)")
    ax.set_title(f"SNR estimado vs. distância (piso de ruído = {noise_floor:.1f} dBm)")
    ax.grid(alpha=0.3)

    # (d) verificação estatística
    ax = axes[1, 1]
    theo_std, emp_std = statistical_sanity_check(RF, distance_m=50.0)
    rng = np.random.default_rng(RF["seed"] + 1)
    samples = rng.normal(0.0, theo_std, 20000)
    ax.hist(samples, bins=60, density=True, alpha=0.6, color="tab:blue",
            label="amostras de ruído (d=50m)")
    xs = np.linspace(-4 * theo_std, 4 * theo_std, 200)
    from scipy.stats import norm
    ax.plot(xs, norm.pdf(xs, 0, theo_std), color="black", lw=2,
            label=f"N(0, {theo_std:.2f}°) teórico")
    ax.set_title(f"Sanidade estatística: std teórico={theo_std:.3f}° "
                 f"| std amostral={emp_std:.3f}°")
    ax.set_xlabel("Erro angular (graus)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(f"{OUT_DIR}/rf_module_validation.png", dpi=150)
    plt.close(fig)

    print(f"Verificação estatística em d=50m: std teórico={theo_std:.4f}°, "
          f"std amostral (n=20000)={emp_std:.4f}°, "
          f"diferença relativa={abs(theo_std-emp_std)/theo_std*100:.2f}%")
    print("\nGerado: rf_module_validation.png")
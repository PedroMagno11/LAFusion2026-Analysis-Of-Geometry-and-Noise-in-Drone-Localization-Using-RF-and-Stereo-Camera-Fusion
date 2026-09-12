"""
Módulo 6 - GDOPCalculator

Calcula um índice de diluição geométrica bearing-only (baseado em
azimute), ponto a ponto ao longo da trajetória, para cada cenário
geométrico.

Formulação (2D, horizontal, azimute-apenas):
Para cada estação i, com azimute verdadeiro az_i medido a partir do drone
na posição (x,y), a derivada do azimute em relação à posição do alvo é:

    d(az_i)/dx =  dy_i / d_i²
    d(az_i)/dy = -dx_i / d_i²

onde dx_i = x - x_i, dy_i = y - y_i, d_i² = dx_i² + dy_i² (distância
horizontal ao quadrado). Essas derivadas formam a matriz de projeto H
(uma linha por estação, 2 colunas: x, y). O índice é então:

    G = sqrt(trace((H^T H)^-1))

ATENÇÃO - NÃO É ADIMENSIONAL (correção da revisão externa - ver
achados_divergencia_fusao.md §11-14): H tem unidade 1/m (derivada de
ângulo, em radianos, por metro de posição) - diferente do GDOP clássico
de GPS/GNSS (pseudo-distância), cujo H é adimensional (cossenos
diretores). Aqui, (HᵀH)⁻¹ tem unidade m², então G = sqrt(trace(...)) tem
unidade de METRO, sob a suposição de variância angular UNITÁRIA e
homogênea entre estações (não usa o ruído angular real - ver
correlate_gdop_rmse em metrics_evaluator.py, que cruza este índice com o
erro empírico justamente para validar essa suposição). Nome mais preciso:
"índice de diluição geométrica bearing-only em 2D, sob variâncias
angulares unitárias e homogêneas" - "GDOP" é mantido como atalho de
nome (e por continuidade com a literatura que trata bearing-only sob a
mesma lógica conceitual do DOP de GPS), mas não deve ser chamado de
"adimensional" nem comparado numericamente ao GDOP clássico de GNSS.
Alternativa mais rigorosa (mas que deixa de representar geometria pura,
por incorporar o ruído real): matriz de informação ponderada J = HᵀR⁻¹H,
G_w = sqrt(trace(J⁻¹)).

Essa é a métrica GEOMÉTRICA PURA no sentido de não usar o ruído real
amostrado (mas não no sentido de ser adimensional - ver acima),
consistente com o Item 13.1: "métrica preditiva independente do ruído
amostrado" [Herath & Pathirana, Sensors 2013; Measurement Along the Path
of UAVs..., PMC12251746].

Quando H^T H é singular ou mal-condicionada (ex.: estações e alvo
colineares - "singular area"), o índice tende a infinito; nesses casos
retornamos NaN.
"""

import numpy as np
import pandas as pd

from trajectory_generator import trajectory_state_at


def _design_matrix(drone_x: float, drone_y: float, stations_df: pd.DataFrame) -> np.ndarray:
    """Matriz de projeto H (n_estações x 2), derivadas do azimute em x,y."""
    dx = drone_x - stations_df["x"].values
    dy = drone_y - stations_df["y"].values
    d2 = dx**2 + dy**2
    H = np.column_stack([dy / d2, -dx / d2])
    return H


def compute_gdop_at_point(drone_x: float, drone_y: float, stations_df: pd.DataFrame) -> float:
    """GDOP geométrico puro (bearing-only, 2D) em um único ponto."""
    H = _design_matrix(drone_x, drone_y, stations_df)
    HtH = H.T @ H
    try:
        cov = np.linalg.inv(HtH)
    except np.linalg.LinAlgError:
        return np.nan
    trace = np.trace(cov)
    if trace < 0 or not np.isfinite(trace):
        return np.nan
    return np.sqrt(trace)


def compute_gdop_along_trajectory(scenario_id: int, traj_config: dict,
                                   stations_df: pd.DataFrame) -> pd.DataFrame:
    """
    Calcula o GDOP teórico ao longo de toda a trajetória (usando o mesmo
    passo de tempo do ground truth, traj_config['dt_s']).

    Retorna
    -------
    pd.DataFrame com colunas: t, x, y, z, gdop, scenario_id
    """
    dt = traj_config["dt_s"]
    T = traj_config["duration_s"]
    t = np.arange(0.0, T + dt / 2, dt)
    state = trajectory_state_at(t, traj_config)

    gdop = np.array([
        compute_gdop_at_point(x, y, stations_df)
        for x, y in zip(state["x"], state["y"])
    ])

    return pd.DataFrame({
        "t": t, "x": state["x"], "y": state["y"], "z": state["z"],
        "gdop": gdop, "scenario_id": scenario_id,
    })


def compute_gdop_grid(stations_df: pd.DataFrame, x_range, y_range, n_points=150) -> dict:
    """
    Calcula o GDOP em uma grade 2D (para mapa de calor), útil para
    visualizar a qualidade geométrica de toda a área de cobertura, não só
    ao longo da trajetória específica.

    Retorna
    -------
    dict com 'X', 'Y' (meshgrid) e 'GDOP' (mesma forma, com NaN onde
    singular).
    """
    xs = np.linspace(*x_range, n_points)
    ys = np.linspace(*y_range, n_points)
    X, Y = np.meshgrid(xs, ys)
    GDOP = np.full(X.shape, np.nan)
    for i in range(X.shape[0]):
        for j in range(X.shape[1]):
            GDOP[i, j] = compute_gdop_at_point(X[i, j], Y[i, j], stations_df)
    return {"X": X, "Y": Y, "GDOP": GDOP}


if __name__ == "__main__":
    from config import TRAJECTORY, STATIONS
    from station_layout_generator import generate_stations

    for sc in (1, 2, 3):
        stations = generate_stations(sc, TRAJECTORY, STATIONS)
        gdop_df = compute_gdop_along_trajectory(sc, TRAJECTORY, stations)
        valid = gdop_df.gdop.replace([np.inf, -np.inf], np.nan).dropna()
        print(f"Cenário {sc}: GDOP min={valid.min():.3f} | "
              f"mediana={valid.median():.3f} | max={valid.max():.3f} | "
              f"NaN/inf: {gdop_df.gdop.isna().sum() + np.isinf(gdop_df.gdop).sum()}/{len(gdop_df)}")

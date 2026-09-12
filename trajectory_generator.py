"""
Módulo 1 - TrajectoryGenerator

Gera a trajetória ground truth do drone (posição e velocidade vetorial),
em referencial local ENU (East-North-Up, metros).

Formato do arco: movimento da esquerda para a direita (eixo x), com subida
suave até a metade do trajeto e descida (perfil senoidal em z), a uma
distância (y) média constante da linha de base das estações.

Curvatura lateral (opcional, y_curve_amplitude/y_curve_cycles): permite que
o drone se afaste e retorne à rota reta uma ou mais vezes ao longo do
trajeto (ex.: formato em "S"), em vez de manter y estritamente constante.

ASSUNÇÃO DECLARADA: progressão horizontal com velocidade constante ao longo
do tempo (s = t / duration_s). Não há fonte da literatura para essa escolha
específica de perfil cinemático - é uma simplificação de modelagem,
documentada aqui e no config.py.
"""

import numpy as np
import pandas as pd


def trajectory_state_at(t: np.ndarray, traj_config: dict) -> dict:
    """
    Calcula a posição e velocidade vetorial do drone em instante(s)
    arbitrário(s) t, usando as mesmas fórmulas analíticas de
    generate_trajectory(). Reutilizada pelos modelos de sensor (RF, câmera),
    que amostram em taxas diferentes do ground truth base.

    Parâmetros
    ----------
    t : np.ndarray - instantes de tempo (s), 0 <= t <= duration_s
    traj_config : dict - bloco TRAJECTORY do config.py

    Retorna
    -------
    dict com arrays: x, y, z, vx, vy, vz
    """
    T = traj_config["duration_s"]
    x0, x1 = traj_config["x_start"], traj_config["x_end"]
    y_off = traj_config["y_offset"]
    z_base = traj_config["z_base"]
    z_amp = traj_config["z_amplitude"]
    y_amp = traj_config.get("y_curve_amplitude", 0.0)
    y_cycles = traj_config.get("y_curve_cycles", 1.0)

    s = np.asarray(t) / T

    x = x0 + s * (x1 - x0)
    y = y_off + y_amp * np.sin(y_cycles * np.pi * s)
    z = z_base + z_amp * np.sin(np.pi * s)

    dxdt = np.full_like(s, (x1 - x0) / T)
    dydt = y_amp * (y_cycles * np.pi / T) * np.cos(y_cycles * np.pi * s)
    dzdt = z_amp * (np.pi / T) * np.cos(np.pi * s)

    return {"x": x, "y": y, "z": z, "vx": dxdt, "vy": dydt, "vz": dzdt}


def generate_trajectory(traj_config: dict) -> pd.DataFrame:
    """
    Gera o ground truth da trajetória do drone.

    Parâmetros
    ----------
    traj_config : dict
        Bloco TRAJECTORY do config.py.

    Retorna
    -------
    pd.DataFrame com colunas:
        t, x, y, z, vx, vy, vz, speed, course_over_ground, vertical_speed
    """
    T = traj_config["duration_s"]
    dt = traj_config["dt_s"]
    t = np.arange(0.0, T + dt / 2, dt)

    st = trajectory_state_at(t, traj_config)
    dxdt, dydt, dzdt = st["vx"], st["vy"], st["vz"]

    speed = np.sqrt(dxdt**2 + dydt**2 + dzdt**2)
    # course over ground: rumo do deslocamento no plano horizontal (x,y),
    # medido a partir do Norte (y), sentido horário - convenção náutica/MAVLink.
    course_over_ground = (np.degrees(np.arctan2(dxdt, dydt))) % 360.0
    vertical_speed = dzdt

    df = pd.DataFrame({
        "t": t,
        "x": st["x"], "y": st["y"], "z": st["z"],
        "vx": dxdt, "vy": dydt, "vz": dzdt,
        "speed": speed,
        "course_over_ground": course_over_ground,
        "vertical_speed": vertical_speed,
    })
    return df


if __name__ == "__main__":
    from config import TRAJECTORY
    df = generate_trajectory(TRAJECTORY)
    print(df.head())
    print(f"\nTotal de amostras: {len(df)}")
    print(f"x: {df.x.min():.1f} -> {df.x.max():.1f} m")
    print(f"z: {df.z.min():.1f} -> {df.z.max():.1f} m (pico em s=0.5)")
    print(f"speed: min={df.speed.min():.2f} max={df.speed.max():.2f} m/s")

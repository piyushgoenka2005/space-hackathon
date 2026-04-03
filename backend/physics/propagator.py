"""
Orbital Mechanics Physics Engine
RK4 integrator with J2 perturbation for LEO satellite/debris propagation.
"""

import math
import numpy as np
from typing import Tuple

# Constants
MU = 398600.4418          # Earth gravitational parameter (km^3/s^2)
RE = 6378.137             # Earth equatorial radius (km)
J2 = 1.08263e-3           # J2 zonal harmonic
G0 = 9.80665              # Standard gravity (m/s^2)


def j2_acceleration(r: np.ndarray) -> np.ndarray:
    """Compute J2 perturbation acceleration vector in ECI frame."""
    x, y, z = r
    r_mag = np.linalg.norm(r)
    r2 = r_mag * r_mag
    z2 = z * z
    factor = 1.5 * J2 * MU * RE * RE / (r_mag ** 5)
    
    ax = factor * x * (5.0 * z2 / r2 - 1.0)
    ay = factor * y * (5.0 * z2 / r2 - 1.0)
    az = factor * z * (5.0 * z2 / r2 - 3.0)
    
    return np.array([ax, ay, az])


def two_body_j2(state: np.ndarray) -> np.ndarray:
    """
    Equations of motion: two-body + J2 perturbation.
    state = [x, y, z, vx, vy, vz]
    Returns derivative [vx, vy, vz, ax, ay, az]
    """
    r = state[:3]
    v = state[3:]
    r_mag = np.linalg.norm(r)
    
    # Two-body acceleration
    a_twobody = -MU / (r_mag ** 3) * r
    
    # J2 perturbation
    a_j2 = j2_acceleration(r)
    
    a_total = a_twobody + a_j2
    
    return np.concatenate([v, a_total])


def rk4_step(state: np.ndarray, dt: float) -> np.ndarray:
    """Single RK4 integration step."""
    k1 = two_body_j2(state)
    k2 = two_body_j2(state + 0.5 * dt * k1)
    k3 = two_body_j2(state + 0.5 * dt * k2)
    k4 = two_body_j2(state + dt * k3)
    
    return state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def propagate(state: np.ndarray, duration: float, dt: float = 30.0) -> np.ndarray:
    """
    Propagate state vector forward by `duration` seconds using RK4.
    dt: integration time step in seconds (default 30s).
    Returns final state vector.
    """
    t = 0.0
    current = state.copy()
    while t < duration:
        step = min(dt, duration - t)
        current = rk4_step(current, step)
        t += step
    return current


def propagate_trajectory(state: np.ndarray, duration: float, dt: float = 30.0):
    """
    Propagate and return list of (time, state) tuples at each step.
    """
    trajectory = [(0.0, state.copy())]
    t = 0.0
    current = state.copy()
    while t < duration:
        step = min(dt, duration - t)
        current = rk4_step(current, step)
        t += step
        trajectory.append((t, current.copy()))
    return trajectory


def compute_delta_m(m_current: float, delta_v_mag: float, isp: float = 300.0) -> float:
    """
    Tsiolkovsky: compute fuel consumed for a given delta-v magnitude.
    delta_v_mag in m/s, returns mass consumed in kg.
    """
    ve = isp * G0  # exhaust velocity m/s
    dm = m_current * (1.0 - math.exp(-delta_v_mag / ve))
    return dm


def eci_to_latlon(r: np.ndarray, gmst: float) -> Tuple[float, float, float]:
    """
    Convert ECI position to geodetic lat, lon, altitude (approximate).
    gmst: Greenwich Mean Sidereal Time in radians.
    Returns (lat_deg, lon_deg, alt_km).
    """
    x, y, z = r
    r_mag = np.linalg.norm(r)
    
    # Rotate ECI -> ECEF by GMST
    x_ecef = x * math.cos(gmst) + y * math.sin(gmst)
    y_ecef = -x * math.sin(gmst) + y * math.cos(gmst)
    z_ecef = z
    
    lon = math.degrees(math.atan2(y_ecef, x_ecef))
    lat = math.degrees(math.asin(z_ecef / r_mag))
    alt = r_mag - RE
    
    return lat, lon, alt


def compute_gmst(timestamp) -> float:
    """
    Approximate GMST from datetime (radians).
    Uses simplified formula from J2000 epoch.
    """
    from datetime import datetime, timezone
    
    if isinstance(timestamp, str):
        timestamp = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
    
    # J2000 epoch: 2000-01-01 12:00:00 UTC
    j2000 = datetime(2000, 1, 1, 12, 0, 0, tzinfo=timezone.utc)
    dt_days = (timestamp - j2000).total_seconds() / 86400.0
    
    # Centuries from J2000
    T = dt_days / 36525.0
    
    # GMST in degrees (IAU simplified)
    gmst_deg = 280.46061837 + 360.98564736629 * dt_days + 0.000387933 * T * T
    gmst_deg = gmst_deg % 360.0
    
    return math.radians(gmst_deg)


def rtn_to_eci(r: np.ndarray, v: np.ndarray, dv_rtn: np.ndarray) -> np.ndarray:
    """
    Convert a delta-v vector from RTN frame to ECI frame.
    R = r_hat (radial), T = along-track, N = cross-track
    """
    r_hat = r / np.linalg.norm(r)
    n_hat = np.cross(r, v)
    n_hat = n_hat / np.linalg.norm(n_hat)
    t_hat = np.cross(n_hat, r_hat)
    
    # Rotation matrix RTN -> ECI
    rot = np.column_stack([r_hat, t_hat, n_hat])
    
    return rot @ dv_rtn

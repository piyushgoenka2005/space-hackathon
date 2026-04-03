"""
Ground station network and line-of-sight (LOS) calculations.
"""

import math
import csv
import os
import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from .propagator import RE, compute_gmst


@dataclass
class GroundStation:
    station_id: str
    name: str
    lat_deg: float
    lon_deg: float
    elevation_m: float
    min_elevation_deg: float
    
    @property
    def lat_rad(self):
        return math.radians(self.lat_deg)
    
    @property
    def lon_rad(self):
        return math.radians(self.lon_deg)
    
    def ecef_position(self) -> np.ndarray:
        """Ground station position in ECEF (km)."""
        r = RE + self.elevation_m / 1000.0
        x = r * math.cos(self.lat_rad) * math.cos(self.lon_rad)
        y = r * math.cos(self.lat_rad) * math.sin(self.lon_rad)
        z = r * math.sin(self.lat_rad)
        return np.array([x, y, z])
    
    def eci_position(self, gmst: float) -> np.ndarray:
        """Ground station position in ECI frame."""
        ecef = self.ecef_position()
        x_eci = ecef[0] * math.cos(gmst) - ecef[1] * math.sin(gmst)
        y_eci = ecef[0] * math.sin(gmst) + ecef[1] * math.cos(gmst)
        z_eci = ecef[2]
        return np.array([x_eci, y_eci, z_eci])


# Built-in fallback network (used only if CSV is unavailable)
FALLBACK_STATIONS = [
    GroundStation("GS-001", "ISTRAC_Bengaluru", 13.0333, 77.5167, 820, 5.0),
    GroundStation("GS-002", "Svalbard_Sat_Station", 78.2297, 15.4077, 400, 5.0),
    GroundStation("GS-003", "Goldstone_Tracking", 35.4266, -116.8900, 1000, 10.0),
    GroundStation("GS-004", "Punta_Arenas", -53.1500, -70.9167, 30, 5.0),
    GroundStation("GS-005", "IIT_Delhi_Ground_Node", 28.5450, 77.1926, 225, 15.0),
    GroundStation("GS-006", "McMurdo_Station", -77.8463, 166.6682, 10, 5.0),
]


def load_ground_stations_from_csv(csv_path: Optional[str] = None) -> List[GroundStation]:
    """Load station definitions from dataset CSV; fall back to built-ins if missing/invalid."""
    if csv_path is None:
        csv_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "data", "ground_stations.csv")
        )

    stations: List[GroundStation] = []
    try:
        with open(csv_path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                stations.append(
                    GroundStation(
                        station_id=row["Station_ID"].strip(),
                        name=row["Station_Name"].strip(),
                        lat_deg=float(row["Latitude"]),
                        lon_deg=float(row["Longitude"]),
                        elevation_m=float(row["Elevation_m"]),
                        min_elevation_deg=float(row["Min_Elevation_Angle_deg"]),
                    )
                )
    except Exception:
        stations = []

    return stations if stations else FALLBACK_STATIONS


DEFAULT_STATIONS = load_ground_stations_from_csv()


def elevation_angle(station_eci: np.ndarray, sat_eci: np.ndarray) -> float:
    """
    Calculate elevation angle of satellite as seen from ground station.
    Both positions in ECI frame (km).
    Returns elevation in degrees.
    """
    # Vector from station to satellite
    rho = sat_eci - station_eci
    rho_mag = np.linalg.norm(rho)
    
    if rho_mag < 1e-10:
        return 90.0
    
    # Station zenith direction (approximately radial from Earth center)
    zenith = station_eci / np.linalg.norm(station_eci)
    
    # Angle between zenith and station-to-satellite vector
    cos_zenith_angle = np.dot(rho, zenith) / rho_mag
    cos_zenith_angle = np.clip(cos_zenith_angle, -1.0, 1.0)
    zenith_angle = math.degrees(math.acos(cos_zenith_angle))
    
    # Elevation = 90 - zenith angle
    elevation = 90.0 - zenith_angle
    
    return elevation


def check_line_of_sight(
    sat_eci: np.ndarray,
    timestamp,
    stations: List[GroundStation] = None
) -> Tuple[bool, Optional[str]]:
    """
    Check if satellite has LOS to any ground station.
    Returns (has_los, station_id or None).
    """
    if stations is None:
        stations = DEFAULT_STATIONS
    
    gmst = compute_gmst(timestamp)
    
    for station in stations:
        station_pos = station.eci_position(gmst)
        elev = elevation_angle(station_pos, sat_eci)
        
        if elev >= station.min_elevation_deg:
            return True, station.station_id
    
    return False, None


def get_visible_stations(
    sat_eci: np.ndarray,
    timestamp,
    stations: List[GroundStation] = None
) -> List[Tuple[str, float]]:
    """
    Get all ground stations with LOS to satellite.
    Returns list of (station_id, elevation_angle).
    """
    if stations is None:
        stations = DEFAULT_STATIONS
    
    gmst = compute_gmst(timestamp)
    visible = []
    
    for station in stations:
        station_pos = station.eci_position(gmst)
        elev = elevation_angle(station_pos, sat_eci)
        
        if elev >= station.min_elevation_deg:
            visible.append((station.station_id, round(elev, 2)))
    
    return visible


def next_los_window(
    sat_state: np.ndarray,
    current_time,
    max_search_hours: float = 2.0,
    dt_step: float = 30.0,
    stations: List[GroundStation] = None
) -> Optional[Tuple[float, str]]:
    """
    Find next time satellite will have LOS to a ground station.
    Returns (seconds_until_los, station_id) or None.
    """
    from datetime import timedelta
    from .propagator import rk4_step
    
    if stations is None:
        stations = DEFAULT_STATIONS
    
    state = sat_state.copy()
    t = 0.0
    max_t = max_search_hours * 3600.0
    
    while t < max_t:
        state = rk4_step(state, dt_step)
        t += dt_step
        
        check_time = current_time + timedelta(seconds=t)
        has_los, sid = check_line_of_sight(state[:3], check_time, stations)
        
        if has_los:
            return t, sid
    
    return None

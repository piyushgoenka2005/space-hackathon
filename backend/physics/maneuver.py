"""
Maneuver planning module.
Handles evasion burns, recovery burns, and fuel management.
"""

import math
import numpy as np
from datetime import datetime, timedelta
from typing import Optional, Dict, List, Tuple
from .propagator import (
    compute_delta_m, propagate, rtn_to_eci, MU, G0
)

# Satellite constants
DRY_MASS = 500.0       # kg
INITIAL_FUEL = 50.0    # kg
ISP = 300.0            # seconds
MAX_DV = 15.0          # m/s per burn
COOLDOWN = 600.0       # seconds between burns
FUEL_CRITICAL = 0.05   # 5% threshold for EOL
STATION_KEEP_RADIUS = 10.0  # km
SIGNAL_DELAY = 10.0    # seconds


class SatelliteState:
    """Tracks full state of a managed satellite."""
    
    def __init__(self, sat_id: str, state: np.ndarray, nominal_state: np.ndarray):
        self.sat_id = sat_id
        self.state = state.copy()              # Current [r, v] ECI
        self.nominal_state = nominal_state.copy()  # Assigned slot [r, v]
        self.fuel_kg = INITIAL_FUEL
        self.total_mass = DRY_MASS + INITIAL_FUEL
        self.last_burn_time: Optional[datetime] = None
        self.status = "NOMINAL"  # NOMINAL, EVADING, RECOVERING, EOL
        self.maneuver_log: List[Dict] = []
        self.total_dv_used = 0.0  # m/s cumulative
        self.outage_seconds = 0.0
    
    @property
    def fuel_fraction(self):
        return self.fuel_kg / INITIAL_FUEL
    
    @property
    def is_critical_fuel(self):
        return self.fuel_fraction <= FUEL_CRITICAL
    
    def can_burn(self, current_time: datetime) -> bool:
        """Check if satellite can execute a burn (cooldown + fuel)."""
        if self.fuel_kg <= 0:
            return False
        if self.last_burn_time is not None:
            elapsed = (current_time - self.last_burn_time).total_seconds()
            if elapsed < COOLDOWN:
                return False
        return True
    
    def cooldown_remaining(self, current_time: datetime) -> float:
        """Seconds remaining in cooldown."""
        if self.last_burn_time is None:
            return 0.0
        elapsed = (current_time - self.last_burn_time).total_seconds()
        return max(0.0, COOLDOWN - elapsed)
    
    def apply_burn(self, dv_eci: np.ndarray, burn_time: datetime) -> Dict:
        """
        Apply impulsive burn. Returns burn record.
        dv_eci: delta-v in ECI frame (km/s)
        """
        dv_mag_ms = np.linalg.norm(dv_eci) * 1000.0  # Convert km/s to m/s
        
        if dv_mag_ms > MAX_DV:
            raise ValueError(f"Delta-v {dv_mag_ms:.2f} m/s exceeds max {MAX_DV} m/s")
        
        if dv_mag_ms < 1e-10:
            return {"burn_id": "NULL", "dv_magnitude_ms": 0.0}
        
        # Fuel consumption (Tsiolkovsky)
        dm = compute_delta_m(self.total_mass, dv_mag_ms, ISP)
        
        if dm > self.fuel_kg:
            raise ValueError(f"Insufficient fuel: need {dm:.3f} kg, have {self.fuel_kg:.3f} kg")
        
        # Apply
        self.state[3:] += dv_eci
        self.fuel_kg -= dm
        self.total_mass -= dm
        self.last_burn_time = burn_time
        self.total_dv_used += dv_mag_ms
        
        record = {
            "burn_time": burn_time.isoformat().replace('+00:00', '') + "Z",
            "dv_eci": dv_eci.tolist(),
            "dv_magnitude_ms": round(dv_mag_ms, 4),
            "fuel_consumed_kg": round(dm, 4),
            "fuel_remaining_kg": round(self.fuel_kg, 4),
            "mass_remaining_kg": round(self.total_mass, 4)
        }
        self.maneuver_log.append(record)
        return record
    
    def distance_to_slot(self) -> float:
        """Distance from current position to nominal slot (km)."""
        return np.linalg.norm(self.state[:3] - self.nominal_state[:3])
    
    def is_in_slot(self) -> bool:
        """Check if satellite is within station-keeping box."""
        return self.distance_to_slot() <= STATION_KEEP_RADIUS
    
    def to_dict(self) -> Dict:
        return {
            "id": self.sat_id,
            "state": self.state.tolist(),
            "fuel_kg": round(self.fuel_kg, 3),
            "total_mass_kg": round(self.total_mass, 3),
            "status": self.status,
            "fuel_fraction": round(self.fuel_fraction, 4),
            "distance_to_slot_km": round(self.distance_to_slot(), 3),
            "total_dv_used_ms": round(self.total_dv_used, 4)
        }


class ManeuverPlanner:
    """Plans evasion and recovery maneuvers."""
    
    def __init__(self):
        self.scheduled_maneuvers: List[Dict] = []
    
    def plan_evasion(
        self,
        satellite: SatelliteState,
        debris_pos: np.ndarray,
        debris_vel: np.ndarray,
        tca: datetime,
        current_time: datetime
    ) -> Optional[List[Dict]]:
        """
        Plan an evasion maneuver for an incoming conjunction.
        Uses tangential (along-track) burn for fuel efficiency.
        Returns maneuver sequence [evasion_burn, recovery_burn] or None.
        """
        if not satellite.can_burn(current_time):
            return None
        
        if satellite.is_critical_fuel:
            return None
        
        # Relative position and velocity
        rel_pos = debris_pos - satellite.state[:3]
        rel_vel = debris_vel - satellite.state[3:]
        miss_dist = np.linalg.norm(rel_pos)
        
        # Calculate required delta-v for safe standoff (200m minimum)
        # Use primarily along-track (transverse) burn for fuel efficiency
        r = satellite.state[:3]
        v = satellite.state[3:]
        
        r_hat = r / np.linalg.norm(r)
        n_hat = np.cross(r, v)
        n_hat = n_hat / np.linalg.norm(n_hat)
        t_hat = np.cross(n_hat, r_hat)
        
        # Compute approach direction in RTN
        approach_rtn = np.array([
            np.dot(rel_pos, r_hat),
            np.dot(rel_pos, t_hat),
            np.dot(rel_pos, n_hat)
        ])
        
        # Escape perpendicular to approach, primarily along-track
        # Target: move ~0.5 km away from conjunction point
        escape_distance = 0.5  # km
        
        # Time until TCA
        time_to_tca = max((tca - current_time).total_seconds(), SIGNAL_DELAY + 1)
        
        # Required delta-v magnitude (simplified: v = d/t for small maneuvers)
        dv_mag = escape_distance / time_to_tca  # km/s
        dv_mag_ms = dv_mag * 1000.0
        
        # Clamp to max
        dv_mag_ms = min(dv_mag_ms, MAX_DV * 0.5)  # Use half max for evasion, save rest for recovery
        dv_mag = dv_mag_ms / 1000.0
        
        # Direction: primarily along-track, opposite to approach
        escape_dir_rtn = np.array([0.0, 1.0, 0.0])  # Default along-track
        
        if abs(approach_rtn[1]) > 0.01:
            escape_dir_rtn[1] = -np.sign(approach_rtn[1])
        
        # Convert to ECI
        dv_rtn = escape_dir_rtn * dv_mag
        dv_eci = rtn_to_eci(r, v, dv_rtn)
        
        # Schedule burn 
        burn_time = current_time + timedelta(seconds=max(SIGNAL_DELAY + 1, 30))
        
        # Recovery burn: opposite delta-v after TCA + margin
        recovery_time = tca + timedelta(seconds=COOLDOWN + 60)
        
        sequence = [
            {
                "burn_id": f"EVASION_{satellite.sat_id}_{int(current_time.timestamp())}",
                "burnTime": burn_time.isoformat().replace('+00:00', '') + "Z",
                "deltaV_vector": {"x": dv_eci[0], "y": dv_eci[1], "z": dv_eci[2]},
                "type": "EVASION"
            },
            {
                "burn_id": f"RECOVERY_{satellite.sat_id}_{int(current_time.timestamp())}",
                "burnTime": recovery_time.isoformat().replace('+00:00', '') + "Z",
                "deltaV_vector": {"x": -dv_eci[0], "y": -dv_eci[1], "z": -dv_eci[2]},
                "type": "RECOVERY"
            }
        ]
        
        self.scheduled_maneuvers.extend(sequence)
        return sequence
    
    def plan_station_keeping(
        self,
        satellite: SatelliteState,
        current_time: datetime
    ) -> Optional[Dict]:
        """
        Plan a recovery burn to return satellite to its nominal slot.
        """
        if not satellite.can_burn(current_time):
            return None
        
        if satellite.is_in_slot():
            return None
        
        # Vector from current to nominal position
        dr = satellite.nominal_state[:3] - satellite.state[:3]
        dist = np.linalg.norm(dr)
        
        if dist < 0.01:  # Already very close
            return None
        
        # Simple proportional correction
        # Orbital period ~90 min = 5400s for LEO
        correction_time = 2700.0  # Half orbit
        dv = dr / correction_time  # km/s
        dv_mag_ms = np.linalg.norm(dv) * 1000.0
        
        if dv_mag_ms > MAX_DV:
            dv = dv / np.linalg.norm(dv) * (MAX_DV / 1000.0)
        
        burn_time = current_time + timedelta(seconds=SIGNAL_DELAY + 1)
        
        burn = {
            "burn_id": f"SK_{satellite.sat_id}_{int(current_time.timestamp())}",
            "burnTime": burn_time.isoformat().replace('+00:00', '') + "Z",
            "deltaV_vector": {"x": dv[0], "y": dv[1], "z": dv[2]},
            "type": "STATION_KEEPING"
        }
        
        self.scheduled_maneuvers.append(burn)
        return burn
    
    def plan_graveyard(
        self,
        satellite: SatelliteState,
        current_time: datetime
    ) -> Optional[Dict]:
        """
        Plan deorbit to graveyard orbit for EOL satellites.
        Raise orbit by ~300km above operational altitude.
        """
        if not satellite.can_burn(current_time):
            return None
        
        r = satellite.state[:3]
        v = satellite.state[3:]
        
        # Prograde burn to raise orbit
        v_hat = v / np.linalg.norm(v)
        dv_mag = 0.05  # km/s (~50 m/s) - enough to raise significantly
        
        fuel_needed = compute_delta_m(satellite.total_mass, dv_mag * 1000, ISP)
        if fuel_needed > satellite.fuel_kg:
            dv_mag = 0.01  # Minimum graveyard burn
        
        dv_eci = v_hat * dv_mag
        burn_time = current_time + timedelta(seconds=SIGNAL_DELAY + 1)
        
        burn = {
            "burn_id": f"GRAVEYARD_{satellite.sat_id}",
            "burnTime": burn_time.isoformat().replace('+00:00', '') + "Z",
            "deltaV_vector": {"x": dv_eci[0], "y": dv_eci[1], "z": dv_eci[2]},
            "type": "GRAVEYARD"  
        }
        
        satellite.status = "EOL"
        self.scheduled_maneuvers.append(burn)
        return burn
    
    def get_pending_maneuvers(self, sat_id: Optional[str] = None) -> List[Dict]:
        """Get scheduled but unexecuted maneuvers."""
        if sat_id:
            return [m for m in self.scheduled_maneuvers if sat_id in m["burn_id"]]
        return self.scheduled_maneuvers.copy()
    
    def clear_executed(self, burn_id: str):
        """Remove an executed maneuver from the schedule."""
        self.scheduled_maneuvers = [
            m for m in self.scheduled_maneuvers if m["burn_id"] != burn_id
        ]

"""
Conjunction Detection and Collision Probability Assessment.

Uses KD-Tree spatial indexing for O(N log N) conjunction screening
and implements Chan's method (2008) for collision probability.

References:
  - Alfano S., "A Numerical Implementation of Spherical Object Collision
    Probability", J. Astronautical Sciences, 53(1), 2005
  - Chan F.K., "Spacecraft Collision Probability", AIAA, 2008
  - Hoots F.R. et al., "An Analytic Method to Determine Future Close
    Approaches Between Satellites", Celestial Mechanics, 33, 1984
"""

import math
import numpy as np
from scipy.spatial import KDTree
from typing import List, Dict, Tuple, Optional
from datetime import datetime, timedelta
from .propagator import propagate, rk4_step

# Thresholds (km)
CRITICAL_DISTANCE = 0.1      # 100 meters
WARNING_DISTANCE = 1.0       # 1 km
CAUTION_DISTANCE = 5.0       # 5 km
SCREENING_DISTANCE = 50.0    # Initial broad screening radius (km)

# Collision probability parameters
COMBINED_HARD_BODY_RADIUS = 0.01   # 10 m in km (satellite + debris)
POSITION_UNCERTAINTY_1SIGMA = 0.05  # 50 m in km (typical TLE-derived)


def compute_collision_probability(miss_distance_km: float,
                                   relative_velocity_kms: float,
                                   sigma_r: float = POSITION_UNCERTAINTY_1SIGMA,
                                   hard_body_radius: float = COMBINED_HARD_BODY_RADIUS
                                   ) -> float:
    """
    Approximate collision probability using Chan's short-encounter model.

    For a circular cross-section of radius R with combined covariance σ,
    Pc ≈ (R² / (2·σ²)) · exp(-d² / (2·σ²))

    where d = miss distance, σ = combined 1-sigma position uncertainty,
    R = combined hard-body radius.

    Returns probability in [0, 1].
    """
    if miss_distance_km <= 0 or sigma_r <= 0:
        return 1.0 if miss_distance_km <= hard_body_radius else 0.0
    sigma2 = sigma_r ** 2
    pc = (hard_body_radius ** 2 / (2.0 * sigma2)) * math.exp(
        -miss_distance_km ** 2 / (2.0 * sigma2)
    )
    return min(pc, 1.0)


class ConjunctionEvent:
    """Represents a predicted close approach."""
    def __init__(self, sat_id: str, debris_id: str, tca: datetime,
                 miss_distance: float, sat_pos: np.ndarray, deb_pos: np.ndarray,
                 relative_velocity: np.ndarray):
        self.sat_id = sat_id
        self.debris_id = debris_id
        self.tca = tca
        self.miss_distance = miss_distance  # km
        self.sat_pos = sat_pos
        self.deb_pos = deb_pos
        self.relative_velocity = relative_velocity
        self.relative_speed = float(np.linalg.norm(relative_velocity))
        
        # Risk classification
        if miss_distance < CRITICAL_DISTANCE:
            self.risk = "CRITICAL"
        elif miss_distance < WARNING_DISTANCE:
            self.risk = "WARNING"
        elif miss_distance < CAUTION_DISTANCE:
            self.risk = "CAUTION"
        else:
            self.risk = "SAFE"

        # Collision probability (Chan 2008)
        self.pc = compute_collision_probability(miss_distance, self.relative_speed)

        # Autonomous mitigation decision telemetry (filled by engine)
        self.selected_plan = "NONE"
        self.plan_a_feasible = False
        self.plan_a_confidence = 0.0
        self.plan_a_result = "SKIPPED"
        self.fallback_triggered = False
        self.expected_miss_gain_km = 0.0

        # Approach angle in the satellite's orbital plane
        miss_vec = deb_pos - sat_pos
        self.approach_angle_deg = float(np.degrees(np.arctan2(miss_vec[1], miss_vec[0]))) % 360

    def to_dict(self):
        return {
            "satellite_id": self.sat_id,
            "debris_id": self.debris_id,
            "tca": self.tca.isoformat().replace('+00:00', '') + "Z",
            "miss_distance_km": round(self.miss_distance, 6),
            "risk": self.risk,
            "relative_velocity_kms": round(self.relative_speed, 3),
            "collision_probability": float(f"{self.pc:.6e}"),
            "approach_angle_deg": round(self.approach_angle_deg, 1),
            "selected_plan": self.selected_plan,
            "plan_a_feasible": self.plan_a_feasible,
            "plan_a_confidence": round(self.plan_a_confidence, 3),
            "plan_a_result": self.plan_a_result,
            "fallback_triggered": self.fallback_triggered,
            "expected_miss_gain_km": round(self.expected_miss_gain_km, 4),
        }


class ConjunctionAssessor:
    """
    Performs conjunction assessment using KD-Tree spatial indexing.
    Avoids O(N^2) by using spatial partitioning.
    """
    
    def __init__(self):
        self.active_cdms: List[ConjunctionEvent] = []
    
    def screen_conjunctions(
        self,
        satellites: Dict[str, np.ndarray],
        debris: Dict[str, np.ndarray],
        current_time: datetime,
        lookahead_hours: float = 24.0,
        time_step: float = 60.0  # seconds
    ) -> List[ConjunctionEvent]:
        """
        Screen for conjunctions over the lookahead window.
        Uses KD-Tree at each time step for O(N log N) performance.
        """
        events = []
        sat_ids = list(satellites.keys())
        deb_ids = list(debris.keys())
        
        if not sat_ids or not deb_ids:
            return events
        
        sat_states = {sid: satellites[sid].copy() for sid in sat_ids}
        deb_states = {did: debris[did].copy() for did in deb_ids}
        
        total_seconds = lookahead_hours * 3600.0
        t = 0.0
        
        checked_pairs = set()
        
        while t < total_seconds:
            step = min(time_step, total_seconds - t)
            
            # Propagate all objects
            for sid in sat_ids:
                sat_states[sid] = rk4_step(sat_states[sid], step)
            for did in deb_ids:
                deb_states[did] = rk4_step(deb_states[did], step)
            
            t += step
            
            # Build KD-tree from debris positions
            deb_positions = np.array([deb_states[did][:3] for did in deb_ids])
            tree = KDTree(deb_positions)
            
            # Query for each satellite
            sat_positions = np.array([sat_states[sid][:3] for sid in sat_ids])
            
            # Batch query - find all debris within screening distance
            results = tree.query_ball_point(sat_positions, SCREENING_DISTANCE)
            
            for i, nearby_indices in enumerate(results):
                sid = sat_ids[i]
                for j in nearby_indices:
                    did = deb_ids[j]
                    pair_key = (sid, did)
                    
                    if pair_key in checked_pairs:
                        continue
                    
                    dist = np.linalg.norm(sat_states[sid][:3] - deb_states[did][:3])
                    
                    if dist < CAUTION_DISTANCE:
                        checked_pairs.add(pair_key)
                        rel_vel = sat_states[sid][3:] - deb_states[did][3:]
                        
                        tca_time = current_time + timedelta(seconds=t)
                        
                        event = ConjunctionEvent(
                            sat_id=sid,
                            debris_id=did,
                            tca=tca_time,
                            miss_distance=dist,
                            sat_pos=sat_states[sid][:3].copy(),
                            deb_pos=deb_states[did][:3].copy(),
                            relative_velocity=rel_vel
                        )
                        events.append(event)
        
        # Sort by miss distance (most dangerous first)
        events.sort(key=lambda e: e.miss_distance)
        self.active_cdms = events
        return events
    
    def get_critical_events(self) -> List[ConjunctionEvent]:
        """Get only critical conjunctions requiring maneuver."""
        return [e for e in self.active_cdms if e.risk == "CRITICAL"]
    
    def get_warning_events(self) -> List[ConjunctionEvent]:
        """Get warning-level conjunctions."""
        return [e for e in self.active_cdms if e.risk in ("CRITICAL", "WARNING")]
    
    def quick_check(
        self,
        satellites: Dict[str, np.ndarray],
        debris: Dict[str, np.ndarray]
    ) -> List[Tuple[str, str, float]]:
        """
        Quick instantaneous proximity check (no propagation).
        Returns list of (sat_id, debris_id, distance) within screening range.
        """
        sat_ids = list(satellites.keys())
        deb_ids = list(debris.keys())
        
        if not sat_ids or not deb_ids:
            return []
        
        deb_positions = np.array([debris[did][:3] for did in deb_ids])
        tree = KDTree(deb_positions)
        
        results = []
        for sid in sat_ids:
            pos = satellites[sid][:3]
            indices = tree.query_ball_point(pos, SCREENING_DISTANCE)
            for idx in indices:
                did = deb_ids[idx]
                dist = np.linalg.norm(pos - debris[did][:3])
                results.append((sid, did, dist))
        
        return sorted(results, key=lambda x: x[2])

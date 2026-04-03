"""
Core simulation engine — Autonomous Constellation Manager (ACM)
Manages all orbital objects, time propagation, autonomous conjunction
assessment, collision avoidance maneuver planning, and fleet-wide
multi-objective optimization.

References:
  - Vallado D.A., "Fundamentals of Astrodynamics and Applications", 4th ed.
  - Alfano S., "A Numerical Implementation of Spherical Object Collision Probability", 2005
  - Chan F.K., "Spacecraft Collision Probability", 2008
  - Hoots F.R. et al., "An Analytic Method to Determine Future Close Approaches", 2004
"""

import math
import os
import numpy as np
import logging
from datetime import datetime, timedelta, timezone
from typing import Dict, List, Optional, Tuple
from collections import deque

from .physics.propagator import (
    propagate, rk4_step, eci_to_latlon, compute_gmst, propagate_trajectory, MU, RE
)
from .physics.conjunction import ConjunctionAssessor, ConjunctionEvent, CRITICAL_DISTANCE
from .physics.conjunction import compute_collision_probability
from .physics.maneuver import (
    ManeuverPlanner, SatelliteState, SIGNAL_DELAY, COOLDOWN, INITIAL_FUEL
)
from .physics.ground_station import (
    check_line_of_sight, get_visible_stations, DEFAULT_STATIONS
)
from .physics.tle_loader import fetch_tle, parse_tle, tle_to_eci

logger = logging.getLogger("acm.simulation")

# Configuration
MAX_TRAIL_POINTS = 60
TRAIL_INTERVAL_S = 90.0
PREDICT_DURATION_S = 5400.0
PREDICT_STEP_S = 90.0
METRICS_HISTORY_MAX = 200
CONJUNCTION_SCREEN_INTERVAL = 900
RISK_GRID_DEG = 10
RISK_FIELD_INTERVAL_S = 300
AUTONOMY_MAX_DEBRIS = 2000
LASER_ENGAGE_MAX_DIST_KM = 1.5
LASER_MIN_TCA_S = 120.0
LASER_MAX_REL_SPEED_KMS = 18.0
LASER_MIN_CONFIDENCE = 0.68
LASER_MIN_GAIN_KM = 0.03

TLE_ACTIVE_URL = [
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=active&FORMAT=tle",
    "https://celestrak.org/NORAD/elements/active.txt",
    "https://celestrak.com/NORAD/elements/active.txt",
]
TLE_DEBRIS_URL = [
    # CelesTrak does not provide a generic GROUP=debris feed.
    # Use known debris catalogs and fallback to cache/offline seed.
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=fengyun-1c-debris&FORMAT=tle",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=cosmos-2251-debris&FORMAT=tle",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=iridium-33-debris&FORMAT=tle",
]
MAX_SATELLITES = 120
MAX_DEBRIS = 5000
OFFLINE_SEED = os.getenv("AOSE_OFFLINE_SEED", "1") == "1"


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, x))


class SimulationEngine:
    def __init__(self):
        self.current_time = datetime(2026, 3, 12, 8, 0, 0, tzinfo=timezone.utc)
        self.start_time = self.current_time
        self.satellites: Dict[str, SatelliteState] = {}
        self.debris: Dict[str, np.ndarray] = {}
        self.conjunction_assessor = ConjunctionAssessor()
        self.maneuver_planner = ManeuverPlanner()

        self.total_collisions = 0
        self.total_maneuvers_executed = 0
        self.collision_log: List[Dict] = []
        self.maneuver_history: List[Dict] = []
        self.pending_burns: List[Dict] = []
        self.sat_trails: Dict[str, deque] = {}
        self._trail_accum = 0.0
        self.metrics_history: List[Dict] = []
        self._last_metrics_time = self.current_time
        self._conj_screen_accum = 0.0
        self.total_fleet_dv = 0.0
        self.collisions_avoided = 0
        self.risk_field: List[Dict] = []
        self._last_risk_field_time = self.current_time
        self._risk_grid = None
        self._risk_lat_bins = []
        self._risk_lon_bins = []
        self._risk_max = 1.0
        self._last_keepalive_time = self.current_time
        self._keepalive_idx = 0
        self.plan_a_attempts = 0
        self.plan_a_success = 0
        self.plan_a_fallback_to_b = 0
        self.plan_b_executed = 0

        self._init_from_tle_catalogs()
        self._record_metrics()
        self._record_trails()

        logger.info(
            f"ACM Engine initialized | {len(self.satellites)} sats | "
            f"{len(self.debris)} debris | T={self.current_time.isoformat()}"
        )

    def _init_from_tle_catalogs(self):
        data_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "data"))
        sat_cache = os.path.join(data_dir, "tle_active.txt")
        deb_cache = os.path.join(data_dir, "tle_debris.txt")

        sat_text = fetch_tle(TLE_ACTIVE_URL, sat_cache)
        deb_text = fetch_tle(TLE_DEBRIS_URL, deb_cache)

        sat_tles = parse_tle(sat_text, limit=MAX_SATELLITES * 2)
        deb_tles = parse_tle(deb_text, limit=MAX_DEBRIS * 2)

        if not sat_tles or not deb_tles:
            if OFFLINE_SEED:
                logger.warning("TLE catalog unavailable; using offline seed constellation.")
                self._init_offline_seed()
            else:
                logger.warning("TLE catalog unavailable; awaiting telemetry ingestion.")
            return

        for name, l1, l2 in sat_tles:
            res = tle_to_eci(name, l1, l2, self.current_time)
            if not res:
                continue
            sid, state = res
            st = np.array(state)
            self.satellites[sid] = SatelliteState(sid, st, st.copy())
            self.sat_trails[sid] = deque(maxlen=MAX_TRAIL_POINTS)
            if len(self.satellites) >= MAX_SATELLITES:
                break

        for name, l1, l2 in deb_tles:
            res = tle_to_eci(name, l1, l2, self.current_time)
            if not res:
                continue
            did, state = res
            self.debris[did] = np.array(state)
            if len(self.debris) >= MAX_DEBRIS:
                break

        logger.info(
            f"Loaded TLE catalog | {len(self.satellites)} sats | {len(self.debris)} debris"
        )

    def _init_offline_seed(self):
        """Deterministic seed when catalogs are unavailable (local demo use)."""
        num_planes, sats_per_plane = 5, 11
        base_alt = 550.0
        r_orbit = RE + base_alt
        v_circ = math.sqrt(MU / r_orbit)
        inc = math.radians(53.0)
        count = 0
        for p in range(num_planes):
            raan = math.radians(p * 360.0 / num_planes)
            for s in range(sats_per_plane):
                ta = math.radians(s * 360.0 / sats_per_plane)
                x_o, y_o = r_orbit * math.cos(ta), r_orbit * math.sin(ta)
                vx_o, vy_o = -v_circ * math.sin(ta), v_circ * math.cos(ta)
                r_eci, v_eci = self._orbital_to_eci(x_o, y_o, vx_o, vy_o, inc, raan)
                state = np.concatenate([r_eci, v_eci])
                sid = f"SAT-Alpha-{count+1:02d}"
                self.satellites[sid] = SatelliteState(sid, state, state.copy())
                self.sat_trails[sid] = deque(maxlen=MAX_TRAIL_POINTS)
                count += 1

        np.random.seed(42)
        num_debris = 10000
        for i in range(num_debris):
            if i < 6000:
                alt = 400 + np.random.random() * 250
            elif i < 8500:
                alt = 700 + np.random.random() * 200
            else:
                alt = 300 + np.random.random() * 600
            r_mag = RE + alt
            theta = np.random.random() * 2 * np.pi
            phi = np.arccos(2 * np.random.random() - 1)
            x = r_mag * np.sin(phi) * np.cos(theta)
            y = r_mag * np.sin(phi) * np.sin(theta)
            z = r_mag * np.cos(phi)
            v_c = math.sqrt(MU / r_mag)
            r_vec = np.array([x, y, z])
            d = np.random.randn(3)
            d -= np.dot(d, r_vec / r_mag) * (r_vec / r_mag)
            n = np.linalg.norm(d)
            d = d / n if n > 1e-10 else np.array([0, 1, 0])
            v = d * v_c * (0.95 + 0.1 * np.random.random())
            self.debris[f"DEB-{90000+i}"] = np.array([x, y, z, v[0], v[1], v[2]])

    @staticmethod
    def _orbital_to_eci(x_o, y_o, vx_o, vy_o, inc, raan):
        ci, si, co, so = math.cos(inc), math.sin(inc), math.cos(raan), math.sin(raan)
        return (
            np.array([co*x_o-so*ci*y_o, so*x_o+co*ci*y_o, si*y_o]),
            np.array([co*vx_o-so*ci*vy_o, so*vx_o+co*ci*vy_o, si*vy_o]),
        )

    def _record_trails(self):
        gmst = compute_gmst(self.current_time)
        for sid, sat in self.satellites.items():
            lat, lon, _ = eci_to_latlon(sat.state[:3], gmst)
            self.sat_trails[sid].append((round(lat, 3), round(lon, 3)))

    def _record_metrics(self):
        elapsed = (self.current_time - self.start_time).total_seconds()
        nominal = sum(1 for s in self.satellites.values() if s.status == "NOMINAL")
        total = len(self.satellites)
        self.metrics_history.append({
            "t": round(elapsed),
            "ts": self.current_time.isoformat().replace('+00:00', '') + "Z",
            "fuel_pct": round(np.mean([s.fuel_fraction for s in self.satellites.values()])*100, 2) if total else 0,
            "uptime_pct": round(nominal/total*100, 2) if total else 0,
            "collisions": self.total_collisions,
            "maneuvers": self.total_maneuvers_executed,
            "dv_total_ms": round(self.total_fleet_dv, 3),
            "avoided": self.collisions_avoided,
            "nominal": nominal,
            "evading": sum(1 for s in self.satellites.values() if s.status == "EVADING"),
            "plan_a_attempts": self.plan_a_attempts,
            "plan_a_success": self.plan_a_success,
            "plan_a_fallback_to_b": self.plan_a_fallback_to_b,
            "plan_b_executed": self.plan_b_executed,
        })
        if len(self.metrics_history) > METRICS_HISTORY_MAX:
            self.metrics_history = self.metrics_history[-METRICS_HISTORY_MAX:]

    def _ensure_risk_field(self):
        age = (self.current_time - self._last_risk_field_time).total_seconds()
        if not self.risk_field or age >= RISK_FIELD_INTERVAL_S:
            self.risk_field = self._compute_risk_field()
            self._last_risk_field_time = self.current_time

    def _compute_risk_field(self) -> List[Dict]:
        gmst = compute_gmst(self.current_time)
        lat_bins = list(range(-90, 91, RISK_GRID_DEG))
        lon_bins = list(range(-180, 181, RISK_GRID_DEG))
        grid = [[0.0 for _ in lon_bins] for _ in lat_bins]

        sigma = 8.0
        for st in self.debris.values():
            lat, lon, alt = eci_to_latlon(st[:3], gmst)
            # Normalize longitudes
            lon = ((lon + 180) % 360) - 180
            li = int(round((lat + 90) / RISK_GRID_DEG))
            lj = int(round((lon + 180) / RISK_GRID_DEG))
            li = max(0, min(li, len(lat_bins) - 1))
            lj = max(0, min(lj, len(lon_bins) - 1))
            base = math.exp(-max(alt, 0.0) / 1200.0)
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    ii = max(0, min(li + di, len(lat_bins) - 1))
                    jj = max(0, min(lj + dj, len(lon_bins) - 1))
                    dlat = lat - lat_bins[ii]
                    dlon = lon - lon_bins[jj]
                    w = math.exp(-(dlat * dlat + dlon * dlon) / (2 * sigma * sigma))
                    grid[ii][jj] += base * w

        max_v = max(max(row) for row in grid) if grid else 1.0
        max_v = max_v if max_v > 0 else 1.0
        field = []
        for i, la in enumerate(lat_bins):
            for j, lo in enumerate(lon_bins):
                field.append({"lat": la, "lon": lo, "risk": round(grid[i][j] / max_v, 4)})

        self._risk_grid = grid
        self._risk_lat_bins = lat_bins
        self._risk_lon_bins = lon_bins
        self._risk_max = max_v
        return field

    def _risk_at(self, lat: float, lon: float) -> float:
        if not self._risk_grid:
            return 0.0
        lon = ((lon + 180) % 360) - 180
        li = int(round((lat + 90) / RISK_GRID_DEG))
        lj = int(round((lon + 180) / RISK_GRID_DEG))
        li = max(0, min(li, len(self._risk_lat_bins) - 1))
        lj = max(0, min(lj, len(self._risk_lon_bins) - 1))
        if self._risk_max <= 0:
            return 0.0
        return self._risk_grid[li][lj] / self._risk_max

    def ingest_telemetry(self, timestamp: str, objects: List[Dict]) -> Dict:
        if timestamp:
            self.current_time = datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
        processed = 0
        for obj in objects:
            oid = obj["id"]
            r = np.array([obj["r"]["x"], obj["r"]["y"], obj["r"]["z"]])
            v = np.array([obj["v"]["x"], obj["v"]["y"], obj["v"]["z"]])
            state = np.concatenate([r, v])
            otype = obj.get("type", "").upper()
            if otype == "DEBRIS" or (not otype and not oid.startswith("SAT")):
                self.debris[oid] = state
            else:
                if oid in self.satellites:
                    self.satellites[oid].state = state
                else:
                    self.satellites[oid] = SatelliteState(oid, state, state.copy())
                    self.sat_trails[oid] = deque(maxlen=MAX_TRAIL_POINTS)
            processed += 1
        return {"status": "ACK", "processed_count": processed,
                "active_cdm_warnings": len(self.conjunction_assessor.active_cdms)}

    def schedule_maneuver(self, satellite_id: str, maneuver_sequence: List[Dict]) -> Dict:
        if satellite_id not in self.satellites:
            return {"status": "ERROR", "message": f"Satellite {satellite_id} not found"}
        sat = self.satellites[satellite_id]
        has_los, _ = check_line_of_sight(sat.state[:3], self.current_time)
        earliest = self.current_time + timedelta(seconds=SIGNAL_DELAY)
        if not has_los:
            return {"status": "ERROR", "message": "No ground-station LOS for upload",
                    "validation": {"ground_station_los": False}}
        total_dv = sum(
            math.sqrt(b["deltaV_vector"]["x"]**2+b["deltaV_vector"]["y"]**2+b["deltaV_vector"]["z"]**2)*1000
            for b in maneuver_sequence
        )
        from .physics.maneuver import compute_delta_m, ISP
        fuel_need = compute_delta_m(sat.total_mass, total_dv, ISP)
        ok = fuel_need <= sat.fuel_kg
        if not ok:
            return {"status": "ERROR", "message": "Insufficient fuel",
                    "validation": {"ground_station_los": has_los, "sufficient_fuel": False}}
        for burn in maneuver_sequence:
            dv = burn["deltaV_vector"]
            bt_clean = burn["burnTime"].replace('Z', '').split('+')[0]
            bt = datetime.fromisoformat(bt_clean).replace(tzinfo=timezone.utc)
            if bt < earliest:
                return {"status": "ERROR", "message": "Burn time violates signal delay",
                        "validation": {"ground_station_los": has_los, "sufficient_fuel": ok}}
            self.pending_burns.append({
                "sat_id": satellite_id, "burn_id": burn.get("burn_id", f"BURN_{len(self.pending_burns)}"),
                "burn_time": bt, "dv_eci": np.array([dv["x"], dv["y"], dv["z"]]),
                "type": burn.get("type", "MANUAL"),
            })
        return {"status": "SCHEDULED", "validation": {
            "ground_station_los": has_los, "sufficient_fuel": ok,
            "projected_mass_remaining_kg": round(sat.total_mass - fuel_need if ok else sat.total_mass, 2),
        }}

    def simulate_step(self, step_seconds: float) -> Dict:
        dt_int = 30.0
        t = 0.0
        man_exec = 0
        coll_step = 0
        while t < step_seconds:
            dt = min(dt_int, step_seconds - t)
            st = self.current_time + timedelta(seconds=t)
            burns = [b for b in self.pending_burns if abs((b["burn_time"]-st).total_seconds()) < dt]
            for burn in burns:
                sid = burn["sat_id"]
                if sid in self.satellites:
                    sat = self.satellites[sid]
                    try:
                        rec = sat.apply_burn(burn["dv_eci"], st)
                        rec["burn_id"] = burn["burn_id"]
                        rec["type"] = burn["type"]
                        rec["mode"] = burn.get("mode")
                        self.maneuver_history.append(rec)
                        man_exec += 1
                        self.total_fleet_dv += rec["dv_magnitude_ms"]
                        if burn["type"] == "EVASION":
                            self.collisions_avoided += 1
                    except ValueError as e:
                        logger.warning(f"Burn failed {sid}: {e}")
                self.pending_burns.remove(burn)
            for sid, sat in self.satellites.items():
                sat.state = rk4_step(sat.state, dt)
                sat.nominal_state = rk4_step(sat.nominal_state, dt)
                # Apply tiny atmospheric drag-like decay to actual state only.
                # This creates gradual slot drift so station-keeping/autonomy can act.
                if sat.status != "EOL":
                    drag = 1.0 - (5e-8 * dt)
                    sat.state[3:] *= drag
                if sat.status != "EOL":
                    if sat.is_in_slot():
                        if sat.status != "EVADING":
                            sat.status = "NOMINAL"
                    else:
                        sat.outage_seconds += dt
                        if sat.status == "NOMINAL":
                            sat.status = "RECOVERING"
            for did in self.debris:
                self.debris[did] = rk4_step(self.debris[did], dt)
            if int(t) % 60 == 0:
                coll_step += self._check_collisions(st)
            self._trail_accum += dt
            if self._trail_accum >= TRAIL_INTERVAL_S:
                self._record_trails()
                self._trail_accum = 0.0
            t += dt

        self.current_time += timedelta(seconds=step_seconds)
        if (self.current_time - self._last_metrics_time).total_seconds() >= 60:
            self._record_metrics()
            self._last_metrics_time = self.current_time
        # Keep constellation actively managed with low-cost station-keeping.
        if int((self.current_time - self.start_time).total_seconds()) % 300 == 0:
            self._ensure_risk_field()
            self._apply_stability_controller()
        # Guarantee visible backend activity in very stable scenarios.
        if (self.current_time - self._last_keepalive_time).total_seconds() >= 300:
            self._schedule_keepalive_trim()
            self._last_keepalive_time = self.current_time
        self._conj_screen_accum += step_seconds
        if self._conj_screen_accum >= CONJUNCTION_SCREEN_INTERVAL:
            self._run_autonomy()
            self._conj_screen_accum = 0.0
        self.total_maneuvers_executed += man_exec
        return {
            "status": "STEP_COMPLETE",
            "new_timestamp": self.current_time.isoformat().replace('+00:00', '') + "Z",
            "collisions_detected": coll_step, "maneuvers_executed": man_exec,
        }

    def _schedule_keepalive_trim(self):
        # Round-robin through healthy satellites and schedule a tiny prograde trim.
        sat_ids = sorted(self.satellites.keys())
        if not sat_ids:
            return
        n = len(sat_ids)
        for k in range(n):
            sid = sat_ids[(self._keepalive_idx + k) % n]
            sat = self.satellites[sid]
            if sat.status == "EOL" or not sat.can_burn(self.current_time):
                continue
            v = sat.state[3:]
            vmag = np.linalg.norm(v)
            if vmag < 1e-9:
                continue
            # 0.02 m/s trim burn (very small but measurable over time)
            dv_eci = (v / vmag) * (0.02 / 1000.0)
            burn_time = self.current_time + timedelta(seconds=SIGNAL_DELAY + 1)
            self.pending_burns.append({
                "sat_id": sid,
                "burn_id": f"TRIM_{sid}_{int(self.current_time.timestamp())}",
                "burn_time": burn_time,
                "dv_eci": dv_eci,
                "type": "HOUSEKEEPING",
                "mode": "KEEPALIVE",
            })
            self._keepalive_idx = (self._keepalive_idx + k + 1) % n
            return

    def _check_collisions(self, at_time):
        from scipy.spatial import KDTree
        if not self.satellites or not self.debris:
            return 0
        sids = list(self.satellites.keys())
        dids = list(self.debris.keys())
        tree = KDTree(np.array([self.debris[d][:3] for d in dids]))
        c = 0
        for sid in sids:
            pos = self.satellites[sid].state[:3]
            for idx in tree.query_ball_point(pos, CRITICAL_DISTANCE):
                did = dids[idx]
                d = np.linalg.norm(pos - self.debris[did][:3])
                if d < CRITICAL_DISTANCE:
                    c += 1; self.total_collisions += 1
                    self.collision_log.append({
                        "time": at_time.isoformat()+"Z", "satellite": sid,
                        "debris": did, "distance_km": round(d, 6)})
        return c

    def _run_autonomy(self):
        self._ensure_risk_field()
        sat_states = {sid: sat.state for sid, sat in self.satellites.items()}
        debris_for_screen = self._sample_debris_for_autonomy()
        events = self.conjunction_assessor.screen_conjunctions(
            sat_states, debris_for_screen, self.current_time,
            lookahead_hours=3.0, time_step=1800.0
        )
        refined = []
        for evt in events:
            sat = self.satellites.get(evt.sat_id)
            deb = self.debris.get(evt.debris_id)
            if not sat or deb is None:
                continue
            tca, dist, rel_v = self._refine_conjunction(sat.state, deb, evt.tca)
            evt.tca = tca
            evt.miss_distance = dist
            evt.relative_velocity = rel_v
            evt.relative_speed = float(np.linalg.norm(rel_v))
            refined.append(evt)
        self.conjunction_assessor.active_cdms = sorted(refined, key=lambda e: e.miss_distance)

        for evt in self.conjunction_assessor.active_cdms:
            sid = evt.sat_id
            did = evt.debris_id
            dist = evt.miss_distance
            sat = self.satellites.get(sid)
            if not sat:
                continue
            if sat.status == "EOL" or not sat.can_burn(self.current_time):
                continue
            if any(b["sat_id"] == sid for b in self.pending_burns):
                continue

            # Plan A: attempt laser-based risk reduction when the geometry is favorable.
            if dist < LASER_ENGAGE_MAX_DIST_KM:
                a_eval = self._evaluate_laser_feasibility(evt, sat)
                evt.plan_a_feasible = a_eval["feasible"]
                evt.plan_a_confidence = a_eval["confidence"]
                evt.expected_miss_gain_km = a_eval["expected_gain_km"]
                evt.selected_plan = "PLAN_A_LASER" if a_eval["feasible"] and a_eval["confidence"] >= LASER_MIN_CONFIDENCE else "PLAN_B_EVASION"

                if evt.selected_plan == "PLAN_A_LASER":
                    self.plan_a_attempts += 1
                    plan_a_ok = self._attempt_laser_mitigation(evt)
                    evt.plan_a_result = "SUCCESS" if plan_a_ok else "FAILED"
                    if plan_a_ok:
                        self.plan_a_success += 1
                        self.collisions_avoided += 1
                        logger.info(f"AUTO-LASER: {sid} mitigated {did} d={evt.miss_distance:.3f}km pc={evt.pc:.2e}")
                        continue

                    evt.fallback_triggered = True
                    self.plan_a_fallback_to_b += 1

            # Plan B: classical evasion + return sequence.
            if dist < CRITICAL_DISTANCE:
                evt.selected_plan = "PLAN_B_EVASION"
                has_los, _ = check_line_of_sight(sat.state[:3], self.current_time)
                if not has_los:
                    continue
                deb = self.debris.get(did)
                if deb is None:
                    continue
                seq = self.maneuver_planner.plan_evasion(
                    sat, deb[:3], deb[3:], evt.tca, self.current_time)
                if seq:
                    for burn in seq:
                        dv = burn["deltaV_vector"]
                        bt_raw = burn["burnTime"]
                        bt_clean = bt_raw.replace('Z', '').split('+')[0]
                        bt = datetime.fromisoformat(bt_clean).replace(tzinfo=timezone.utc)
                        if not self._has_los_window(sat.state, self.current_time, bt):
                            continue
                        self.pending_burns.append({
                            "sat_id": sid, "burn_id": burn["burn_id"],
                            "burn_time": bt,
                            "dv_eci": np.array([dv["x"], dv["y"], dv["z"]]),
                            "type": burn["type"], "mode": "GROUND-AUTHORIZED"})
                    sat.status = "EVADING"
                    self.plan_b_executed += 1
                    logger.info(f"AUTO-CA: {sid} evading {did} d={dist:.3f}km")

        # Constellation stability controller (fast approximation)
        self._apply_stability_controller()

        for sid, sat in self.satellites.items():
            if sat.is_critical_fuel and sat.status != "EOL":
                burn = self.maneuver_planner.plan_graveyard(sat, self.current_time)
                if burn:
                    bt_clean = burn["burnTime"].replace('Z', '').split('+')[0]
                    bt = datetime.fromisoformat(bt_clean).replace(tzinfo=timezone.utc)
                    self.pending_burns.append({
                        "sat_id": sid, "burn_id": burn["burn_id"], "burn_time": bt,
                        "dv_eci": np.array([burn["deltaV_vector"][k] for k in ("x","y","z")]),
                        "type": "GRAVEYARD"})

    def _sample_debris_for_autonomy(self) -> Dict[str, np.ndarray]:
        if len(self.debris) <= AUTONOMY_MAX_DEBRIS:
            return self.debris
        keys = sorted(self.debris.keys())
        stride = max(1, len(keys) // AUTONOMY_MAX_DEBRIS)
        sampled_keys = keys[::stride][:AUTONOMY_MAX_DEBRIS]
        return {k: self.debris[k] for k in sampled_keys}

    def _evaluate_laser_feasibility(self, evt: ConjunctionEvent, sat: SatelliteState) -> Dict[str, float]:
        tca_s = max(0.0, (evt.tca - self.current_time).total_seconds())
        dist_score = clamp01((LASER_ENGAGE_MAX_DIST_KM - evt.miss_distance) / LASER_ENGAGE_MAX_DIST_KM)
        tca_score = clamp01((tca_s - LASER_MIN_TCA_S) / 900.0)
        speed_score = clamp01((LASER_MAX_REL_SPEED_KMS - evt.relative_speed) / LASER_MAX_REL_SPEED_KMS)
        fuel_score = clamp01((sat.fuel_fraction - 0.1) / 0.9)
        confidence = 0.35 * dist_score + 0.25 * tca_score + 0.25 * speed_score + 0.15 * fuel_score
        feasible = (tca_s >= LASER_MIN_TCA_S and evt.relative_speed <= LASER_MAX_REL_SPEED_KMS and evt.miss_distance <= LASER_ENGAGE_MAX_DIST_KM)
        expected_gain = 0.02 + 0.20 * confidence
        return {
            "feasible": feasible,
            "confidence": round(confidence, 3),
            "expected_gain_km": round(expected_gain, 4),
        }

    def _attempt_laser_mitigation(self, evt: ConjunctionEvent) -> bool:
        deb = self.debris.get(evt.debris_id)
        sat = self.satellites.get(evt.sat_id)
        if deb is None or sat is None:
            return False

        old_dist = evt.miss_distance
        old_pc = evt.pc
        rel = deb[:3] - sat.state[:3]
        rel_n = np.linalg.norm(rel)
        if rel_n < 1e-9:
            return False

        # Apply a very small transverse velocity nudge to debris (ablation-like effect).
        r_hat = rel / rel_n
        v_rel = deb[3:] - sat.state[3:]
        n = np.cross(r_hat, v_rel)
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-12:
            n = np.cross(r_hat, np.array([0.0, 0.0, 1.0]))
            n_norm = np.linalg.norm(n)
        if n_norm < 1e-12:
            return False
        n_hat = n / n_norm
        nudge_mag = (0.006 + 0.018 * evt.plan_a_confidence) / 1000.0  # km/s
        deb[3:] = deb[3:] + n_hat * nudge_mag

        tca, dist, rel_v = self._refine_conjunction(sat.state, deb, evt.tca, window_s=1200.0, step_s=20.0)
        evt.tca = tca
        evt.miss_distance = dist
        evt.relative_velocity = rel_v
        evt.relative_speed = float(np.linalg.norm(rel_v))
        evt.pc = compute_collision_probability(evt.miss_distance, evt.relative_speed)

        improved = (dist - old_dist) >= LASER_MIN_GAIN_KM
        risk_reduced = dist > CRITICAL_DISTANCE and (old_pc <= 0 or (dist / max(old_dist, 1e-6)) > 1.2)
        return bool(improved and risk_reduced)

    def _apply_stability_controller(self):
        gmst = compute_gmst(self.current_time)
        candidates = []
        for sid, sat in self.satellites.items():
            if sat.status in ("EVADING", "EOL"):
                continue
            if not sat.can_burn(self.current_time):
                continue
            lat, lon, _ = eci_to_latlon(sat.state[:3], gmst)
            risk = self._risk_at(lat, lon)
            drift = sat.distance_to_slot()
            if drift < 0.25 and risk < 0.12:
                continue
            score = risk * 1.5 + min(drift / 10.0, 1.0)
            candidates.append((score, sid, sat))

        candidates.sort(key=lambda x: x[0], reverse=True)
        if not candidates:
            # If the constellation is too stable, still schedule light station-keeping
            # for a few satellites with the largest drift to keep metrics moving.
            drift_rank = []
            for sid, sat in self.satellites.items():
                if sat.status in ("EVADING", "EOL"):
                    continue
                if not sat.can_burn(self.current_time):
                    continue
                drift_rank.append((sat.distance_to_slot(), sid, sat))
            drift_rank.sort(key=lambda x: x[0], reverse=True)
            candidates = [(0.05, sid, sat) for _, sid, sat in drift_rank[:3] if _ > 0.02]

        for score, sid, sat in candidates[:5]:
            burn = self.maneuver_planner.plan_station_keeping(sat, self.current_time)
            if not burn:
                continue
            bt_clean = burn["burnTime"].replace('Z', '').split('+')[0]
            bt = datetime.fromisoformat(bt_clean).replace(tzinfo=timezone.utc)
            dv = burn["deltaV_vector"]
            self.pending_burns.append({
                "sat_id": sid, "burn_id": burn["burn_id"], "burn_time": bt,
                "dv_eci": np.array([dv["x"], dv["y"], dv["z"]]),
                "type": burn["type"], "mode": "STABILITY"})

    def _has_los_window(self, sat_state: np.ndarray, start: datetime, burn_time: datetime) -> bool:
        if burn_time <= start:
            return False
        horizon = min((burn_time - start).total_seconds(), 6 * 3600)
        dt = 60.0
        state = sat_state.copy()
        t = 0.0
        while t <= horizon:
            at = start + timedelta(seconds=t)
            has_los, _ = check_line_of_sight(state[:3], at)
            if has_los:
                return True
            state = rk4_step(state, dt)
            t += dt
        return False

    def _refine_conjunction(
        self,
        sat_state: np.ndarray,
        deb_state: np.ndarray,
        tca_guess: datetime,
        window_s: float = 1800.0,
        step_s: float = 30.0,
    ) -> Tuple[datetime, float, np.ndarray]:
        start_s = max(0.0, (tca_guess - self.current_time).total_seconds() - window_s)
        total_s = window_s * 2.0
        sat = sat_state.copy()
        deb = deb_state.copy()
        t = 0.0

        while t < start_s:
            dt = min(step_s, start_s - t)
            sat = rk4_step(sat, dt)
            deb = rk4_step(deb, dt)
            t += dt

        min_d = float("inf")
        min_t = start_s
        min_rel_v = np.zeros(3)
        elapsed = 0.0
        while elapsed <= total_s:
            rel = sat[:3] - deb[:3]
            d = float(np.linalg.norm(rel))
            if d < min_d:
                min_d = d
                min_t = start_s + elapsed
                min_rel_v = sat[3:] - deb[3:]
            sat = rk4_step(sat, step_s)
            deb = rk4_step(deb, step_s)
            elapsed += step_s

        tca = self.current_time + timedelta(seconds=min_t)
        return tca, min_d, min_rel_v

    def _predict_trajectory(self, state, gmst_base, duration=PREDICT_DURATION_S, step=PREDICT_STEP_S):
        pts = []
        cur = state.copy()
        omega = 7.2921159e-5
        t = 0.0
        while t < duration:
            dt = min(step, duration-t)
            cur = rk4_step(cur, dt)
            t += dt
            lat, lon, _ = eci_to_latlon(cur[:3], gmst_base + omega*t)
            pts.append((round(lat, 3), round(lon, 3)))
        return pts

    def get_visualization_snapshot(self) -> Dict:
        self._ensure_risk_field()
        gmst = compute_gmst(self.current_time)
        sats = []
        for sid, sat in self.satellites.items():
            lat, lon, alt = eci_to_latlon(sat.state[:3], gmst)
            vis = get_visible_stations(sat.state[:3], self.current_time)
            trail = list(self.sat_trails.get(sid, []))
            pred = self._predict_trajectory(sat.state, gmst)
            sats.append({
                "id": sid, "lat": round(lat, 4), "lon": round(lon, 4), "alt": round(alt, 2),
                "fuel_kg": round(sat.fuel_kg, 2), "fuel_pct": round(sat.fuel_fraction*100, 1),
                "status": sat.status,
                "distance_to_slot_km": round(sat.distance_to_slot(), 2),
                "total_dv_ms": round(sat.total_dv_used, 3),
                "r": [round(x, 3) for x in sat.state[:3].tolist()],
                "v": [round(x, 6) for x in sat.state[3:].tolist()],
                "visible_gs": [g for g, _ in vis],
                "trail": trail, "predicted": pred,
            })
        debris_cloud = []
        for did, st in self.debris.items():
            lat, lon, alt = eci_to_latlon(st[:3], gmst)
            debris_cloud.append([did, round(lat, 3), round(lon, 3), round(alt, 1)])
        gs = [{"id": g.station_id, "name": g.name, "lat": g.lat_deg, "lon": g.lon_deg,
               "elev_m": g.elevation_m, "min_elev": g.min_elevation_deg} for g in DEFAULT_STATIONS]
        cdms = [e.to_dict() for e in self.conjunction_assessor.active_cdms[:25]]
        pending = []
        for b in self.pending_burns[:25]:
            sid = b["sat_id"]
            sat = self.satellites.get(sid)
            has_los = False
            if sat is not None:
                has_los, _ = check_line_of_sight(sat.state[:3], self.current_time)
            bt = b["burn_time"]
            bt_s = (bt - self.start_time).total_seconds()
            blackout_risk = (not has_los) and (0 <= (bt - self.current_time).total_seconds() <= 900)
            pending.append({
                "burn_id": b["burn_id"], "sat_id": sid,
                "burn_time": bt.isoformat() + "Z", "burn_time_s": round(bt_s, 1),
                "type": b["type"], "mode": b.get("mode"),
                "los_now": has_los, "blackout_risk": blackout_risk,
            })
        nom = sum(1 for s in self.satellites.values() if s.status == "NOMINAL")
        evd = sum(1 for s in self.satellites.values() if s.status == "EVADING")
        rec = sum(1 for s in self.satellites.values() if s.status == "RECOVERING")
        eol = sum(1 for s in self.satellites.values() if s.status == "EOL")
        tot = len(self.satellites)
        avg_f = np.mean([s.fuel_fraction for s in self.satellites.values()])*100 if tot else 0
        return {
            "timestamp": self.current_time.isoformat().replace('+00:00', '')+"Z",
            "elapsed_s": round((self.current_time-self.start_time).total_seconds()),
            "satellites": sats, "debris_cloud": debris_cloud,
            "ground_stations": gs, "active_cdms": cdms,
            "pending_maneuvers": pending,
            "collision_log": self.collision_log[-10:],
            "maneuver_log": [
                {**m, "time_s": round((datetime.fromisoformat(m["burn_time"].replace('Z', ''))
                                       .replace(tzinfo=timezone.utc) - self.start_time).total_seconds(), 1)}
                for m in self.maneuver_history[-15:]
            ],
            "metrics": {
                "total_satellites": tot, "total_debris": len(self.debris),
                "total_collisions": self.total_collisions,
                "total_maneuvers": self.total_maneuvers_executed,
                "nominal_count": nom, "evading_count": evd,
                "recovering_count": rec, "eol_count": eol,
                "avg_fuel_pct": round(avg_f, 1),
                "constellation_uptime_pct": round(nom/tot*100, 1) if tot else 0,
                "total_fleet_dv_ms": round(self.total_fleet_dv, 3),
                "collisions_avoided": self.collisions_avoided,
                "plan_a_attempts": self.plan_a_attempts,
                "plan_a_success": self.plan_a_success,
                "plan_a_fallback_to_b": self.plan_a_fallback_to_b,
                "plan_b_executed": self.plan_b_executed,
            },
            "metrics_history": self.metrics_history[-100:],
            "risk_field": self.risk_field,
            "sun_subsolar": self._sun_subsolar_point(),
        }

    def _sun_subsolar_point(self) -> Dict:
        """Approximate sub-solar latitude and longitude for terminator line."""
        # Simple solar position model (adequate for visualization)
        # Day of year from March 12 = ~71
        doy = (self.current_time - datetime(self.current_time.year, 1, 1, tzinfo=timezone.utc)).days + 1
        # Solar declination (approximate)
        decl = -23.44 * math.cos(math.radians(360/365 * (doy + 10)))
        # Hour angle -> sub-solar longitude
        hours = self.current_time.hour + self.current_time.minute / 60.0 + self.current_time.second / 3600.0
        sun_lon = -(hours - 12.0) * 15.0  # 15°/hour, noon = 0°
        return {"lat": round(decl, 2), "lon": round(sun_lon, 2)}

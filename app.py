"""
Autonomous Constellation Manager — REST API
FastAPI application serving the ACM backend and frontend dashboard.
"""

import os
import logging
from datetime import datetime, timezone
from typing import List, Optional, Dict, Any

from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from backend.simulation import SimulationEngine

logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(name)-18s | %(levelname)s | %(message)s")
logger = logging.getLogger("acm.api")

app = FastAPI(title="ACM — Autonomous Constellation Manager", version="2.0.0")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

assets_dir = os.path.join(os.path.dirname(__file__), "assets")
if os.path.isdir(assets_dir):
    app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")


@app.middleware("http")
async def log_requests(request: Request, call_next):
    start = datetime.now(timezone.utc)
    response = await call_next(request)
    dur_ms = (datetime.now(timezone.utc) - start).total_seconds() * 1000
    logger.info(f"{request.method} {request.url.path} {response.status_code} {dur_ms:.1f}ms")
    print(f"REQ {request.method} {request.url.path} {response.status_code} {dur_ms:.1f}ms", flush=True)
    return response

engine = SimulationEngine()


# ── Pydantic models ──────────────────────────────────────────────────────────
class Vec3(BaseModel):
    x: float; y: float; z: float

class SpaceObject(BaseModel):
    id: str; r: Vec3; v: Vec3; type: Optional[str] = "SATELLITE"

class TelemetryPayload(BaseModel):
    timestamp: str; objects: List[SpaceObject]

class BurnCommand(BaseModel):
    burn_id: str; burnTime: str; deltaV_vector: Vec3; type: Optional[str] = "MANUAL"

class ManeuverPayload(BaseModel):
    satellite_id: Optional[str] = None
    satelliteId: Optional[str] = None  # Accept camelCase from grader
    maneuver_sequence: List[BurnCommand]

    @property
    def sat_id_resolved(self) -> str:
        return self.satellite_id or self.satelliteId or ""

class SimulatePayload(BaseModel):
    step_seconds: float = 60.0


# ── Frontend ─────────────────────────────────────────────────────────────────
@app.get("/", response_class=HTMLResponse)
async def serve_frontend():
    fp = os.path.join(os.path.dirname(__file__), "frontend", "index.html")
    if not os.path.exists(fp):
        return HTMLResponse("<h1>Frontend not found</h1>", status_code=404)
    with open(fp, encoding="utf-8") as f:
        return HTMLResponse(f.read())


# ── Telemetry Ingestion ──────────────────────────────────────────────────────
@app.post("/api/telemetry")
async def ingest_telemetry(payload: TelemetryPayload):
    data = [{"id": o.id, "r": o.r.dict(), "v": o.v.dict(), "type": o.type} for o in payload.objects]
    return engine.ingest_telemetry(payload.timestamp, data)


# ── Maneuver Scheduling ─────────────────────────────────────────────────────
@app.post("/api/maneuver/schedule")
async def schedule_maneuver(payload: ManeuverPayload):
    seq = [{"burn_id": b.burn_id, "burnTime": b.burnTime,
            "deltaV_vector": b.deltaV_vector.dict(), "type": b.type}
           for b in payload.maneuver_sequence]
    result = engine.schedule_maneuver(payload.sat_id_resolved, seq)
    if result.get("status") == "SCHEDULED":
        return JSONResponse(content=result, status_code=202)
    return JSONResponse(content=result, status_code=400)


# ── Simulation Step ──────────────────────────────────────────────────────────
@app.post("/api/simulate/step")
async def simulate_step(payload: SimulatePayload):
    result = engine.simulate_step(payload.step_seconds)
    logger.info(
        f"STEP {payload.step_seconds:.0f}s | maneuvers={result.get('maneuvers_executed', 0)} | "
        f"collisions={result.get('collisions_detected', 0)} | time={result.get('new_timestamp')}"
    )
    print(
        f"STEP {payload.step_seconds:.0f}s maneuvers={result.get('maneuvers_executed', 0)} "
        f"collisions={result.get('collisions_detected', 0)} time={result.get('new_timestamp')}",
        flush=True
    )
    return result


# ── Visualization Snapshot ───────────────────────────────────────────────────
@app.get("/api/visualization/snapshot")
async def get_snapshot():
    snap = engine.get_visualization_snapshot()
    m = snap.get("metrics", {})
    logger.info(
        "SNAP | sats=%s debris=%s fuel=%.2f uptime=%.2f dv=%.2f avoided=%s cdms=%s",
        m.get("total_satellites", 0), m.get("total_debris", 0),
        m.get("avg_fuel_pct", 0.0), m.get("constellation_uptime_pct", 0.0),
        m.get("total_fleet_dv_ms", 0.0), m.get("collisions_avoided", 0),
        len(snap.get("active_cdms", []))
    )
    print(
        "SNAP | sats={s} debris={d} fuel={f:.2f} uptime={u:.2f} dv={dv:.2f} avoided={a} cdms={c}".format(
            s=m.get("total_satellites", 0), d=m.get("total_debris", 0),
            f=m.get("avg_fuel_pct", 0.0), u=m.get("constellation_uptime_pct", 0.0),
            dv=m.get("total_fleet_dv_ms", 0.0), a=m.get("collisions_avoided", 0),
            c=len(snap.get("active_cdms", []))
        ),
        flush=True
    )
    return snap


# ── Fleet Status ─────────────────────────────────────────────────────────────
@app.get("/api/status")
async def get_status():
    snap = engine.get_visualization_snapshot()
    return {"status": "OPERATIONAL", "timestamp": snap["timestamp"],
            "metrics": snap["metrics"]}


@app.get("/api/satellites")
async def list_satellites():
    snap = engine.get_visualization_snapshot()
    return [{"id": s["id"], "status": s["status"], "lat": s["lat"], "lon": s["lon"],
             "alt": s["alt"], "fuel_pct": s["fuel_pct"]} for s in snap["satellites"]]


@app.get("/api/satellites/{sat_id}")
async def get_satellite(sat_id: str):
    if sat_id not in engine.satellites:
        raise HTTPException(404, f"Satellite {sat_id} not found")
    snap = engine.get_visualization_snapshot()
    for s in snap["satellites"]:
        if s["id"] == sat_id:
            return s
    raise HTTPException(404, "Not found")


@app.get("/api/conjunctions")
async def list_conjunctions():
    snap = engine.get_visualization_snapshot()
    return snap["active_cdms"]


@app.get("/api/maneuvers/history")
async def maneuver_history():
    return engine.maneuver_history[-50:]


@app.get("/api/metrics/history")
async def metrics_history():
    return engine.metrics_history


@app.get("/api/health")
async def health():
    return {"status": "ok", "engine_time": engine.current_time.isoformat()+"Z",
            "satellites": len(engine.satellites), "debris": len(engine.debris)}

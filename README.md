# Team MindMatrix - Autonomous Constellation Manager (ACM)

National Space Hackathon 2026 submission - Team MindMatrix.

This repository implements an end-to-end autonomous constellation operations stack:
- High-frequency orbital telemetry ingestion
- Physics-based propagation and conjunction screening
- Autonomous maneuver scheduling under fuel and cooldown constraints
- Real-time mission dashboard (3D globe + Mercator ground track)

## 0. Problem Statement (Short)

Space is increasingly crowded with active satellites and debris, making collision avoidance harder to manage manually at scale.

We track satellites and space debris, predict possible collisions, and automatically plan safe movements while saving fuel.


## 1. Highlights

- FastAPI backend on port 8000
- RK4 integration with two-body + J2 dynamics
- KD-tree conjunction broad-phase screening
- Maneuver planning: evasion, recovery, station-keeping, graveyard
- Ground-station line-of-sight aware command scheduling
- Operator dashboard with manual and auto simulation controls

## 1.1 USP: Dual-Layer Collision Mitigation

Our core differentiator is not only detecting conjunctions, but selecting and executing the best mitigation mode automatically:

1. Plan A - Collision-Laser Risk Reduction (non-contact)
- When encounter geometry is favorable, the engine attempts a laser-style debris deflection strategy first.
- This is designed to increase miss distance without spending satellite fuel.
- The system computes feasibility and confidence before committing to this action.

2. Plan B - Autonomous Evasion Burns (fuel-aware fallback)
- If Plan A is not feasible or confidence is below threshold, the engine falls back to classical satellite evasion burns.
- Evasion and recovery burns are planned under fuel, cooldown, and operational constraints.
- This guarantees mission continuity when non-contact mitigation is not reliable.


Operational loop in one line:
Detect risk -> evaluate laser feasibility -> apply laser if confident -> otherwise execute evasion -> recover nominal orbit.

## 2. Core Mathematical Model

### 2.1 State Dynamics

Each object is represented in ECI coordinates by:

$$
\mathbf{x} = [x, y, z, v_x, v_y, v_z]^T
$$

Propagation uses:

$$
\dot{\mathbf{r}} = \mathbf{v},\qquad
\dot{\mathbf{v}} = -\mu\frac{\mathbf{r}}{\|\mathbf{r}\|^3} + \mathbf{a}_{J2}
$$

with J2 perturbation:

$$
\mathbf{a}_{J2}=\frac{3J_2\mu R_E^2}{2\|\mathbf{r}\|^5}
\begin{bmatrix}
x\left(5\frac{z^2}{\|\mathbf{r}\|^2}-1\right) \\
y\left(5\frac{z^2}{\|\mathbf{r}\|^2}-1\right) \\
z\left(5\frac{z^2}{\|\mathbf{r}\|^2}-3\right)
\end{bmatrix}
$$

### 2.2 Numerical Integration (RK4)

$$
\mathbf{k}_1=f(\mathbf{x}_n),\;
\mathbf{k}_2=f\left(\mathbf{x}_n+\frac{\Delta t}{2}\mathbf{k}_1\right),\;
\mathbf{k}_3=f\left(\mathbf{x}_n+\frac{\Delta t}{2}\mathbf{k}_2\right),\;
\mathbf{k}_4=f\left(\mathbf{x}_n+\Delta t\mathbf{k}_3\right)
$$

$$
\mathbf{x}_{n+1}=\mathbf{x}_n+\frac{\Delta t}{6}(\mathbf{k}_1+2\mathbf{k}_2+2\mathbf{k}_3+\mathbf{k}_4)
$$

### 2.3 Collision Probability Approximation

$$
P_c \approx \frac{R^2}{2\sigma^2}\exp\left(-\frac{d^2}{2\sigma^2}\right)
$$

### 2.4 Fuel Update (Tsiolkovsky)

$$
\Delta m = m_0\left(1-\exp\left(-\frac{\Delta v}{I_{sp}g_0}\right)\right)
$$

## 3. Architecture

- API Layer: route handling and contract compliance
- Simulation Engine: time-stepping, event orchestration, metrics
- Physics Layer: propagator, conjunction, maneuver, LOS modules
- Data Layer: TLE ingest/cache + ground station catalog
- Frontend: Orbital Insight mission UI

Main files:
- app.py
- backend/simulation.py
- backend/physics/propagator.py
- backend/physics/conjunction.py
- backend/physics/maneuver.py
- backend/physics/ground_station.py
- frontend/index.html

## 4. API Contract

### Required endpoints
- POST /api/telemetry
- POST /api/maneuver/schedule
- POST /api/simulate/step

### Visualization and diagnostics
- GET /api/visualization/snapshot
- GET /api/health
- GET /api/status
- GET /api/satellites
- GET /api/conjunctions

### Example: simulation step

```json
POST /api/simulate/step
{
	"step_seconds": 3600
}
```

### Example: maneuver scheduling

```json
POST /api/maneuver/schedule
{
	"satelliteId": "SAT-Alpha-04",
	"maneuver_sequence": [
		{
			"burn_id": "EVASION_1",
			"burnTime": "2026-03-12T14:15:30.000Z",
			"deltaV_vector": {"x": 0.002, "y": 0.015, "z": -0.001},
			"type": "EVASION"
		}
	]
}
```

## 5. Local Setup

### 5.1 Create environment and install

```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
pip install -r .\requirements.txt
```

### 5.2 Run backend + frontend

```powershell
.\venv\Scripts\python.exe -m uvicorn app:app --host 127.0.0.1 --port 8000
```

Open:
- http://127.0.0.1:8000/

## 6. Docker Setup (Submission Requirement)

```bash
docker build -t team-mindmatrix-acm .
docker run --rm -p 8000:8000 team-mindmatrix-acm
```

Compliance:
- Base image: ubuntu:22.04
- Exposed port: 8000
- Uvicorn bind: 0.0.0.0

## 7. Dashboard Features

- Manual controls: +1 MIN, +1 HR, +1 DAY
- AUTO mode with in-flight step protection
- 3D globe with trails + predicted paths
- Mercator ground track map with declutter modes
- Conjunction and maneuver situational panels
- Satellite-focused operational inspection by selecting a target from fleet list

## 8. Data Sources

- data/ground_stations.csv
- data/tle_active.txt
- data/tle_debris.txt
- assets/Mercator-projection.jpg

## 9. Submission Artifacts

- Final LaTeX report source: overleaf/Team_MindMatrix_Final_Report.tex
- Video speaking script: VIDEO_DEMONSTRATION_SCRIPT.md
- Audit checklist: SUBMISSION_AUDIT.md

## 10. Known Notes

- Local PDF generation is environment-dependent (no TeX engine installed in this workspace).
- Report compiles directly in Overleaf from overleaf/Team_MindMatrix_Final_Report.tex.

## 11. Team

Team MindMatrix

# Submission Audit - AOSE ACM

Date: 2026-04-03
Workspace: space-hackathon

## 1) Scope

This audit checks submission readiness for:
- Required API contract behavior
- Frontend control reliability for simulation stepping
- Docker/runtime packaging basics
- Documentation completeness and report artifacts

## 2) Critical Findings and Fixes

### Finding A - Simulation controls could appear non-responsive

Observed risk:
- Repeated or overlapping `/api/simulate/step` calls can flood the backend event loop.
- Large jumps (especially +1 DAY) can make the UI look frozen while backend work is in progress.

Fix applied:
- Frontend simulation controls are now serialized through a queue.
- Large manual time jumps are chunked into smaller backend steps (`600s` chunks).
- Header buttons (`+1 MIN`, `+1 HR`, `+1 DAY`) are disabled while a step is running.
- A small status text now shows stepping progress for long operations.
- Auto-run loop is throttled and only sends a new step when no step is already in flight.

File changed:
- `frontend/index.html`

## 3) API Smoke Test Results

Executed endpoint checks locally against `http://127.0.0.1:8000`.

Results:
- `GET /api/health` -> OK (`status=ok`)
- `GET /api/visualization/snapshot` -> OK (`sats=121`, `debris=1856`)
- `POST /api/simulate/step` -> OK (`status=STEP_COMPLETE`)
- `POST /api/telemetry` -> OK
- `POST /api/maneuver/schedule` -> OK (`HTTP 202`)

Conclusion:
- Required endpoint surface is live and functional.

## 4) Packaging and Config Review

### Dockerfile

Passes submission baseline checks:
- Base image: `ubuntu:22.04`
- Exposes port `8000`
- Starts `uvicorn` on `0.0.0.0:8000`

### Requirements

Core dependencies are present:
- fastapi, uvicorn
- numpy, scipy
- pydantic
- sgp4
- requests

### Data and Assets

Verified expected assets/datasets present:
- `data/ground_stations.csv`
- `data/tle_active.txt`
- `data/tle_debris.txt`
- `assets/Mercator-projection.jpg`

## 5) Documentation Readiness

Existing docs reviewed:
- `README.md`

Added in this audit:
- `SUBMISSION_AUDIT.md` (this file)
- `overleaf/Team_Kamal_Final_Report.tex` (final LaTeX report for Overleaf/PDF export)

## 6) Remaining Submission Checklist

Before final upload, verify:
- Demo video (<5 minutes) is recorded and includes:
  - live conjunction alert
  - successful maneuver scheduling
  - map + globe trajectory views
  - AUTO and manual stepping controls
- PDF report exported from Overleaf build of the `.tex` file.
- Public repository includes latest frontend fix commit.

## 7) Audit Verdict

Status: READY WITH REPORT EXPORT PENDING

Rationale:
- Functional/API checks pass.
- Reliability fix for control buttons is in place.
- Docker and docs meet baseline requirements.
- Final remaining action is producing and attaching the report PDF from the supplied LaTeX source.

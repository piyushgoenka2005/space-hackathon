# Team Kamal - Video Demonstration Script (Under 5 Minutes)

Use this as a direct speaking script. Follow the timestamp, action, and exact lines.

## Total Duration Target

- 4 minutes 30 seconds to 4 minutes 50 seconds

## 00:00 - 00:20 | Opening

Action:
- Show project title slide or repository homepage.

Say exactly:
- Hello judges, we are Team Kamal.
- This is our Autonomous Constellation Manager for real-time debris avoidance and constellation operations.
- We will demonstrate the architecture, core algorithms, APIs, and live dashboard behavior.

## 00:20 - 01:00 | Problem + Approach

Action:
- Show README architecture section or system diagram.

Say exactly:
- Our goal is to keep satellites safe in a dense debris environment while minimizing fuel and maintaining uptime.
- The system has five layers: frontend dashboard, FastAPI services, simulation engine, physics modules, and data ingestion.
- The control loop is: propagate, screen conjunctions, score risk, schedule maneuver, and refresh telemetry.

## 01:00 - 01:45 | Numerical Methods (with equations)

Action:
- Show README math equations or Team_Kamal_Final_Report.tex equations section.

Say exactly:
- We model each object in ECI state as position and velocity.
- Dynamics use two-body gravity with J2 perturbation.
- Time propagation uses fixed-step RK4 for stable integration.
- Initial states are seeded using SGP4 from TLE catalogs.
- This gives us a practical balance between physical fidelity and runtime performance.

## 01:45 - 02:25 | Spatial Optimization + Risk

Action:
- Show conjunction module mention in README and then dashboard alerts panel.

Say exactly:
- For conjunction screening, we use a KD-tree broad phase instead of brute-force all-pairs checks.
- This reduces computational load significantly for large debris sets.
- For collision likelihood, we use a Chan-style probability approximation based on miss distance and uncertainty.
- Risk events are then prioritized for autonomous planning.

## 02:25 - 03:10 | Maneuver Logic + Constraints

Action:
- Show maneuver section in README and then live metrics/controls in dashboard.

Say exactly:
- Maneuver planning is constrained by per-burn delta-v limits, cooldown windows, and remaining fuel.
- Fuel depletion is computed using the Tsiolkovsky equation.
- The planner balances risk reduction against fuel use and service disruption.
- We support evasion, recovery, station-keeping, and operational scheduling via API.

## 03:10 - 04:00 | Live UI Demonstration

Action:
- Open dashboard at http://127.0.0.1:8000/
- Press +1 MIN, +1 HR, and AUTO for a few seconds.
- Show map view and globe view.

Say exactly:
- Here is the live Orbital Insight dashboard.
- We have manual time controls and auto mode for continuous simulation.
- We hardened controls to avoid overlapping step requests and to keep the UI responsive during larger jumps.
- The globe and Mercator map provide orbit context, trajectories, and conjunction visibility for operations.

## 04:00 - 04:30 | API Compliance + Close

Action:
- Show terminal or docs with endpoints list.

Say exactly:
- We implemented all required endpoints: telemetry ingestion, maneuver scheduling, and simulation stepping.
- We also provide visualization snapshots and health endpoints for monitoring.
- Docker packaging and port 8000 compliance are included for evaluation.
- Thank you. This is Team Kamal.

---

## Delivery Tips

- Speak clearly and slightly slower than normal.
- Keep cursor movement intentional and minimal.
- If a live call is slow, narrate what is expected and continue.
- Keep this exact sequence to stay under five minutes.

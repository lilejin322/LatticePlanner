# Planning / Lattice tests

> These commands assume you have created and activated your own virtual environment (venv or conda) and installed the dependencies from the repo-root `requirements.txt` (`pip install -r requirements.txt`). This project does not ship a virtual environment.

## Scripts

| Script | Purpose |
|------|------|
| `run_lattice_component_checks.py` | Lattice, obstacle, ST-graph, reference-line, and geometry regressions |
| `run_lattice_scenario_cases.py` | Closed-loop scenarios on a fetched CommonRoad lanelet. Tags: `lattice`, `stress`, `sim` |
| `run_lattice_demo_cases.py` | Short demo (a subset of the scenario set) |
| `run_minimal_lattice_plan.py` | One straight centerline and `LatticePlanner.Plan` |
| `run_closed_loop_sim.py` | Closed loop on a fetched CommonRoad lanelet |
| `fetch_commonroad_scenarios.py` | Clone the public scenario repository into gitignored `data/commonroad-scenarios/` |
| `run_lattice_known_failures.py` | Three recorded lattice defects on fetched scenarios. Passes while the defect signature still matches |
| `run_lattice_dataset_sweep.py` | Classify a fetched scenario set. Writes a gitignored report; not a pass/fail gate |
| `run_all_scenario_animations.py` | GIFs for scenarios that publish a trajectory |
| `run_all_planner_checks.py` | Component checks, the minimal plan, the closed loop, and the recorded defects |

## Quick start

```bash
python scripts/run_lattice_scenario_cases.py --list

# Closed loop: open road, follow a slower leader, follow a stopped leader, and the loaded map
python scripts/run_closed_loop_sim.py
python scripts/run_lattice_scenario_cases.py --tag sim

# Run every scenario
python scripts/run_lattice_scenario_cases.py

# Lattice core only
python scripts/run_lattice_scenario_cases.py --tag lattice

# One case, or several
python scripts/run_lattice_scenario_cases.py open_road mid_lane_start stress_obstacle_distance_sweep

# Scenario animation: one frame every 0.1s, written as a GIF
python scripts/run_lattice_scenario_cases.py open_road far_obstacle_stop --animate

# Also export per-frame PNGs (frame_0000.png, ...)
python scripts/run_lattice_scenario_cases.py open_road --animate --animate-frames

# Short demo
python scripts/run_lattice_demo_cases.py --animate

# Every scenario animation GIF
python scripts/run_all_scenario_animations.py
python scripts/run_all_scenario_animations.py --list
```

Scenario tags:

| `--tag` | Contents |
|---------|------|
| `lattice` | LatticePlanner on the lanelet centerline: open road, obstacles, backup, initial-state changes |
| `stress` | Obstacle-distance and initial-speed sweeps (informational; not a failure gate) |
| `sim` | Closed-loop regression for the open road, a slower leader, and a stopped leader |

Lattice follows the centerline and stops behind a blocking vehicle. Actors hold a constant speed. There is no lane-borrow or traffic-rule task chain.

**`--animate`** plays the executed trace with matplotlib at **0.1s** per frame. The default output is `scripts/output/animations/<scenario>.gif`. `python scripts/run_closed_loop_sim.py --animate` draws the same drive with the CommonRoad renderer on the loaded lanelets.

```bash
python scripts/run_all_planner_checks.py

# or step by step
python scripts/run_lattice_component_checks.py
python scripts/run_minimal_lattice_plan.py
python scripts/run_closed_loop_sim.py
python scripts/run_lattice_known_failures.py
```

## Recorded defects and the dataset sweep

`run_lattice_known_failures.py` plans once on three fetched files:

| Scenario | Recorded kind | What it locks |
|----------|---------------|---------------|
| `USA_Peach-3_1_T-1` | `numeric_blowup` | backup trajectory first point outside the 1e5 m guard |
| `ESP_Monzon-7_5_T-1` | `negative_speed_assert` | negative Frenet speed raises `AssertionError` |
| `ZAM_Tjunction-1_266_T-1` | `left_road` | within 1 s a committed point jumps off the ego |

The command exits 0 while those signatures still match. When a guard removes a defect, change that row's expected kind to `contained` in `scripts/run_lattice_known_failures.py`. The 0.1 s constraint sampling checked by Z3 is not part of this command.

`run_lattice_dataset_sweep.py` is the discovery run. It does not fail the process. Counts are split into numeric blow-ups, negative-speed assertions, far matches, executed acceleration outside [-6, 4] m/s², collisions, and backup-only cycles. The default selection is recorded scenarios with at most 8 dynamic obstacles and a 1 s horizon. The report goes to `scripts/output/lattice_sweep/report.jsonl`.

```bash
python scripts/run_lattice_dataset_sweep.py --limit 20
python scripts/run_lattice_dataset_sweep.py
```

## Covered by the component checks

- 1D trajectory extrapolation, backup, and the dynamic-obstacle PathTimeGraph
- Constraint checks, including the sampled longitudinal and lateral bounds
- Collision checking and ST-boundary construction
- Reference-line projection, lane width, and speed limit on a point centerline
- Polygon, box, and path-approximation geometry
- A fetched CommonRoad scenario file loads
- A loaded CommonRoad file is driven without adding lanelets
- Three recorded lattice defects still show their saved signatures

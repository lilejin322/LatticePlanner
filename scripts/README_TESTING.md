# Planning / Lattice tests

> These commands assume you have created and activated your own virtual environment (venv or conda) and installed the dependencies from the repo-root `requirements.txt` (`pip install -r requirements.txt`). This project does not ship a virtual environment.

## Scripts

| Script | Purpose |
|------|------|
| `run_lattice_component_checks.py` | Main regression: lattice, reference line, traffic rules, PathAssessment, PathBounds, OnLanePlanning, and related pieces |
| `run_extended_planner_checks.py` | Extended regression: Frenet derivatives, obstacle scope, large-lateral PathData, borrow bounds, blocking plus backup, legacy switches |
| `run_lattice_scenario_cases.py` | Scenario tests: lattice / decider / on_lane / stress / overtake / lane_change, plus the closed-loop `sim` tag. Use `--list` for the current count |
| `run_lattice_demo_cases.py` | Short demo (6 cases, a subset of the full scenario set) |
| `run_minimal_lattice_plan.py` | Minimal end-to-end smoke test (one reference line plus `LatticePlanner.Plan`) |
| `run_closed_loop_sim.py` | **Closed-loop simulation** on the converted Borregas Ave lanelets: replan every cycle and advance ego and dynamic actors |
| `run_overtake_animations.py` | **Overtake** tests plus animation (shortcut for `--tag overtake --animate`) |
| `run_all_scenario_animations.py` | GIFs for every scenario, including overtake |
| `run_all_planner_checks.py` | Run the regression scripts in order (no animation) |

## Quick start

```bash
# Full scenario list, including the closed-loop sim tag
python scripts/run_lattice_scenario_cases.py --list

# Closed loop: open road, follow a slower leader, follow a stopped leader, and the loaded map
python scripts/run_closed_loop_sim.py
python scripts/run_lattice_scenario_cases.py --tag sim

# Run every scenario
python scripts/run_lattice_scenario_cases.py

# Lattice core only (18 cases on the converted Borregas lanelet)
python scripts/run_lattice_scenario_cases.py --tag lattice

# One case, or several
python scripts/run_lattice_scenario_cases.py open_road mid_lane_start stress_obstacle_distance_sweep

# Scenario animation: one frame every 0.1s on the XY map, written as a GIF
python scripts/run_lattice_scenario_cases.py open_road far_obstacle_stop --animate

# Also export per-frame PNGs (frame_0000.png, ...)
python scripts/run_lattice_scenario_cases.py open_road --animate --animate-frames

# Open the GIF on macOS
open scripts/output/animations/open_road.gif

# Short 6-case demo
python scripts/run_lattice_demo_cases.py --animate

# Every scenario animation GIF
python scripts/run_all_scenario_animations.py
python scripts/run_all_scenario_animations.py --frames   # also save per-frame PNGs
python scripts/run_all_scenario_animations.py --list    # preview which GIFs would be written
```

Scenario tags:

| `--tag` | Contents |
|---------|------|
| `lattice` | LatticePlanner only: open road, obstacles, backup, initial-state changes |
| `decider` | PathBounds / Assessment / Combine / cruise speed |
| `on_lane` | End-to-end `OnLanePlanning.RunOnce`, including the PathBounds borrow-to-overtake branch |
| `stress` | Obstacle-distance and initial-speed sweeps (informational; not a failure gate) |
| `overtake` | **Closed-loop overtake**: recompute the PathBounds borrow each cycle (or follow with lattice only). The animation plays the executed trace |
| `lane_change` | **Closed loop**: recompute the borrow corridor each cycle and use the lateral QP to pass a slow vehicle |
| `sim` | Closed-loop regression for the open road, a slower leader, and a stopped leader |

**Overtake animation (start with `overtake_path_bounds_left`)**:

```bash
python scripts/run_lattice_scenario_cases.py overtake_path_bounds_left --animate
# or every overtake scenario
python scripts/run_overtake_animations.py
```

Lattice alone can only follow and stop behind a blocking vehicle. An overtake goes through OnLane, the PathBoundsDecider borrow, PathAssessment, and an S-shaped l(s), matching the C++ lane-follow task chain.

**`--animate`** on the scenario runner plays the executed trace with matplotlib at **0.1s** per frame. The default output is `scripts/output/animations/<scenario>.gif`. `--animate-frames` also writes `scripts/output/frames/<scenario>/frame_0000.png`, and so on. `python scripts/run_closed_loop_sim.py --animate` draws the same drive with the CommonRoad renderer on the loaded lanelets.

```bash
# Run the full regression
python scripts/run_all_planner_checks.py

# or step by step
python scripts/run_lattice_component_checks.py
python scripts/run_extended_planner_checks.py
python scripts/run_minimal_lattice_plan.py
```

## Covered by the main script

- 1D trajectory extrapolation, backup, and the dynamic-obstacle PathTimeGraph
- HDMap / PncMap / ReferenceLineProvider / relative map
- TrafficDecider (stop line, yield, KeepClear)
- PathDecider static nudge, and PathDecider after the lateral lattice
- QP reference-line smoothing and trajectory stitching
- PathAssessment / PathBounds / XY distance / debug recording (decider unit checks)
- OnLanePlanning output
- The pure lattice main path (aligned with `lattice_planner.cc`)

## Covered by the extended script

- `CalculateLateralDerivative` / `GetFrenetPoint` consistency
- `IsWithinPathDeciderScopeObstacle` filtering
- XY fallback for a large-lateral `BuildLatticeCandidatePath`
- Paths that leave the reference line or collide are invalid
- `ComparePathData` prefers the longer in-lane path
- `PathBoundary.boundary()` API
- PlanningContext borrow produces left and right boundary candidates
- A blocked road with backup still plans (LatticePlanner core)
- Default pure-lattice path (`check_lattice_default_plan`)
- PathDecider and PathAssessment together
- `SetPathInfo` skips an empty discretized path

## Still worth adding (needs more scenarios or data)

1. **Multi-reference-line lane change**: `OnLanePlanning` + `FLAGS_cost_non_priority_reference_line` + a parallel lane
2. **PathDecider after lateral**: the full lattice chain with `FLAGS_enable_path_decider=True` (the extended checks already cover one PathDecider call after assessment)
3. **Boundary candidates only, no feasible lattice pair**: PathBounds + PathAssessment + `CombinePathAndSpeedProfile` (extended check `boundary_only_path_assessment_combine`)
4. **Blocked and no backup**: pure lattice should fail (extended check `lattice_fails_when_blocked_without_backup`); borrow bounds are covered by `path_bounds_lane_borrow_from_context`
5. **Curved road / high-curvature reference line**: stability of `SetFrenetPath` and `CombinePathAndSpeedProfile`
6. **OnLanePlanning final aggregation**: `AggregateReferenceLineTrajectory` (extended check `on_lane_aggregate_path_speed`)
7. **Pullover / open space**: the other PlanningContext scenario state machines
8. **Full traffic-rule set**: per-rule checks for REFERENCE_LINE_END, CREEP, REROUTING, and the rest

## Adding a case

Append `(name, fn)` to the `CHECKS` list in `run_extended_planner_checks.py`, and reuse the builders in `planner_test_fixtures.py` instead of copying `_build_reference_line`.

[![GPLv3 License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Python port of Apollo's lattice planner, kept so the planner itself can be tested. The polynomials, costs, trajectory combiner, and 0.1 s constraint checks are the subject. CommonRoad supplies the road and the recorded vehicles. This is not a traffic simulator and not a new planning method.

The work stops at the defect list. A plain account is in [docs/说明.md](docs/说明.md).

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

`commonroad-io` stays on `protobuf` 3.20.2. Do not upgrade protobuf.

## CommonRoad scenarios

Scenario XML is not in this repository. The fetch script shallow-clones the public [commonroad-scenarios](https://gitlab.lrz.de/tum-cps/commonroad-scenarios) branch `2020a_scenarios` into gitignored `data/commonroad-scenarios/`:

```bash
python scripts/fetch_commonroad_scenarios.py
```

Running it again leaves an existing clone in place. That checkout is the public 2020a set (on the order of 18k XML files). The larger catalog on commonroad.in.tum.de asks for a login and is not what this script downloads.

Closed-loop cases that place their own actors use the longest lanelet in `DEU_A9-2_1_T-1`. An actor's `x` is station along that centerline and `y` is lateral offset. Those actors hold a constant speed. Recorded vehicles in a loaded file are read with `state_at_time` and `occupancy_at_time`. Nothing here builds lanelets or generates traffic.

## What to run

From the repo root, after the dataset is present:

```bash
python scripts/run_all_planner_checks.py
```

That runs, in order:

| Command | What it checks |
|---------|----------------|
| `scripts/run_lattice_component_checks.py` | Polynomials, backup, ST graph, collision, one cycle on a loaded scenario |
| `scripts/run_minimal_lattice_plan.py` | One lattice cycle on a straight lanelet, no obstacles |
| `scripts/run_lattice_cycle_checks.py` | Cruise and stop samples, cost order, fallback, the next cycle's init |
| `scripts/run_lattice_known_failures.py` | Four recorded defects on fetched scenarios. Passes while each signature is still present |
| `scripts/run_lattice_synthetic_failures.py` | Three straight-road defects. Passes while each signature is still present |
| `verification/pinned_sampling_gaps.py` | Two fixed curves the 0.1 s checker still misses |
| `verification/verify_backup_jerk.py` | Backup deceleration reports jerk 0 across its stop |
| `verification/verify_extrapolation.py` | A 1d-valid quartic exceeds the speed bound after it ends |
| `verification/verify_cartesian_jerk.py` | The Cartesian 0.1 s jerk difference misses the pinned spike |

The same pieces one at a time:

```bash
python scripts/verify_imports.py
python scripts/run_lattice_component_checks.py
python scripts/run_minimal_lattice_plan.py
python scripts/run_lattice_cycle_checks.py
python scripts/run_lattice_known_failures.py
python scripts/run_lattice_synthetic_failures.py
python verification/pinned_sampling_gaps.py
python verification/verify_backup_jerk.py
python verification/verify_extrapolation.py
python verification/verify_cartesian_jerk.py
```

`scripts/run_closed_loop_sim.py` and `scripts/run_lattice_scenario_cases.py` still replan on the A9 lanelet and score how far the ego traveled. They are not part of the gate above. `--animate` writes a GIF under `scripts/output/animations/`.

A discovery sweep over fetched files is not a pass/fail gate. It writes `scripts/output/lattice_sweep/report.jsonl` (gitignored) and separates numeric blow-ups, negative-speed assertions, commits that leave the ego, acceleration outside [-6, 4] m/s², collisions, and backup-only cycles:

```bash
python scripts/run_lattice_dataset_sweep.py --limit 20
python scripts/run_lattice_dataset_sweep.py
```

The default selection is recorded scenarios with at most 8 dynamic obstacles and a 1 s horizon.

## Recorded defects

`run_lattice_known_failures.py` exits 0 while these signatures still match. They are measurements of this planner. When a change removes one, set that row's expected kind to `contained` in `scripts/run_lattice_known_failures.py`.

| Scenario | Kind | What is still true |
|----------|------|--------------------|
| `USA_Peach-3_1_T-1` | `numeric_blowup` | The backup trajectory's committed point is outside the 1e5 m guard |
| `ESP_Monzon-7_5_T-1` | `negative_speed_assert` | A negative Frenet speed raises `AssertionError` instead of a failed plan |
| `ZAM_Tjunction-1_266_T-1` | `left_road` | The first cycle stays on the ego. At 0.7 s the backup trajectory's committed point jumps away from that cycle's init |
| `FRA_MoissyCramayel-1_1_T-1` | `accel_bound` | Replanning from each committed point, a returned backup changes speed by about -8.9 m/s² over 0.1 s. That trajectory's sample accelerations stay inside [-6, 4] m/s² |

`run_lattice_synthetic_failures.py` is the same kind of record for inputs that do not need a fetched file. A straight lanelet, one call to `PlanOnLanelet`.

| Case | Kind | What is still true |
|------|------|--------------------|
| `stopped_heading` | `fallback_blowup` | Speed 0 and heading 1 rad have a finite Frenet state. Backup still returns a point near y = 2.14×10⁹ m with speed about 2143 m/s, and the cartesian checker rejects the speed |
| `reverse_heading` | `negative_speed_assert` | Heading π at 1 m/s makes Frenet speed negative and raises `AssertionError` |
| `backup_stop_jerk` | `backup_jerk_blind` | `ConstantDecelerationTrajectory1d` reports jerk 0 across the stop, so the 1d checker accepts a −4 m/s² curve. On an 8 m stopped car the combined samples change acceleration from −4 to 0 in one 0.1 s step, jerk 40 m/s³, and that trajectory is still returned |

## SMT checks

`verification/` uses Z3 to ask whether a bound violation exists anywhere on a continuous interval, not only at the 0.1 s samples the checker looks at. A `sat` result is a counterexample: the sampled checker says valid, and the polynomial is outside a bound between samples. These misses are left in the checker on purpose.

`common/constraint_checker1d.py` walks `while t < ParamLength(): t += 0.1`. The last sliver before the end is never evaluated, so a jerk or acceleration spike there is invisible to the checker. Duration is almost never an exact multiple of 0.1 s.

```bash
python verification/verify_constraint_checker1d.py [num_random] [seed]
python verification/verify_lateral_constraint1d.py 200 1
```

`num_random` (default 2000) is how many random curves to try per family. `seed` (default 0) keeps a run repeatable. A longitudinal example the checker accepts, with jerk past -4 m/s³ at t=1.84375 s inside a curve that ends at 1.8694 s:

```text
tested=4000 checker_said_valid=38 counterexamples=1
[quartic] start=[0.0, 30.93, 2.39] end=[30.57, -3.38] T=1.8694 -> checker says VALID, but at t=1.84375: v=30.6554 a=-3.2763 j=-4.0306
```

`pinned_sampling_gaps.py` builds that quartic from the boundary values above. The search print had rounded them; the jerk at the rounded values is -4.0306 m/s³.

The lateral script uses the same loop on lateral acceleration. One miss sits between the sample at 0.5 s and the unchecked end at 0.5536 s, with lateral acceleration past ±4 m/s²:

```bash
python verification/verify_lateral_constraint1d.py 200 1
```

```text
tested=400 checker_said_valid=43 counterexamples=3
[quartic] T=0.5536 S=40 -> checker says VALID, but at t=0.531250: a=-4.3781 j=0.2919
```

`verification/verify_constraint_checker.py` asks the same question for `ConstraintChecker.ValidTrajectory` on the Cartesian trajectory the planner actually accepts. The reference line in that proof is perfectly straight, so the Frenet-to-Cartesian formulas stay closed form. Curved reference lines are outside that proof. Longitudinal jerk there is a difference between two samples. `verify_cartesian_jerk.py` proves the difference from 1.8 s to 1.9 s stays inside [-4, 2] for the pinned quartic, while the jerk at 1.84375 s is -4.0306 m/s³. The curve is wrapped the way the planner wraps it, so acceleration is frozen after the duration.

`verify_backup_jerk.py` is not a polynomial search. For every backup deceleration except -0.1 m/s², and every initial speed in (0, 40] m/s, the stop falls in some 0.1 s sample interval whose finite-difference jerk is |a|/0.1 and exceeds +2 m/s³. The curve reports jerk 0 at every sample, so the 1d checker accepts it.

`verify_extrapolation.py` uses a quartic the 1d checker accepts on [0, 2). Constant-acceleration extrapolation reaches 59 m/s at the 8 s combiner horizon. The speed bound is 40 m/s.

`verification/pinned_sampling_gaps.py` replays the two curves above and passes while the checker still misses them. `FRA_MoissyCramayel-1_1_T-1` is the same kind of hole on a trajectory the planner returned: the speed change between the planning init and the committed point is about -8.9 m/s², while the accelerations stored on the 0.1 s samples stay inside [-6, 4].

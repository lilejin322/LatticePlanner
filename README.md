[![GPLv3 License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is a **Python translation for learning** Baidu Apollo's lattice planner (not for a production simulator). After refactoring, all modules should import cleanly for reading and white-box study.

## QuickStart

### Setup

Create and activate your own virtual environment (venv, conda, etc. — none is bundled with this repo), then:

```bash
pip install -r requirements.txt
```

The lattice **planning pipeline** in `lattice_planner.py` is the main readable path.

### Test against the original C++

Run the regression test suite from the repo root after activating your environment:

```bash
python scripts/verify_imports.py            # every module imports cleanly
python scripts/run_all_planner_checks.py    # component + extended unit checks
python scripts/run_lattice_demo_cases.py    # smoke scenarios (open road, stop, backup fallback, ...)
python scripts/run_lattice_scenario_cases.py  # broader scenario coverage, incl. lane-borrow overtakes
```

## Formal Verification via SMT (demo)

The regression tests above only check a trajectory at fixed time steps (`FLAGS_trajectory_time_resolution`). `verification/` holds a small Z3-based (SMT solver) proof-of-concept that instead checks a trajectory analytically over the *entire* continuous time interval, to catch bound violations that fall between two samples.

`verification/verify_constraint_checker1d.py` targets `common/constraint_checker1d.py`'s `IsValidLongitudinalTrajectory`, which samples a closed-form polynomial curve (`QuinticPolynomialCurve1d` / `QuarticPolynomialCurve1d`) every 0.1s and checks velocity/acceleration/jerk bounds only at those points. The script:

1. Generates random candidate curves within the planner's own start/end/duration ranges.
2. Keeps only the ones the real checker already marks `valid`.
3. Encodes each curve's velocity/acceleration/jerk (the polynomial's exact derivatives, as Z3 rationals matching Python's floats bit-for-bit) as an SMT formula, and asks Z3 whether a bound violation exists anywhere on `[0, duration]` — not just at the sampled points.
4. A `sat` result is a genuine counterexample: a trajectory the sampled checker calls safe that actually leaves the feasible envelope between two samples.

Run it from the repo root (`z3-solver` is already in `requirements.txt`):

```bash
python3 verification/verify_constraint_checker1d.py [num_random] [seed]
```

`num_random` (default 2000) is how many random candidates to try per curve type; `seed` (default 0) makes runs reproducible. Sample output:

```
tested=4000 checker_said_valid=38 counterexamples=1
[quartic] start=[0.0, 30.93, 2.39] end=[30.57, -3.38] T=1.8694 -> checker says VALID, but at t=1.843750: v=30.6592 a=-3.2740 j=-4.0280
```

This already found a real, structural bug: the checker's `while t < ParamLength(): ... t += 0.1` loop never evaluates the trajectory at its true end time, so a jerk violation confined to the last (<0.1s) sliver before the end can slip through undetected. Since a trajectory's duration is essentially never an exact multiple of the 0.1s sampling step, this isn't a rare edge case.

This is intentionally a minimal demo of the technique — extending the same continuous-vs-sampled approach to `IsValidLateralTrajectory` (which composes two polynomials and is higher-degree) is a natural next step.

### Notice
Notably, the goal of this project is not to build a functional planner in a distinct simulator to do experiments, but to learn the algorithms and data structures within it. Therefore, we are translating the C++ functions in the Lattice algorithm into their corresponding Python frameworks as much as possible. Due to the introduction of CyberRT, the analysis of the program has also caused confusion. We are consolidating the protobuf message objects used by this planner into Python dataclasses to clearly demonstrate how the planner manipulates data.
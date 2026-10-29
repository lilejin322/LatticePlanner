# Where this repository stops

This is a Python port of Apollo's lattice planner, kept so the planner itself can be tested: the polynomials, the costs, how a trajectory is assembled, and the constraint check that runs every 0.1 s. CommonRoad supplies an existing road and recorded vehicles.

It is not a traffic simulator and not a new way to drive. When the checker says the trajectory is valid, the motion can already be outside the bounds. The notes below are the facts that were found.

## What the checker misses

The backup braking curve answers "no jerk" at every sample. That answer is hard-coded as 0. When the vehicle finishes the stop, acceleration changes from a hard brake to 0, and that change sits between two samples. At a deceleration of 4 m/s² the jerk of that step is 40 m/s³. The limit is 2. The one-dimensional checker trusts the hard-coded answer, so it accepts the brake.

Another curve is checked only until it ends, here at 2 s. The planner still uses it out to 8 s by holding the final acceleration. Speed rises from 35 m/s to 59 m/s. The speed limit is 40 m/s. Nothing checks the piece after the curve ends.

One pinned curve has jerk −4.03 m/s³ at 1.84375 s. The limit is −4. The checker does not look at that instant. It compares acceleration at 1.8 s and at 1.9 s. After the curve ends, acceleration is frozen, so the difference between those two samples stays inside the limit. Both the one-dimensional checker and the Cartesian checker accept it. The lateral checker has the same kind of hole: lateral acceleration is −4.38 m/s² at 0.53125 s, against a limit of ±4, and the 0.1 s samples do not see it.

## What the planner returns

The inputs are ordinary. The returned trajectories are still wrong.

Speed 0 and a heading of 1 rad. The Frenet state is finite. The backup trajectory still returns a point about 2.1×10⁹ m away, at about 2143 m/s. The Cartesian checker rejects the speed, and the trajectory is still returned.

Heading turned 180 degrees, speed 1 m/s. Speed along the road is negative. The planner does not return a failed plan. It raises `AssertionError`.

A stopped car 8 m ahead. The backup trajectory is the brake above. The assembled samples change acceleration from −4 to 0 at 1.2 s, jerk 40 m/s³, and the trajectory is still returned.

Four recorded roads have their own signatures. On Peach the backup point is beyond 1×10⁵ m. On Monzon a negative speed along the road raises the same assertion. On ZAM, at 0.7 s, the backup trajectory's committed point jumps away from that cycle's start. On Moissy, at 0.9 s, speed falls from 14.13 m/s to 13.24 m/s in 0.1 s, which is −8.92 m/s², while the accelerations stored on that same trajectory stay inside the limit.

## Where the records are

To run them again:

```bash
python scripts/run_all_planner_checks.py
```

The checks pass while each signature is still present. When one is fixed, change that record to "contained". That item then fails until the record is updated.

| File | What it records |
|------|-----------------|
| `scripts/run_lattice_known_failures.py` | Four recorded roads |
| `scripts/run_lattice_synthetic_failures.py` | Three straight-road cases: stopped heading, reversed heading, backup jerk |
| `verification/pinned_sampling_gaps.py` | Two curves the 0.1 s samples miss |
| `verification/verify_backup_jerk.py` | Backup deceleration reports jerk 0 |
| `verification/verify_extrapolation.py` | Speed passes 40 m/s after the curve ends |
| `verification/verify_cartesian_jerk.py` | The 1.8 s to 1.9 s difference passes; the jerk between them does not |

Commands and numbers in more detail are in the repository `README.md`.

No further holes are being added. The kinds already repeat: samples are too sparse, the backup curve does not report jerk, and the trajectory keeps being extended after the curve ends.

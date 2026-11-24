# learned-tracking

**Joint-space trajectory tracking on a planar two-link arm: analytic motion
profiles vs. model-based control, benchmarked in MuJoCo — with a learned
residual controller as the next milestone.**

![benchmark](results/baseline.png)

Most robotics portfolios show a planner that works in the nominal case. This
repo asks the harder question: *which controller survives when the model is
wrong?* The answer changes with the failure mode, which is exactly why a
learned residual policy is worth training.

---

## What's inside

| Layer | Implementation |
|---|---|
| Motion profiles | The five C++ planners from my ROS 2 package [`trajectory_control`](https://github.com/): trapezoidal, cubic, quintic, 7-phase S-curve, multi-segment. Vendored and exposed to Python via pybind11. |
| Plant | MuJoCo planar 2-link arm, joint-torque control, inertial coupling, damping, friction, actuator limits, optional latency and sensor noise. |
| Controllers | `pd` (no model), `ff_pd` (inverse-dynamics feedforward + PD), `lqr` (time-varying LQR from MuJoCo linearizations). |
| Metrics | Position/velocity RMSE, max error, settling time, control effort, torque rate, jerk, real-time factor. |

The controller never sees the true plant: `Scenario.plant` is ground truth,
`Scenario.model` is the controller's belief. Model mismatch between the two is
the whole experiment.

```mermaid
graph TD
    A["PlannerSpec<br/>trapezoidal · cubic · quintic · S-curve"] --> B["C++ profile core<br/>pybind11"]
    B --> C["Reference<br/>q, qd, qdd on uniform grid"]
    C --> D["Controller<br/>pd · ff_pd · lqr"]
    E["PlantConfig (truth)<br/>payload · friction · latency"] --> F["MuJoCo 2-link arm"]
    G["PlantConfig (belief)<br/>nominal model"] --> D
    D -- "tau" --> F
    F -- "q, qd (noisy)" --> D
    F --> H["Metrics<br/>RMSE · settle · effort · jerk"]
    C --> H
    H --> I["results/baseline.md · .json · .png"]
```

---

## Results

Fixed gains for all conditions (tuned once on the nominal plant), 500 Hz
control, 2 ms MuJoCo timestep. `settle 0.000` means the error never exceeded
10 mrad.

| scenario | controller | RMSE [rad] | max err [rad] | vel RMSE [rad/s] | settle [s] | \|tau\| RMS [Nm] | d\|tau\|/dt RMS [Nm/s] | jerk RMS [rad/s^3] | RT factor |
|---|---|---|---|---|---|---|---|---|---|
| nominal | pd | 0.0044 | 0.0107 | 0.0280 | 0.418 | 0.506 | 3.7 | 38.0 | 200x |
| nominal | ff_pd | 0.0006 | 0.0010 | 0.0035 | 0.000 | 0.489 | 3.8 | 37.8 | 130x |
| nominal | lqr | 0.0005 | 0.0010 | 0.0038 | 0.000 | 0.489 | 3.7 | 38.2 | 37x |
| payload 2 kg | pd | 0.0290 | 0.0596 | 0.1728 | 1.142 | 3.134 | 20.9 | 38.8 | 211x |
| payload 2 kg | ff_pd | 0.0329 | 0.0685 | 0.1916 | 1.180 | 3.042 | 19.4 | 38.7 | 127x |
| payload 2 kg | lqr | 0.1131 | 0.3861 | 0.5766 | 1.218 | 2.048 | 74.6 | 692.9 | 39x |
| friction x25 | pd | 0.0111 | 0.0228 | 0.0403 | 0.894 | 1.012 | 5.0 | 42.5 | 211x |
| friction x25 | ff_pd | 0.0128 | 0.0209 | 0.0326 | 1.060 | 1.015 | 5.0 | 41.9 | 144x |
| friction x25 | lqr | 0.0412 | 0.0674 | 0.1231 | 1.218 | 0.979 | 21.2 | 611.4 | 41x |
| latency 10 ms | pd | 0.0336 | 0.0845 | 4.0255 | 1.218 | 10.858 | 2342.8 | 102573.3 | 178x |
| latency 10 ms | ff_pd | 0.0301 | 0.0723 | 3.9635 | 1.216 | 10.527 | 2086.2 | 97396.8 | 124x |
| latency 10 ms | lqr | 0.0016 | 0.0037 | 0.0125 | 0.000 | 0.499 | 20.8 | 296.7 | 39x |
| combined | pd | 0.0685 | 0.1684 | 1.7737 | 1.218 | 9.519 | 1674.4 | 45626.3 | 179x |
| combined | ff_pd | 0.0599 | 0.1549 | 1.6380 | 1.218 | 8.461 | 1366.8 | 41296.3 | 126x |
| combined | lqr | 0.1097 | 0.2812 | 0.4818 | 1.218 | 2.257 | 72.6 | 521.2 | 39x |

### What the numbers say

1. **With a correct model, feedforward is everything.** `ff_pd` and `lqr` are
   ~7× better than plain PD (6e-4 vs 4.4e-3 rad) at essentially the same
   effort. Classical computed-torque control is hard to beat when you know
   the plant.
2. **An unmodelled 2 kg payload erases that advantage.** `ff_pd` goes from
   6e-4 to 3.3e-2 rad — a 55× degradation — landing at the same error as PD.
   The feedforward term is confidently wrong.
3. **LQR is the most brittle to model error** (0.11 rad under payload): it
   trusts the model twice, in the feedforward *and* in the feedback gains, and
   in the combined case it visibly diverges at the end of the move.
4. **Delay flips the ranking.** With 10 ms of latency, `lqr` is ~19× better
   than `ff_pd` (1.6e-3 vs 3.0e-2 rad) because it is a closed-loop design;
   PD and FF+PD chatter against the delay, saturating the ±12 Nm actuators for
   most of the move.
5. **No controller wins everywhere.** That gap — model-based precision when
   the model is right, feedback robustness when it isn't — is the target for a
   learned residual policy.

---

## Install

```bash
git clone <this-repo> && cd learned-tracking
uv venv --python 3.12 && source .venv/bin/activate
uv pip install -e . --no-build-isolation   # builds the C++ core
```

## Run

```bash
# full grid: 5 conditions x 3 controllers -> results/
python experiments/baseline.py --plot

# one scenario
lt bench --controller lqr --scenario payload --payload 2.0 --json out.json

# tests
pytest
```

## Layout

```
cpp/                    C++ planners (from the ROS 2 trajectory_control package) + pybind11
src/learned_tracking/
    core.py             planner specs -> sampled multi-joint reference
    plant.py            MuJoCo 2-link arm, inverse dynamics, linearization
    controllers.py      pd, ff_pd, lqr
    metrics.py          tracking metrics
    experiment.py       rollout loop, markdown table
    cli.py              `lt bench`
experiments/baseline.py condition x controller grid, plots
tests/                  planners, plant, experiment smoke tests
results/                committed table, JSON and figure
```

## Roadmap

- **v0.2 — learned residual (next).** PPO policy that outputs a torque
  correction on top of `ff_pd`, trained with domain randomization over payload,
  friction and latency; evaluated on held-out conditions. Success criterion:
  match `ff_pd` on nominal *and* `lqr` on delay, without a per-condition model.
- **v0.3 — stronger baselines.** Gain-scheduled PD, online parameter
  adaptation, iLQR with online model estimation.
- **v1.0 — hardware.** Same interface on a real 2-DOF testbed or SO-101 arm,
  with the ROS 2 node from `trajectory_control` as the command source.

## License

MIT — see [LICENSE](LICENSE).

"""Baseline benchmark: analytic profiles x controllers x plant mismatches.

Run with:

    python experiments/baseline.py            # table + JSON + plots
    python experiments/baseline.py --plot     # also open the figure

Outputs land in ``results/``:

    baseline.json   – every rollout with its metrics
    baseline.md     – markdown table (pasted into the README)
    baseline.png    – error traces + RMSE bars
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from learned_tracking import PlannerSpec, Scenario, run, to_markdown  # noqa: E402
from learned_tracking.plant import PlantConfig  # noqa: E402

# Aggressive but feasible reference: peak velocity 3 rad/s, peak accel 8 rad/s^2.
PLANNERS = (
    PlannerSpec(kind="s_curve", q0=0.0, qf=1.2, v_max=3.0, a_max=8.0, j_max=60.0),
    PlannerSpec(kind="s_curve", q0=0.0, qf=-0.9, v_max=2.5, a_max=7.0, j_max=55.0),
)

CONDITIONS: dict[str, PlantConfig] = {
    "nominal": PlantConfig(),
    "payload 2 kg": PlantConfig(payload=2.0),
    "friction x25": PlantConfig(frictionloss=0.5, damping=0.4),
    "latency 10 ms": PlantConfig(latency_steps=5, noise_std=0.001),
    "combined": PlantConfig(
        payload=2.0, frictionloss=0.5, damping=0.4, latency_steps=5, noise_std=0.001
    ),
}

CONTROLLERS = ["pd", "ff_pd", "lqr"]


def build_scenarios() -> list[Scenario]:
    return [
        Scenario(name=cond, plant=plant, planners=PLANNERS, controller=ctrl)
        for cond, plant in CONDITIONS.items()
        for ctrl in CONTROLLERS
    ]


def plot(results, out: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.2), constrained_layout=True)

    combined = [r for r in results if r.name == "combined"]
    for r in combined:
        err = np.linalg.norm(r.q - r.reference.q, axis=0)
        ax1.plot(r.t, err, label=r.scenario.controller, lw=1.6)
    ax1.set_xlabel("time [s]")
    ax1.set_ylabel("‖q − q_ref‖ [rad]")
    ax1.set_title("Tracking error, combined mismatch")
    ax1.grid(alpha=0.3)
    ax1.legend()

    conds = list(CONDITIONS)
    width = 0.8 / len(CONTROLLERS)
    x = np.arange(len(conds))
    for i, ctrl in enumerate(CONTROLLERS):
        vals = [next(r.metrics["rmse"] for r in results if r.name == c and r.scenario.controller == ctrl) for c in conds]
        ax2.bar(x + i * width, vals, width, label=ctrl)
    ax2.set_yscale("log")
    ax2.set_xticks(x + width * (len(CONTROLLERS) - 1) / 2, conds, rotation=15)
    ax2.set_ylabel("position RMSE [rad]")
    ax2.set_title("RMSE by condition (log scale)")
    ax2.grid(alpha=0.3, axis="y")
    ax2.legend()

    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plot", action="store_true", help="render results/baseline.png")
    args = parser.parse_args()

    out_dir = ROOT / "results"
    out_dir.mkdir(exist_ok=True)

    results = [run(s) for s in build_scenarios()]
    table = to_markdown(results)
    print(table)

    (out_dir / "baseline.md").write_text(table + "\n")
    payload = [
        {
            "scenario": r.name,
            "controller": r.scenario.controller,
            "metrics": r.metrics,
            "plant": r.scenario.plant.__dict__,
        }
        for r in results
    ]
    (out_dir / "baseline.json").write_text(json.dumps(payload, indent=2))
    print(f"\nwrote {out_dir / 'baseline.md'} and baseline.json")

    if args.plot:
        plot(results, out_dir / "baseline.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

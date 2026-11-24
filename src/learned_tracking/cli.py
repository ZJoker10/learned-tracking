"""Command line entry point: ``lt bench``."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .controllers import CONTROLLERS
from .core import PlannerSpec
from .experiment import Scenario, run, to_markdown
from .plant import PlantConfig


def _scenario(args: argparse.Namespace) -> Scenario:
    planners = (
        PlannerSpec(kind=args.planner, q0=0.0, qf=1.2, v_max=3.0, a_max=8.0, j_max=60.0),
        PlannerSpec(kind=args.planner, q0=0.0, qf=-0.9, v_max=2.5, a_max=7.0, j_max=55.0),
    )
    plant = PlantConfig(
        payload=args.payload,
        frictionloss=args.frictionloss,
        latency_steps=args.latency_steps,
        noise_std=args.noise,
    )
    return Scenario(
        name=args.scenario,
        plant=plant,
        planners=planners,
        controller=args.controller,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="lt", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    bench = sub.add_parser("bench", help="run one scenario")
    bench.add_argument("--scenario", default="nominal")
    bench.add_argument("--planner", default="s_curve", choices=["trapezoidal", "cubic", "quintic", "s_curve", "multi_segment"])
    bench.add_argument("--controller", default="ff_pd", choices=sorted(CONTROLLERS))
    bench.add_argument("--payload", type=float, default=0.0, help="unmodelled tip mass [kg]")
    bench.add_argument("--frictionloss", type=float, default=0.02)
    bench.add_argument("--latency-steps", type=int, default=0)
    bench.add_argument("--noise", type=float, default=0.0, help="joint position noise std [rad]")
    bench.add_argument("--json", type=Path, help="write metrics to this file")

    args = parser.parse_args(argv)
    if args.command == "bench":
        result = run(_scenario(args))
        print(to_markdown([result]))
        if args.json:
            args.json.write_text(json.dumps(result.metrics, indent=2))
            print(f"wrote {args.json}")
        return 0
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Analytic, optimal-control and learned joint-space trajectory tracking.

The analytic planner core is the C++ implementation from the ROS 2 package
``trajectory_control`` (trapezoidal, cubic, quintic, S-curve, multi-segment),
vendored here so the benchmark installs without a ROS toolchain.
"""

from __future__ import annotations

from .core import PlannerSpec, Reference, make_planner, make_reference
from .experiment import Scenario, RunResult, run, run_sweep, to_markdown
from .plant import PlanarArm, PlantConfig

__all__ = [
    "PlannerSpec",
    "Reference",
    "make_planner",
    "make_reference",
    "PlantConfig",
    "PlanarArm",
    "Scenario",
    "RunResult",
    "run",
    "run_sweep",
    "to_markdown",
]

__version__ = "0.1.0"

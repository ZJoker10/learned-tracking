"""Experiment loop: roll out a controller against the reference on the plant."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

from .controllers import build_controller
from .core import PlannerSpec, Reference, make_reference
from .metrics import tracking_metrics
from .plant import PlanarArm, PlantConfig

DEFAULT_PLANNERS: tuple[PlannerSpec, ...] = (
    PlannerSpec(kind="s_curve", q0=0.0, qf=1.2, v_max=1.5, a_max=4.0, j_max=40.0),
    PlannerSpec(kind="s_curve", q0=0.0, qf=-0.9, v_max=1.2, a_max=3.5, j_max=35.0),
)


@dataclass(frozen=True)
class Scenario:
    """One (plant, reference, controller) combination.

    ``plant`` is ground truth. ``model`` is the controller's belief about the
    plant; ``None`` means the nominal arm (no payload, light friction). Model
    mismatch between the two is what separates the controllers.
    """

    name: str = "nominal"
    plant: PlantConfig = field(default_factory=PlantConfig)
    model: PlantConfig | None = None
    planners: Sequence[PlannerSpec] = DEFAULT_PLANNERS
    controller: str = "ff_pd"
    hold_after: float = 0.3
    controller_kwargs: Mapping[str, Any] = field(default_factory=dict)


@dataclass
class RunResult:
    scenario: Scenario
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    tau: np.ndarray
    reference: Reference
    wall_time: float
    metrics: dict[str, float]

    @property
    def name(self) -> str:
        return self.scenario.name

    def to_row(self) -> dict[str, Any]:
        return {"scenario": self.name, "controller": self.scenario.controller, **self.metrics}


def run(scenario: Scenario) -> RunResult:
    """Roll out one scenario and return the recorded signals."""
    dt = scenario.plant.timestep
    ref = make_reference(scenario.planners, dt=dt, hold_after=scenario.hold_after)

    arm = PlanarArm(scenario.plant)
    arm.reset(ref.q[:, 0], ref.qd[:, 0])
    model = PlanarArm(scenario.model or PlantConfig())

    ctrl = build_controller(scenario.controller, **dict(scenario.controller_kwargs))
    t0 = time.perf_counter()
    ctrl.reset(ref, model)
    reset_time = time.perf_counter() - t0

    n = ref.n_steps
    t = np.arange(n) * dt
    q = np.zeros((ref.n_joints, n))
    qd = np.zeros_like(q)
    tau = np.zeros_like(q)

    limit = scenario.plant.torque_limit
    loop_start = time.perf_counter()
    for k in range(n):
        q_k, qd_k = arm.measure()  # noisy: what the controller sees
        q[:, k], qd[:, k] = arm.q, arm.qd  # ground truth: what we score
        cmd = ctrl(k, q_k, qd_k, ref, model)
        tau[:, k] = np.clip(cmd, -limit, limit)
        if k < n - 1:
            arm.step(cmd)
    wall_time = (time.perf_counter() - loop_start) + reset_time

    metrics = tracking_metrics(
        t=t,
        q=q,
        qd=qd,
        tau=tau,
        q_ref=ref.q,
        qd_ref=ref.qd,
        wall_time=wall_time,
    )
    metrics["setup_time"] = reset_time
    return RunResult(scenario, t, q, qd, tau, ref, wall_time, metrics)


def run_sweep(scenarios: Iterable[Scenario]) -> list[RunResult]:
    return [run(s) for s in scenarios]


_COLUMNS = [
    ("rmse", "RMSE [rad]", "{:.4f}"),
    ("max_err", "max err [rad]", "{:.4f}"),
    ("vel_rmse", "vel RMSE [rad/s]", "{:.4f}"),
    ("settle_time", "settle [s]", "{:.3f}"),
    ("tau_rms", "|tau| RMS [Nm]", "{:.3f}"),
    ("tau_rate_rms", "d|tau|/dt RMS [Nm/s]", "{:.1f}"),
    ("jerk_rms", "jerk RMS [rad/s^3]", "{:.1f}"),
    ("realtime_factor", "RT factor", "{:.0f}x"),
]


def to_markdown(results: Sequence[RunResult]) -> str:
    """Render results as a markdown table (used by the README and the CLI)."""
    header = ["scenario", "controller"] + [label for _, label, _ in _COLUMNS]
    lines = [
        "| " + " | ".join(header) + " |",
        "|" + "|".join(["---"] * len(header)) + "|",
    ]
    for r in results:
        cells = [r.name, r.scenario.controller]
        cells += [fmt.format(r.metrics[key]) for key, _, fmt in _COLUMNS]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)

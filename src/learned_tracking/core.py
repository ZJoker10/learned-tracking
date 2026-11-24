"""Planner specifications and reference-trajectory construction.

Thin Python layer over the C++ profile implementations (``learned_tracking._core``).
Everything here returns numpy arrays on a uniform control grid so the
controllers and the MuJoCo plant can consume it without per-step FFI calls.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Sequence

import numpy as np

from . import _core

PlannerKind = Literal["trapezoidal", "cubic", "quintic", "s_curve", "multi_segment"]


@dataclass(frozen=True)
class PlannerSpec:
    """Parameters shared by every planner kind.

    ``T`` is only used by the polynomial planners; when omitted a duration is
    derived from ``v_max`` (1.875 * distance / v_max, the minimum-jerk rule of
    thumb). ``waypoints`` is only used by ``multi_segment``.
    """

    kind: PlannerKind = "s_curve"
    q0: float = 0.0
    qf: float = 1.0
    v_max: float = 1.5
    a_max: float = 4.0
    j_max: float = 40.0
    T: float | None = None
    waypoints: tuple[float, ...] = ()


def _poly_duration(spec: PlannerSpec) -> float:
    if spec.T is not None:
        return float(spec.T)
    dist = abs(spec.qf - spec.q0)
    if dist < 1e-9:
        return 1e-3
    return 1.875 * dist / max(spec.v_max, 1e-9)


def make_planner(spec: PlannerSpec) -> _core.Planner:
    """Instantiate the requested C++ profile."""
    if spec.kind == "trapezoidal":
        return _core.trapezoidal(spec.q0, spec.qf, spec.v_max, spec.a_max)
    if spec.kind == "cubic":
        return _core.cubic(spec.q0, spec.qf, 0.0, 0.0, _poly_duration(spec))
    if spec.kind == "quintic":
        return _core.quintic(spec.q0, spec.qf, 0.0, 0.0, 0.0, 0.0, _poly_duration(spec))
    if spec.kind == "s_curve":
        return _core.s_curve(spec.q0, spec.qf, spec.v_max, spec.a_max, spec.j_max)
    if spec.kind == "multi_segment":
        pts = (spec.q0, *spec.waypoints, spec.qf)
        plan = _core.SegmentPlan()
        plan.reserve(len(pts) - 1)
        for a, b in zip(pts[:-1], pts[1:]):
            plan.add(_core.s_curve(a, b, spec.v_max, spec.a_max, spec.j_max))
        return plan  # type: ignore[return-value]
    raise ValueError(f"unknown planner kind: {spec.kind!r}")


class Reference:
    """Multi-joint reference sampled on a uniform grid.

    ``q``, ``qd``, ``qdd`` and ``jerk`` are shaped ``(n_joints, n_steps)``.
    Joints that finish early are held at their final position.
    """

    def __init__(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray, jerk: np.ndarray, dt: float):
        self.q = q
        self.qd = qd
        self.qdd = qdd
        self.jerk = jerk
        self.dt = dt
        self.n_joints, self.n_steps = q.shape

    @property
    def duration(self) -> float:
        return (self.n_steps - 1) * self.dt

    def state(self, k: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        k = min(max(k, 0), self.n_steps - 1)
        return self.q[:, k], self.qd[:, k], self.qdd[:, k]

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Reference(joints={self.n_joints}, steps={self.n_steps}, dt={self.dt})"


def make_reference(
    specs: Sequence[PlannerSpec], dt: float = 0.002, hold_after: float = 0.3
) -> Reference:
    """Build one synchronized reference from a per-joint planner spec."""
    if not specs:
        raise ValueError("need at least one joint spec")

    plans = [make_planner(s) for s in specs]
    durations = [p.duration() for p in plans]
    total = max(durations) + hold_after
    n_steps = int(np.floor(total / dt)) + 1

    t = np.arange(n_steps) * dt
    q = np.zeros((len(plans), n_steps))
    qd = np.zeros_like(q)
    qdd = np.zeros_like(q)
    jerk = np.zeros_like(q)

    for i, (plan, dur) in enumerate(zip(plans, durations)):
        states = plan.sample(0.0, dur, max(int(np.ceil(dur / dt)) + 1, 2))
        ts = np.array([s.time for s in states])
        qs = np.array([s.position for s in states])
        vds = np.array([s.velocity for s in states])
        qdds = np.array([s.acceleration for s in states])
        js = np.array([s.jerk for s in states])
        # np.interp clamps outside the sampled range, which is exactly the
        # "hold at the goal" behaviour we want after the joint settles.
        q[i] = np.interp(t, ts, qs)
        qd[i] = np.interp(t, ts, vds)
        qdd[i] = np.interp(t, ts, qdds)
        jerk[i] = np.interp(t, ts, js)

    return Reference(q, qd, qdd, jerk, dt)


def make_trajectory(spec: PlannerSpec, dt: float = 0.002) -> _core.Planner:
    """Return the raw C++ planner (for plotting exact profiles)."""
    return make_planner(spec)

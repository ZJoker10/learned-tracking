"""Tracking metrics for one benchmark rollout."""

from __future__ import annotations

import numpy as np


def _rms(x: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(x))))


def tracking_metrics(
    t: np.ndarray,
    q: np.ndarray,
    qd: np.ndarray,
    tau: np.ndarray,
    q_ref: np.ndarray,
    qd_ref: np.ndarray,
    wall_time: float,
    settle_tol: float = 0.01,
) -> dict[str, float]:
    """Summarize a rollout.

    Parameters are shaped ``(n_joints, n_steps)`` except ``t``/``wall_time``.

    Reported quantities:
      * ``rmse``            – position tracking error RMS [rad]
      * ``max_err``         – worst absolute position error [rad]
      * ``vel_rmse``        – velocity tracking error RMS [rad/s]
      * ``settle_time``     – last time any joint exceeds ``settle_tol`` [s]
      * ``jerk_rms``        – RMS of d(acceleration)/dt from measured motion
      * ``tau_rms``         – control effort RMS [Nm]
      * ``tau_peak``        – largest absolute torque [Nm]
      * ``realtime_factor`` – simulated time / wall time
    """
    err = q - q_ref
    vel_err = qd - qd_ref
    dt = float(t[1] - t[0]) if t.size > 1 else 1.0

    acc = np.gradient(qd, dt, axis=1)
    jerk = np.gradient(acc, dt, axis=1)
    tau_rate = np.diff(tau, axis=1) / dt

    over = np.abs(err) > settle_tol
    settle_idx = int(np.max(np.nonzero(over)[1])) if over.any() else 0
    settle_time = float(t[settle_idx]) if t.size > settle_idx else 0.0

    return {
        "rmse": _rms(err),
        "max_err": float(np.max(np.abs(err))),
        "vel_rmse": _rms(vel_err),
        "settle_time": settle_time,
        "jerk_rms": _rms(jerk),
        "tau_rms": _rms(tau),
        "tau_rate_rms": _rms(tau_rate),
        "tau_peak": float(np.max(np.abs(tau))),
        "realtime_factor": float(t[-1] / wall_time) if wall_time > 0 else float("inf"),
    }

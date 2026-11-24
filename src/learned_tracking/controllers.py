"""Controllers compared in the benchmark.

All controllers share one call signature so the experiment loop stays trivial:

    ctrl.reset(ref, arm)                 # optional precomputation
    tau = ctrl(k, q, qd, ref, arm)       # torque for control step k

Included:

* ``pd``     – joint-space PD on position error (no model knowledge).
* ``ff_pd``  – inverse-dynamics feedforward + PD (classic computed torque).
* ``lqr``    – time-varying LQR along the reference (offline Riccati solve,
               online state feedback). This is the linear-quadratic baseline
               that a learned policy has to beat to be interesting.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from .core import Reference
from .plant import PlanarArm


class Controller(Protocol):
    name: str

    def reset(self, ref: Reference, arm: PlanarArm) -> None: ...

    def __call__(
        self, k: int, q: np.ndarray, qd: np.ndarray, ref: Reference, arm: PlanarArm
    ) -> np.ndarray: ...


@dataclass
class PDController:
    kp: float = 80.0
    kd: float = 12.0
    name: str = "pd"

    def reset(self, ref: Reference, arm: PlanarArm) -> None:
        return None

    def __call__(self, k, q, qd, ref, arm) -> np.ndarray:
        q_d, qd_d, _ = ref.state(k)
        return self.kp * (q_d - q) + self.kd * (qd_d - qd)


@dataclass
class FeedforwardPD:
    """tau = ID(q_d, qd_d, qdd_d) + kp e + kd edot."""

    kp: float = 60.0
    kd: float = 9.0
    name: str = "ff_pd"

    def reset(self, ref: Reference, arm: PlanarArm) -> None:
        return None

    def __call__(self, k, q, qd, ref, arm) -> np.ndarray:
        q_d, qd_d, qdd_d = ref.state(k)
        tau_ff = arm.inverse_dynamics(q_d, qd_d, qdd_d)
        return tau_ff + self.kp * (q_d - q) + self.kd * (qd_d - qd)


@dataclass
class TrackingLQR:
    """Time-varying LQR tracking the reference.

    Linearizes the MuJoCo plant along the reference trajectory, solves the
    backward Riccati recursion once, then applies ``u = u_ref - K (x - x_ref)``.
    """

    q_weight: float = 400.0
    qd_weight: float = 4.0
    r_weight: float = 1.0
    name: str = "lqr"

    _K: np.ndarray | None = None
    _u_ref: np.ndarray | None = None
    _x_ref: np.ndarray | None = None

    def reset(self, ref: Reference, arm: PlanarArm) -> None:
        n = ref.n_joints
        N = ref.n_steps
        nu = n

        x_ref = np.zeros((2 * n, N))
        u_ref = np.zeros((nu, N))
        for k in range(N):
            q_d, qd_d, qdd_d = ref.state(k)
            x_ref[:, k] = np.concatenate([q_d, qd_d])
            u_ref[:, k] = arm.inverse_dynamics(q_d, qd_d, qdd_d)

        Q = np.diag(np.concatenate([np.full(n, self.q_weight), np.full(n, self.qd_weight)]))
        Qf = Q * 10.0
        R = np.eye(nu) * self.r_weight

        K = np.zeros((nu, 2 * n, N))
        P = Qf
        for k in range(N - 2, -1, -1):
            A, B = arm.linearize(x_ref[:n, k], x_ref[n:, k], u_ref[:, k])
            BtP = B.T @ P
            K[:, :, k] = np.linalg.solve(R + BtP @ B, BtP @ A)
            P = Q + A.T @ P @ (A - B @ K[:, :, k])

        self._K, self._u_ref, self._x_ref = K, u_ref, x_ref

    def __call__(self, k, q, qd, ref, arm) -> np.ndarray:
        assert self._K is not None and self._u_ref is not None and self._x_ref is not None
        k = min(k, self._u_ref.shape[1] - 1)
        x = np.concatenate([q, qd])
        return self._u_ref[:, k] - self._K[:, :, k] @ (x - self._x_ref[:, k])


CONTROLLERS = {
    "pd": PDController,
    "ff_pd": FeedforwardPD,
    "lqr": TrackingLQR,
}


def build_controller(name: str, **kwargs) -> Controller:
    try:
        cls = CONTROLLERS[name]
    except KeyError:
        raise ValueError(f"unknown controller {name!r}; choose from {sorted(CONTROLLERS)}") from None
    return cls(**kwargs)  # type: ignore[return-value]

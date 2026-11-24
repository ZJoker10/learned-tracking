"""MuJoCo plant: a planar two-link arm under joint-torque control.

The model is deliberately simple (no contact, no gravity in the plane) but it
still has the properties that make trajectory tracking non-trivial: inertial
coupling between joints, Coriolis terms, passive damping/friction, actuator
limits, and optional measurement noise and control latency.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass(frozen=True)
class PlantConfig:
    """Physical and sensing parameters of the arm."""

    l1: float = 0.35
    l2: float = 0.30
    radius: float = 0.02
    density: float = 1200.0
    payload: float = 0.0
    damping: float = 0.05
    armature: float = 0.01
    frictionloss: float = 0.02
    torque_limit: float = 12.0
    gravity: float = 0.0
    timestep: float = 0.002
    noise_std: float = 0.0
    latency_steps: int = 0
    seed: int = 0

    def xml(self) -> str:
        payload_mass = max(self.payload, 1e-6)
        return f"""
<mujoco model="planar2r">
  <compiler angle="radian"/>
  <option timestep="{self.timestep}" integrator="implicitfast" gravity="0 0 {self.gravity}"/>

  <default>
    <joint type="hinge" axis="0 0 1" damping="{self.damping}"
           armature="{self.armature}" frictionloss="{self.frictionloss}"
           range="-3.14 3.14" limited="true"/>
    <geom type="capsule" size="{self.radius}" density="{self.density}"
          contype="0" conaffinity="0"/>
    <motor ctrllimited="true" ctrlrange="-{self.torque_limit} {self.torque_limit}"/>
  </default>

  <worldbody>
    <body name="link1" pos="0 0 0.05">
      <joint name="j1"/>
      <geom fromto="0 0 0 {self.l1} 0 0"/>
      <body name="link2" pos="{self.l1} 0 0">
        <joint name="j2"/>
        <geom fromto="0 0 0 {self.l2} 0 0"/>
        <body name="tip" pos="{self.l2} 0 0">
          <geom name="payload" type="sphere" size="0.03" mass="{payload_mass}"/>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor joint="j1"/>
    <motor joint="j2"/>
  </actuator>
</mujoco>
"""


class PlanarArm:
    """Joint-torque interface to the MuJoCo model."""

    def __init__(self, config: PlantConfig | None = None):
        self.config = config or PlantConfig()
        self.model = mujoco.MjModel.from_xml_string(self.config.xml())
        self.data = mujoco.MjData(self.model)
        self.n_joints = self.model.nv
        self.dt = self.config.timestep
        self.time = 0.0
        self._rng = np.random.default_rng(self.config.seed)
        self._queue: list[np.ndarray] = [
            np.zeros(self.n_joints) for _ in range(self.config.latency_steps)
        ]

    # ------------------------------------------------------------------ state
    def reset(self, q0: np.ndarray | None = None, qd0: np.ndarray | None = None) -> None:
        q0 = np.zeros(self.n_joints) if q0 is None else np.asarray(q0, dtype=float)
        qd0 = np.zeros(self.n_joints) if qd0 is None else np.asarray(qd0, dtype=float)
        self.data.qpos[:] = q0
        self.data.qvel[:] = qd0
        self.data.ctrl[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.time = 0.0
        self._queue = [np.zeros(self.n_joints) for _ in range(self.config.latency_steps)]

    @property
    def q(self) -> np.ndarray:
        return np.array(self.data.qpos, copy=True)

    @property
    def qd(self) -> np.ndarray:
        return np.array(self.data.qvel, copy=True)

    def measure(self) -> tuple[np.ndarray, np.ndarray]:
        """Noisy joint positions/velocities as seen by the controller."""
        q, qd = self.q, self.qd
        if self.config.noise_std > 0.0:
            q = q + self._rng.normal(0.0, self.config.noise_std, size=q.shape)
            qd = qd + self._rng.normal(0.0, self.config.noise_std * 10.0, size=qd.shape)
        return q, qd

    # ------------------------------------------------------------------- step
    def step(self, torque: np.ndarray) -> None:
        torque = np.asarray(torque, dtype=float).clip(
            -self.config.torque_limit, self.config.torque_limit
        )
        if self.config.latency_steps > 0:
            self._queue.append(torque)
            applied = self._queue.pop(0)
        else:
            applied = torque
        self.data.ctrl[:] = applied
        mujoco.mj_step(self.model, self.data)
        self.time += self.dt

    # --------------------------------------------------------- model helpers
    def inverse_dynamics(
        self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray
    ) -> np.ndarray:
        """Generalized force required to realize (q, qd, qdd).

        Uses ``mj_inverse`` so passive forces (damping, friction) are included,
        which is what a model-based feedforward term needs.
        """
        d = self.data
        saved = (d.qpos.copy(), d.qvel.copy(), d.qacc.copy(), d.qfrc_applied.copy())
        try:
            d.qpos[:] = q
            d.qvel[:] = qd
            d.qacc[:] = qdd
            d.qfrc_applied[:] = 0.0
            mujoco.mj_inverse(self.model, d)
            return np.array(d.qfrc_inverse, copy=True)
        finally:
            d.qpos[:], d.qvel[:], d.qacc[:], d.qfrc_applied[:] = saved

    def linearize(
        self, q: np.ndarray, qd: np.ndarray, u: np.ndarray, eps: float = 1e-6
    ) -> tuple[np.ndarray, np.ndarray]:
        """Discrete-time linearization ``x+ = A x + B u`` around (q, qd, u)."""
        d = self.data
        saved = (d.qpos.copy(), d.qvel.copy(), d.ctrl.copy())
        try:
            d.qpos[:] = q
            d.qvel[:] = qd
            d.ctrl[:] = u
            mujoco.mj_forward(self.model, d)
            n = 2 * self.model.nv
            A = np.zeros((n, n))
            B = np.zeros((n, self.model.nu))
            mujoco.mjd_transitionFD(self.model, d, eps, True, A, B, None, None)
            return A, B
        finally:
            d.qpos[:], d.qvel[:], d.ctrl[:] = saved

    def forward_kinematics(self, q: np.ndarray) -> np.ndarray:
        """Tip position in the plane, for plots and videos."""
        l1, l2 = self.config.l1, self.config.l2
        x = l1 * np.cos(q[0]) + l2 * np.cos(q[0] + q[1])
        y = l1 * np.sin(q[0]) + l2 * np.sin(q[0] + q[1])
        return np.array([x, y])

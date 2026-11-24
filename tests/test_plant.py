import numpy as np
import pytest

from learned_tracking.plant import PlanarArm, PlantConfig


def test_torque_accelerates_the_arm():
    arm = PlanarArm(PlantConfig())
    arm.reset()
    for _ in range(50):
        arm.step(np.array([2.0, 0.0]))
    assert arm.q[0] > 0.0
    assert np.abs(arm.q[1]) > 0.0  # inertial coupling


def test_joint_limits_are_enforced():
    arm = PlanarArm(PlantConfig(torque_limit=40.0))
    arm.reset()
    for _ in range(3000):
        arm.step(np.array([40.0, -40.0]))
    # MuJoCo limits are soft constraints, so a small overshoot is expected.
    assert np.all(np.abs(arm.q) <= 3.14 + 0.1)
    assert np.all(np.isfinite(arm.q))


def test_inverse_dynamics_reproduces_acceleration():
    """With no damping/friction/gravity, applying ID torque must yield qdd."""
    cfg = PlantConfig(damping=0.0, frictionloss=0.0, armature=0.0)
    arm = PlanarArm(cfg)
    arm.reset()
    q = np.array([0.3, -0.5])
    qd = np.array([0.4, 0.1])
    qdd = np.array([1.2, -2.0])

    tau = arm.inverse_dynamics(q, qd, qdd)

    arm.reset(q, qd)
    arm.step(tau)
    # semi-implicit Euler: qvel_new = qvel + qacc * dt
    np.testing.assert_allclose(arm.qd, qd + qdd * cfg.timestep, atol=2e-2)


def test_measurement_noise_is_seeded():
    cfg = PlantConfig(noise_std=0.01, seed=7)
    a1, a2 = PlanarArm(cfg), PlanarArm(cfg)
    a1.reset()
    a2.reset()
    np.testing.assert_allclose(a1.measure()[0], a2.measure()[0])


def test_linearization_shapes():
    arm = PlanarArm()
    arm.reset()
    A, B = arm.linearize(np.zeros(2), np.zeros(2), np.zeros(2))
    assert A.shape == (4, 4)
    assert B.shape == (4, 2)
    assert np.all(np.isfinite(A)) and np.all(np.isfinite(B))

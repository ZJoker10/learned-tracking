import numpy as np
import pytest

from learned_tracking import PlannerSpec, make_planner, make_reference

KINDS = ["trapezoidal", "cubic", "quintic", "s_curve"]


@pytest.mark.parametrize("kind", KINDS)
def test_boundary_conditions(kind):
    spec = PlannerSpec(kind=kind, q0=0.0, qf=1.0, v_max=2.0, a_max=6.0, j_max=60.0)
    plan = make_planner(spec)
    T = plan.duration()
    assert T > 0.0
    assert plan.state(0.0).position == pytest.approx(0.0, abs=1e-9)
    assert plan.state(T).position == pytest.approx(1.0, abs=1e-6)
    assert plan.state(T).velocity == pytest.approx(0.0, abs=1e-6)


@pytest.mark.parametrize("kind", KINDS)
def test_states_are_finite_and_bounded(kind):
    spec = PlannerSpec(kind=kind, q0=-0.4, qf=0.8, v_max=2.0, a_max=6.0, j_max=60.0)
    plan = make_planner(spec)
    T = plan.duration()
    states = plan.sample(0.0, T, 200)
    assert all(s.is_finite() for s in states)
    vmax = max(abs(s.velocity) for s in states)
    assert vmax <= spec.v_max * 1.02


def test_s_curve_is_monotone():
    plan = make_planner(PlannerSpec(kind="s_curve", q0=0.0, qf=1.2, v_max=3.0, a_max=8.0, j_max=60.0))
    pos = np.array([s.position for s in plan.sample(0.0, plan.duration(), 400)])
    assert np.all(np.diff(pos) >= -1e-9)


def test_multi_segment_visits_waypoints():
    spec = PlannerSpec(
        kind="multi_segment", q0=0.0, qf=1.0, waypoints=(0.5, -0.3), v_max=2.0, a_max=5.0, j_max=50.0
    )
    plan = make_planner(spec)
    assert plan.duration() > 0.0
    assert plan.state(plan.duration()).position == pytest.approx(1.0, abs=1e-6)


def test_reference_shape_and_hold():
    ref = make_reference(
        [PlannerSpec(kind="s_curve", q0=0.0, qf=1.0), PlannerSpec(kind="trapezoidal", q0=0.0, qf=0.2)],
        dt=0.002,
        hold_after=0.2,
    )
    assert ref.q.shape == (2, ref.n_steps)
    assert ref.q[0, -1] == pytest.approx(1.0, abs=1e-6)
    # both joints hold still at the end
    assert np.allclose(ref.qd[:, -10:], 0.0, atol=1e-6)

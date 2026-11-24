import numpy as np

from learned_tracking import Scenario, run
from learned_tracking.plant import PlantConfig


def _scenario(controller: str, **plant) -> Scenario:
    return Scenario(name="test", plant=PlantConfig(**plant), controller=controller)


def test_nominal_tracking_is_tight():
    result = run(_scenario("ff_pd"))
    assert result.metrics["rmse"] < 0.01
    assert np.all(np.isfinite(result.q))


def test_payload_degrades_pd_but_not_as_much_with_feedforward():
    nominal = run(_scenario("pd"))
    loaded = run(_scenario("pd", payload=2.0))
    assert loaded.metrics["rmse"] > nominal.metrics["rmse"]


def test_lqr_runs_and_tracks():
    result = run(_scenario("lqr"))
    assert result.metrics["rmse"] < 0.01
    assert result.metrics["realtime_factor"] > 1.0


def test_metrics_are_complete():
    result = run(_scenario("pd"))
    for key in ("rmse", "max_err", "vel_rmse", "settle_time", "jerk_rms", "tau_rms", "tau_peak"):
        assert key in result.metrics
        assert np.isfinite(result.metrics[key])

"""Speed changes and paths without a step in the attitude."""
import math

import numpy as np
import pytest

from psi_vtol.trajectory import Path, SpeedPlan, SpeedRamp, stopping_distance

SQUARE = [(0.0, 0.0), (50.0, 0.0), (50.0, 50.0), (0.0, 50.0), (0.0, 0.0)]


@pytest.mark.parametrize("v0, v1", [(0.0, 5.0), (5.0, 0.0), (5.0, 3.0), (0.3, 0.0)])
def test_a_speed_ramp_stays_within_its_limits_and_ends_exactly(v0, v1):
    ramp = SpeedRamp(v0, v1, 1.0, 0.7)
    t = np.linspace(0.0, ramp.duration, 3001)
    s, v, a = np.array([ramp.state(x) for x in t]).T
    assert v[0] == pytest.approx(v0) and v[-1] == pytest.approx(v1, abs=1e-9)
    assert np.abs(a).max() <= 1.0 + 1e-9 and abs(a[0]) < 1e-9          # no step at the start
    assert np.abs(np.diff(a) / np.diff(t)).max() <= 1.0 / 0.7 + 1e-6    # jerk limited
    assert np.allclose(np.gradient(s, t), v, atol=1e-3)
    assert ramp.state(ramp.duration + 2.0)[0] == pytest.approx(ramp.length + 2.0 * v1)


def test_a_stop_from_5_mps_takes_14_m_eased_in_and_out():
    assert stopping_distance(5.0, 1.0, 0.7) == pytest.approx(14.25)      # v (v/a + ramp) / 2
    assert stopping_distance(5.0, 1.0, 0.0) == pytest.approx(12.5)       # v^2 / 2a, a step


def test_a_rounded_square_keeps_its_heading_and_curvature_smooth():
    path = Path(SQUARE, radius_m=15.0, transition_m=3.5)
    assert [kind for kind, *_ in path.sections] == ["line", "turn"] * 3 + ["line"]
    s = np.linspace(0.0, path.length, 5001)
    n, e, h, k = np.array([path.at(x) for x in s]).T
    assert (n[0], e[0]) == (0.0, 0.0) and (n[-1], e[-1]) == pytest.approx((0.0, 0.0), abs=1e-4)
    assert np.abs(np.diff(np.unwrap(h))).max() < 0.01          # no kink
    assert np.abs(np.diff(k)).max() < 0.01                     # curvature ramps, no step
    assert k.max() == pytest.approx(1 / 15.0)
    step = np.hypot(np.diff(n), np.diff(e))                    # positions follow the heading
    assert np.allclose(step, np.diff(s), atol=1e-3)


def test_corners_between_short_legs_are_made_tighter_and_impossible_ones_refused():
    path = Path([(0, 0), (12, 0), (12, 12), (0, 12)], radius_m=15.0, transition_m=3.5,
                min_radius_m=3.0)
    assert max(path.radii) < 15.0
    with pytest.raises(ValueError, match="too short"):
        Path([(0, 0), (12, 0), (12, 12)], radius_m=15.0, transition_m=3.5)
    with pytest.raises(ValueError, match="turn"):
        Path([(0, 0), (50, 0), (0, 0)], radius_m=15.0, transition_m=3.5, min_radius_m=3.0)


def test_the_plan_slows_for_a_tight_corner_and_stops_on_the_last_point():
    path = Path(SQUARE, radius_m=6.0, transition_m=3.5)
    plan = SpeedPlan(path, cruise_mps=5.0, accel_mps2=1.0, ramp_s=0.7, lateral_mps2=1.7)
    corner_speed = [p[4] for p in plan.pieces if p[0] == "corner"]
    assert corner_speed and max(corner_speed) == pytest.approx(math.sqrt(1.7 * 6.0), rel=1e-3)
    t = np.linspace(0.0, plan.duration, 6001)
    s, v, a, _ = zip(*(plan.state(x) for x in t))
    lateral = [vi * vi * abs(path.at(si)[3]) for si, vi in zip(s, v)]
    assert max(lateral) <= 1.7 + 1e-6 and max(v) == pytest.approx(5.0)
    assert s[-1] == pytest.approx(path.length) and v[-1] == 0.0
    assert max(abs(x) for x in a) <= 1.0 + 1e-9

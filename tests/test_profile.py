"""The mission profile, flown against a vehicle that does exactly what it is told."""
import math

import pytest

from psi_vtol import config
from psi_vtol.profile import LAND_FROM_M, LANDING_PHASES, PATH_PHASES, Profile

P = config.profile()
DT = 0.05
HOME, YAW0 = (3.0, -2.0, -0.1), 0.5


def fly(profile, pos=HOME, t=100.0, disarm_below_m=0.2):
    """Follow the setpoints perfectly. Returns the time history of (t, phase, setpoint, pos)."""
    pos, vel, history = list(pos), [0.0, 0.0, 0.0], []
    armed = True
    while not profile.done and t < 400.0:
        phase = profile.phase
        sp = profile.update(t, tuple(pos), tuple(vel), armed)
        history.append((t, phase, sp, tuple(pos)))
        for i, (target, speed) in enumerate(((sp.x, sp.vx), (sp.y, sp.vy), (sp.z, sp.vz))):
            if not math.isnan(target):          # position commanded: move towards it at 2 m/s
                vel[i] = max(-2.0, min(2.0, (target - pos[i]) / DT))
            elif not math.isnan(speed):
                vel[i] = speed
            pos[i] += vel[i] * DT
        if phase == "land":                     # PX4's Land mode: sink, then disarm on the ground
            pos[2] += 0.7 * DT
            armed = profile.home[2] - pos[2] > disarm_below_m
        t += DT
    return history


def test_all_phases_run_in_order_and_finish():
    profile = Profile(P, HOME, YAW0)
    fly(profile)
    assert profile.done
    assert tuple(profile.times) == PATH_PHASES + LANDING_PHASES
    ends = [profile.times[name] for name in profile.times]
    assert all(a[1] == b[0] for a, b in zip(ends, ends[1:]))    # no gap between phases


def test_phase_lengths_follow_the_mission_file():
    profile = Profile(P, HOME, YAW0)
    fly(profile)
    length = {name: end - start for name, (start, end) in profile.times.items()}
    ramp = P["cruise_mps"] / P["accel_mps2"]
    assert length["hover1"] == pytest.approx(P["hover_s"], abs=2 * DT)
    assert length["accel"] == pytest.approx(ramp, abs=2 * DT)
    assert length["cruise"] == pytest.approx(P["cruise_s"], abs=2 * DT)
    assert length["decel"] == pytest.approx(ramp, abs=1.0)
    assert length["hover2"] == pytest.approx(P["hover_s"], abs=2 * DT)


def test_commanded_speed_never_changes_faster_than_the_limit():
    """PX4 uses offboard setpoints unsmoothed: a step in the setpoint is a step in the pusher."""
    profile = Profile(P, HOME, YAW0)
    history = [(t, math.hypot(sp.vx, sp.vy)) for t, phase, sp, _ in fly(profile)
               if phase in ("accel", "cruise", "decel")]
    for (t0, v0), (t1, v1) in zip(history, history[1:]):
        assert abs(v1 - v0) <= P["accel_mps2"] * (t1 - t0) + 1e-9
    assert max(v for _, v in history) == pytest.approx(P["cruise_mps"])


def test_flies_level_along_the_first_heading():
    profile = Profile(P, HOME, YAW0)
    for _, phase, sp, _ in fly(profile):
        assert sp.yaw == YAW0
        if phase in ("accel", "cruise", "decel"):
            assert sp.z == HOME[2] - P["alt_m"]
            speed = math.hypot(sp.vx, sp.vy)
            if speed > 0:
                assert math.atan2(sp.vy, sp.vx) == pytest.approx(YAW0)


def test_hover_and_descent_stay_over_the_point_where_the_vehicle_stopped():
    profile = Profile(P, HOME, YAW0)
    history = fly(profile)
    stopped = next(pos for _, phase, _, pos in history if phase == "hover2")
    assert profile.hold == stopped
    for _, phase, sp, _ in history:
        if phase in ("hover2", "descend"):
            assert (sp.x, sp.y) == stopped[:2]
    lowest = min(HOME[2] - pos[2] for _, phase, _, pos in history if phase == "descend")
    assert lowest == pytest.approx(LAND_FROM_M, abs=0.2)


def test_path_and_landing_can_be_flown_by_two_objects():
    """The ROS 2 nodes split the flight: the path node, then the landing node."""
    path = Profile(P, HOME, YAW0, phases=PATH_PHASES)
    history = fly(path)
    assert path.done and tuple(path.times) == PATH_PHASES
    t, _, _, where = history[-1]

    landing = Profile(P, (where[0], where[1], HOME[2]), YAW0, phases=LANDING_PHASES)
    fly(landing, pos=where, t=t)
    assert landing.done and tuple(landing.times) == LANDING_PHASES
    assert landing.hold == where


def test_a_phase_that_takes_too_long_is_reported():
    profile = Profile(P, HOME, YAW0)
    profile.update(0.0, HOME, (0.0, 0.0, 0.0))                  # the vehicle never climbs
    assert not profile.overdue(59.0)
    assert profile.overdue(61.0)


def test_acceleration_does_not_finish_if_vehicle_never_reaches_speed():
    profile = Profile(P, HOME, 0, phases=("accel", "cruise"))
    for i in range(400):
        profile.update(i * .1, HOME, (0, 0, 0))
    assert profile.phase == "accel"
    assert profile.overdue(40)


def test_heading_into_wind_needs_the_wind():
    """Without a wind it would quietly fly along the track, unlike the simulation."""
    with pytest.raises(ValueError, match="wind"):
        Profile(dict(P, heading_into_wind=1), HOME, YAW0)
    Profile(dict(P, heading_into_wind=1), HOME, YAW0, wind_ne=(0.0, 0.0))  # a measured calm


def test_heading_into_wind_turns_no_faster_than_the_turn_rate():
    profile = Profile(dict(P, heading_into_wind=1), HOME, 0.0, wind_ne=(0.0, -6.0))
    yaws = [(t, sp.yaw) for t, _, sp, _ in fly(profile)]
    for (t0, y0), (t1, y1) in zip(yaws, yaws[1:]):
        assert abs(y1 - y0) <= math.radians(20.0) * 0.1 + 1e-9
    assert max(y for _, y in yaws) > math.radians(30)           # the nose did turn into the wind

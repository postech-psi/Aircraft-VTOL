"""The mission profile, flown against a vehicle that does exactly what it is told."""
import math

import pytest

from psi_vtol import config
from psi_vtol.profile import (APPROACH_MPS, ARRIVE_M, LAND_FROM_M, LANDING_PHASES, MIN_CRUISE_S,
                              TURN_RATE, TURN_SETTLE_S, Profile)

P = config.profile()                                            # the 50 m square, right turns
OB = dict(P, cruise_mps=10.0, route_m=[[250.0, 0.0], [0.0, 0.0]])  # out and back at 10 m/s
DT = 0.05
HOME, YAW0 = (3.0, -2.0, -0.1), 0.5
SQUARE = ("climb", "hover1", "accel1", "cruise1", "decel1", "hover2",
          "turn2", "accel2", "cruise2", "decel2", "hover3",
          "turn3", "accel3", "cruise3", "decel3", "hover4",
          "turn4", "accel4", "cruise4", "decel4", "hover5")
OUT_AND_BACK = ("climb", "hover1", "accel1", "cruise1", "decel1", "hover2",
                "turn2", "accel2", "cruise2", "decel2", "hover3")
ROUTES = pytest.mark.parametrize("p", [P, OB], ids=["square", "out_and_back"])


def fly(profile, pos=HOME, t=100.0, disarm_below_m=0.2):
    """Follow the setpoints perfectly. Returns the time history of (t, phase, setpoint, pos)."""
    pos, vel, history = list(pos), [0.0, 0.0, 0.0], []
    armed = True
    while not profile.done and t < 600.0:
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


def test_the_default_route_is_a_50_m_square_flown_with_right_turns():
    profile = Profile(P, HOME, YAW0)
    assert P["route_m"] == [[50.0, 0.0], [50.0, -50.0], [0.0, -50.0], [0.0, 0.0]]
    assert [leg["length"] for leg in profile.legs] == pytest.approx([50.0] * 4)
    for before, after in zip(profile.legs, profile.legs[1:]):     # NED: a right turn adds 90 deg
        assert math.remainder(after["heading"] - before["heading"], 2 * math.pi) == pytest.approx(math.pi / 2)
    c, s = math.cos(YAW0), math.sin(YAW0)                           # the second corner lies right
    right = (profile.legs[1]["end"][0] - HOME[0]) * -s + (profile.legs[1]["end"][1] - HOME[1]) * c
    assert right == pytest.approx(50.0)
    fly(profile)
    assert profile.done
    assert tuple(profile.times) == SQUARE + LANDING_PHASES
    ends = list(profile.times.values())
    assert all(a[1] == b[0] for a, b in zip(ends, ends[1:]))    # no gap between phases


def test_out_and_back_flies_out_turns_and_comes_back():
    profile = Profile(OB, HOME, YAW0)
    fly(profile)
    assert profile.done and tuple(profile.times) == OUT_AND_BACK + LANDING_PHASES


@pytest.mark.parametrize("p, side_m, turn_rad", [(P, 50.0, math.pi / 2), (OB, 250.0, math.pi)],
                         ids=["square", "out_and_back"])
def test_phase_lengths_follow_the_mission_file(p, side_m, turn_rad):
    profile = Profile(p, HOME, YAW0)
    fly(profile)
    length = {name: end - start for name, (start, end) in profile.times.items()}
    v = p["cruise_mps"]
    ramp, braking = v / p["accel_mps2"], math.pi / 2 * v / p["accel_mps2"]
    cruise = (side_m - v * ramp / 2 - v * braking / 2) / v       # square 3.6 s, out and back 12.1 s
    assert length["hover1"] == pytest.approx(p["hover_s"], abs=2 * DT)
    for leg in range(1, len(profile.legs) + 1):
        assert length[f"accel{leg}"] == pytest.approx(ramp, abs=2 * DT)
        assert length[f"cruise{leg}"] == pytest.approx(cruise, abs=2 * DT)
        assert length[f"decel{leg}"] == pytest.approx(braking, abs=1.0)
        assert length[f"hover{leg + 1}"] >= p["hover_s"]
    assert length["turn2"] == pytest.approx(turn_rad / TURN_RATE + TURN_SETTLE_S, abs=3 * DT)


@ROUTES
def test_commanded_speed_never_changes_faster_than_the_limit(p):
    """PX4 uses offboard setpoints unsmoothed: a step in the setpoint is a step in the pusher."""
    profile = Profile(p, HOME, YAW0)
    history = [(t, math.hypot(sp.vx, sp.vy)) for t, phase, sp, _ in fly(profile)
               if phase.rstrip("0123456789") in ("accel", "cruise", "decel")]
    for (t0, v0), (t1, v1) in zip(history, history[1:]):
        assert abs(v1 - v0) <= p["accel_mps2"] * (t1 - t0) + 1e-9
    assert max(v for _, v in history) == pytest.approx(p["cruise_mps"])


@ROUTES
def test_slowing_down_starts_and_ends_gently(p):
    """A step into braking steps the pitch setpoint and cuts the pusher at once (report 10-18, §6)."""
    profile = Profile(p, HOME, YAW0)
    history = [(t, phase, math.hypot(sp.vx, sp.vy)) for t, phase, sp, _ in fly(profile)]
    a = p["accel_mps2"]
    for leg in range(1, len(profile.legs) + 1):
        speeds = [(t, v) for t, phase, v in history if phase == f"decel{leg}"]
        (t0, v0), stop = speeds[0], next(t for t, v in speeds if v == 0.0)
        early = [v0 - v for t, v in speeds if t - t0 <= 0.5]
        late = [v for t, v in speeds if 0.0 <= stop - t <= 0.5]
        assert max(early) < 0.25 * a * 0.5         # at the limit from the start it would be a * 0.5
        assert max(late) < 0.25 * a * 0.5
        rates = [(va - vb) / (tb - ta) for (ta, va), (tb, vb) in zip(speeds, speeds[1:])]
        assert max(rates) == pytest.approx(a, rel=0.01)    # the limit is reached, halfway


@ROUTES
def test_each_leg_is_flown_level_and_nose_first(p):
    profile = Profile(p, HOME, YAW0)
    for _, phase, sp, _ in fly(profile):
        if phase.rstrip("0123456789") in ("accel", "cruise", "decel"):
            heading = profile.legs[int(phase[-1]) - 1]["heading"]
            assert sp.z == HOME[2] - p["alt_m"]
            assert math.remainder(sp.yaw - heading, 2 * math.pi) == pytest.approx(0, abs=1e-9)
            if math.hypot(sp.vx, sp.vy) > 0:
                assert math.atan2(sp.vy, sp.vx) == pytest.approx(math.atan2(math.sin(heading), math.cos(heading)))
    assert profile.legs[0]["heading"] == pytest.approx(YAW0)


@ROUTES
def test_heading_never_turns_faster_than_the_turn_rate(p):
    profile = Profile(p, HOME, YAW0)
    yaws = [(t, sp.yaw) for t, _, sp, _ in fly(profile)]
    for (t0, y0), (t1, y1) in zip(yaws, yaws[1:]):
        assert abs(math.remainder(y1 - y0, 2 * math.pi)) <= TURN_RATE * (t1 - t0) + 1e-9


def test_a_hover_starts_where_the_vehicle_stopped_and_slides_onto_the_waypoint():
    """PX4 flies a step in the position setpoint with a hard tilt."""
    profile = Profile(P, HOME, YAW0)
    history = fly(profile)
    hover = [(t, sp, pos) for t, phase, sp, pos in history if phase == "hover2"]
    assert (hover[0][1].x, hover[0][1].y) == pytest.approx(hover[0][2][:2])
    for (t0, a, _), (t1, b, _) in zip(hover, hover[1:]):
        assert math.hypot(b.x - a.x, b.y - a.y) <= APPROACH_MPS * (t1 - t0) + 1e-9
    assert (hover[-1][1].x, hover[-1][1].y) == pytest.approx(profile.legs[0]["end"])


@ROUTES
def test_every_turn_is_flown_in_a_hover_over_its_waypoint(p):
    profile = Profile(p, HOME, YAW0)
    for _, phase, sp, _ in fly(profile):
        if phase.startswith("turn"):
            corner = profile.legs[int(phase[4:]) - 1]["start"]
            assert (sp.x, sp.y) == pytest.approx(corner) and math.isnan(sp.vx)


@ROUTES
def test_it_lands_where_it_took_off(p):
    profile = Profile(p, HOME, YAW0)
    history = fly(profile)
    assert profile.hold[:2] == pytest.approx(HOME[:2])
    for _, phase, sp, _ in history:
        if phase == "descend":
            assert (sp.x, sp.y) == pytest.approx(HOME[:2])
    arrived = next(pos for _, phase, _, pos in history if phase == "descend")
    assert math.hypot(arrived[0] - HOME[0], arrived[1] - HOME[1]) < ARRIVE_M
    lowest = min(HOME[2] - pos[2] for _, phase, _, pos in history if phase == "descend")
    assert lowest == pytest.approx(LAND_FROM_M, abs=0.2)


def test_a_waypoint_to_the_side_turns_first_and_a_short_leg_still_has_a_cruise():
    p = dict(P, cruise_mps=10.0, route_m=[[0.0, 40.0]])  # 40 m to the left: too short for 10 m/s
    profile = Profile(p, HOME, YAW0)
    assert profile.phases[:3] == ("climb", "hover1", "turn1")
    v, a = profile.legs[0]["speed"], p["accel_mps2"]
    speeding_up, slowing_down = v * v / (2 * a), math.pi * v * v / (4 * a)
    assert v < 10.0 and speeding_up + v * MIN_CRUISE_S + slowing_down == pytest.approx(40.0)
    fly(profile)
    assert profile.done
    assert profile.times["cruise1"][1] - profile.times["cruise1"][0] == pytest.approx(MIN_CRUISE_S, abs=3 * DT)
    assert profile.hold[:2] == pytest.approx(profile.legs[0]["end"])


def test_the_path_and_the_landing_can_be_flown_by_two_objects():
    """The ROS 2 nodes split the flight: the path node, then the landing node."""
    path = Profile(P, HOME, YAW0, part="path")
    history = fly(path)
    assert path.done and tuple(path.times) == SQUARE
    t, _, _, where = history[-1]

    landing = Profile(P, (where[0], where[1], HOME[2]), YAW0, part="landing")
    fly(landing, pos=where, t=t)
    assert landing.done and tuple(landing.times) == LANDING_PHASES
    assert landing.hold == where


def test_a_phase_that_takes_too_long_is_reported():
    profile = Profile(P, HOME, YAW0)
    profile.update(0.0, HOME, (0.0, 0.0, 0.0))                  # the vehicle never climbs
    assert not profile.overdue(59.0)
    assert profile.overdue(61.0)


def test_acceleration_does_not_finish_if_vehicle_never_reaches_speed():
    profile = Profile(P, HOME, 0, part="path")
    hover, now = (HOME[0], HOME[1], HOME[2] - P["alt_m"]), 100.0
    while profile.phase != "accel1":                            # climbed, hovered
        profile.update(now, hover, (0, 0, 0))
        now += DT
    for i in range(400):                                        # it stays put instead of speeding up
        profile.update(now + i * .1, hover, (0, 0, 0))
    assert profile.phase == "accel1"
    assert profile.overdue(now + 40)


def test_a_route_with_a_leg_shorter_than_a_metre_is_refused():
    with pytest.raises(ValueError, match="shorter than 1 m"):
        Profile(dict(P, route_m=[[250.0, 0.0], [250.0, 0.5]]), HOME, YAW0)
    with pytest.raises(ValueError, match="no waypoints"):
        Profile(dict(P, route_m=[]), HOME, YAW0)


def test_heading_into_wind_needs_the_wind():
    """Without a wind it would quietly fly along the track, unlike the simulation."""
    with pytest.raises(ValueError, match="wind"):
        Profile(dict(P, heading_into_wind=1), HOME, YAW0)
    Profile(dict(P, heading_into_wind=1), HOME, YAW0, wind_ne=(0.0, 0.0))  # a measured calm


def test_heading_into_wind_flies_no_turns_and_turns_no_faster_than_the_turn_rate():
    profile = Profile(dict(P, heading_into_wind=1), HOME, 0.0, wind_ne=(0.0, -6.0))
    assert not any(phase.startswith("turn") for phase in profile.phases)
    yaws = [(t, sp.yaw) for t, _, sp, _ in fly(profile)]
    for (t0, y0), (t1, y1) in zip(yaws, yaws[1:]):
        assert abs(y1 - y0) <= TURN_RATE * 0.1 + 1e-9
    assert max(y for _, y in yaws) > math.radians(30)           # the nose did turn into the wind

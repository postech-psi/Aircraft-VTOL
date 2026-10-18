"""The mission profile, flown against a vehicle that does exactly what it is told."""
import math

import pytest

from psi_vtol import config
from psi_vtol.profile import (ARRIVE_M, FLY_KINDS, LAG_FAULT_M, LAND_FROM_M, LANDING_PHASES,
                              MIN_CRUISE_S, STALL_S, STOP_ACCEL_MPS2, STOP_RAMP_S, TRACK_MAX_MPS,
                              TRACK_P, TURN_ALIGNED_S, TURN_RATE, WIND_RATE, Profile, Setpoint, Stop,
                              for_px4, needs_land)
from psi_vtol.trajectory import SpeedRamp

P = config.profile()                                            # the 50 m square, flown through
STOP = dict(P, fly_through=0)                                   # the same square, stop and turn
OB = dict(P, cruise_mps=10.0, route_m=[[250.0, 0.0], [0.0, 0.0]])  # out and back at 10 m/s
DT = 0.05
HOME, YAW0 = (3.0, -2.0, -0.1), 0.5
SQUARE = ("climb", "hover1", "accel1", "cruise1", "decel1", "hover2",
          "turn2", "accel2", "cruise2", "decel2", "hover3",
          "turn3", "accel3", "cruise3", "decel3", "hover4",
          "turn4", "accel4", "cruise4", "decel4", "hover5")
THROUGH = ("climb", "hover1", "accel1", "cruise1", "corner1", "cruise2", "corner2",
           "cruise3", "corner3", "cruise4", "decel4", "hover5")
OUT_AND_BACK = ("climb", "hover1", "accel1", "cruise1", "decel1", "hover2",
                "turn2", "accel2", "cruise2", "decel2", "hover3")
ROUTES = pytest.mark.parametrize("p", [P, STOP, OB], ids=["square_through", "square_stop", "out_and_back"])


def fly(profile, pos=HOME, t=100.0, disarm_below_m=0.2, heading=YAW0):
    """Follow the setpoints perfectly. Returns the time history of (t, phase, setpoint, pos).

    A moving reference (position and velocity) is tracked exactly; a position alone is
    approached at 2 m/s; a velocity alone is flown. The nose is where it was last told to be.
    """
    pos, vel, history = list(pos), [0.0, 0.0, 0.0], []
    armed = True
    while not profile.done and t < 900.0:
        phase = profile.phase
        sp = profile.update(t, tuple(pos), tuple(vel), armed, (0.0, 0.0, heading))
        history.append((t, phase, sp, tuple(pos)))
        for i, (target, speed) in enumerate(((sp.x, sp.vx), (sp.y, sp.vy), (sp.z, sp.vz))):
            if not math.isnan(target) and not math.isnan(speed):
                pos[i], vel[i] = target, speed
                continue
            if not math.isnan(target):          # position commanded: move towards it at 2 m/s
                vel[i] = max(-2.0, min(2.0, (target - pos[i]) / DT))
            elif not math.isnan(speed):
                vel[i] = speed
            pos[i] += vel[i] * DT
        if not math.isnan(sp.yaw):
            heading = sp.yaw
        if phase == "land":                     # PX4's Land mode: sink, then disarm on the ground
            pos[2] += 0.7 * DT
            armed = profile.home[2] - pos[2] > disarm_below_m
        t += DT
    return history


def speeds(history):
    return [(t, math.hypot(sp.vx, sp.vy)) for t, phase, sp, _ in history if phase.rstrip("0123456789") in FLY_KINDS]


def test_the_default_route_is_a_50_m_square_flown_through_its_corners():
    profile = Profile(P, HOME, YAW0)
    assert P["route_m"] == [[50.0, 0.0], [50.0, -50.0], [0.0, -50.0], [0.0, 0.0]]
    assert P["fly_through"] == 1
    assert [leg["length"] for leg in profile.legs] == pytest.approx([50.0] * 4)
    for before, after in zip(profile.legs, profile.legs[1:]):     # NED: a right turn adds 90 deg
        assert math.remainder(after["heading"] - before["heading"], 2 * math.pi) == pytest.approx(math.pi / 2)
    c, s = math.cos(YAW0), math.sin(YAW0)                           # the second corner lies right
    right = (profile.legs[1]["end"][0] - HOME[0]) * -s + (profile.legs[1]["end"][1] - HOME[1]) * c
    assert right == pytest.approx(50.0)


@pytest.mark.parametrize("p, phases", [(P, THROUGH), (STOP, SQUARE), (OB, OUT_AND_BACK)],
                         ids=["square_through", "square_stop", "out_and_back"])
def test_each_route_flies_its_phases_in_order_without_a_gap(p, phases):
    """Out and back is flown stop and turn even when flying through: a 180 degree corner cannot
    be rounded on the way."""
    profile = Profile(p, HOME, YAW0)
    fly(profile)
    assert profile.done and tuple(profile.times) == phases + LANDING_PHASES
    ends = list(profile.times.values())
    assert all(a[1] == b[0] for a, b in zip(ends, ends[1:]))


@pytest.mark.parametrize("p, side_m, turn_rad", [(STOP, 50.0, math.pi / 2), (OB, 250.0, math.pi)],
                         ids=["square_stop", "out_and_back"])
def test_phase_lengths_follow_the_mission_file(p, side_m, turn_rad):
    profile = Profile(p, HOME, YAW0)
    fly(profile)
    length = {name: end - start for name, (start, end) in profile.times.items()}
    v = p["cruise_mps"]
    ramp = SpeedRamp(0.0, v, p["accel_mps2"], p["brake_ramp_s"])
    cruise = (side_m - 2 * ramp.length) / v                     # square 4.3 s, out and back 14.3 s
    assert length["hover1"] == pytest.approx(p["hover_s"], abs=2 * DT)
    for leg in range(1, len(profile.legs) + 1):
        assert length[f"accel{leg}"] == pytest.approx(ramp.duration, abs=2 * DT)
        assert length[f"cruise{leg}"] == pytest.approx(cruise, abs=2 * DT)
        assert length[f"decel{leg}"] == pytest.approx(ramp.duration, abs=2 * DT)
        assert length[f"hover{leg + 1}"] >= p["hover_s"]
    assert length["turn2"] == pytest.approx(turn_rad / TURN_RATE + TURN_ALIGNED_S, abs=3 * DT)


@ROUTES
def test_speed_changes_build_up_and_fade_out_and_never_pass_the_limit(p):
    """PX4 uses offboard setpoints unsmoothed: a step into braking steps the pitch setpoint and
    cuts the pusher at once (SITL)."""
    profile = Profile(p, HOME, YAW0)
    history = fly(profile)
    a = p["accel_mps2"]
    flown = speeds(history)
    for (t0, v0), (t1, v1) in zip(flown, flown[1:]):
        assert abs(v1 - v0) <= a * (t1 - t0) + 1e-9
    assert max(v for _, v in flown) == pytest.approx(p["cruise_mps"])
    for name in profile.times:
        if name.rstrip("0123456789") not in ("accel", "decel"):
            continue
        part = [(t, math.hypot(sp.vx, sp.vy)) for t, phase, sp, _ in history if phase == name]
        (t0, v0), (t1, v1) = part[0], part[-1]
        early = [abs(v - v0) for t, v in part if t - t0 <= 0.5]
        late = [abs(v - v1) for t, v in part if t1 - t <= 0.5]
        assert max(early) < 0.5 * a * 0.5 and max(late) < 0.5 * a * 0.5   # at the limit: a * 0.5
        rates = [abs(vb - va) / (tb - ta) for (ta, va), (tb, vb) in zip(part, part[1:])]
        assert max(rates) == pytest.approx(a, rel=0.02)              # the limit is reached between


@ROUTES
def test_the_reference_stops_exactly_on_each_waypoint(p):
    profile = Profile(p, HOME, YAW0)
    history = fly(profile)
    for run in profile.runs:
        last = run["legs"][-1]
        name = next(n for n in reversed(profile.times) if n.endswith(str(last + 1))
                    and n.rstrip("0123456789") in FLY_KINDS)
        sp = [sp for _, phase, sp, _ in history if phase == name][-1]
        assert (sp.x, sp.y) == pytest.approx(profile.legs[last]["end"], abs=0.05)
        assert math.hypot(sp.vx, sp.vy) < 0.05


def test_corners_are_flown_banked_with_the_nose_along_the_path():
    profile = Profile(P, HOME, YAW0)
    history = fly(profile)
    corner = [sp for _, phase, sp, _ in history if phase.startswith("corner")]
    assert corner
    lateral = []
    for sp in corner:
        v = math.hypot(sp.vx, sp.vy)
        assert math.atan2(sp.vy, sp.vx) == pytest.approx(math.remainder(sp.yaw, 2 * math.pi), abs=1e-6)
        sideways = -sp.ax * math.sin(sp.yaw) + sp.ay * math.cos(sp.yaw)     # to the right
        lateral.append(sideways)
        assert sp.yawspeed == pytest.approx(sideways / v, rel=1e-6)          # a coordinated turn
    assert max(lateral) == pytest.approx(P["cruise_mps"] ** 2 / P["turn_radius_m"], rel=0.01)
    assert max(lateral) <= P["lateral_mps2"]
    assert lateral[0] < 0.2 * max(lateral) and lateral[-1] < 0.2 * max(lateral)   # bank eases in


def test_the_path_rounds_each_corner_inside_the_waypoint():
    profile = Profile(P, HOME, YAW0)
    points = profile.polyline(0.1)
    for leg in profile.legs[:-1]:
        nearest = min(math.hypot(n - leg["end"][0], e - leg["end"][1]) for n, e in points)
        assert 5.0 < nearest < 7.0          # about 0.41 of the 15 m radius


@pytest.mark.parametrize("p", [STOP, OB], ids=["square_stop", "out_and_back"])
def test_each_leg_is_flown_level_and_nose_first(p):
    profile = Profile(p, HOME, YAW0)
    for _, phase, sp, _ in fly(profile):
        if phase.rstrip("0123456789") in FLY_KINDS:
            heading = profile.legs[int(phase[-1]) - 1]["heading"]
            assert sp.z == HOME[2] - p["alt_m"]
            assert math.remainder(sp.yaw - heading, 2 * math.pi) == pytest.approx(0, abs=1e-9)
            if math.hypot(sp.vx, sp.vy) > 0:
                assert math.atan2(sp.vy, sp.vx) == pytest.approx(math.atan2(math.sin(heading), math.cos(heading)))
    assert profile.legs[0]["heading"] == pytest.approx(YAW0)


@ROUTES
def test_heading_never_turns_faster_than_the_turn_rate(p):
    """20 deg/s in a hover; a 15 m corner at 5 m/s turns at 19 deg/s."""
    yaws = [(t, sp.yaw) for t, _, sp, _ in fly(Profile(p, HOME, YAW0))]
    for (t0, y0), (t1, y1) in zip(yaws, yaws[1:]):
        assert abs(math.remainder(y1 - y0, 2 * math.pi)) <= TURN_RATE * (t1 - t0) + 1e-9


@pytest.mark.parametrize("p", [STOP, OB], ids=["square_stop", "out_and_back"])
def test_every_turn_is_flown_in_a_hover_over_its_waypoint(p):
    profile = Profile(p, HOME, YAW0)
    for _, phase, sp, _ in fly(profile):
        if phase.startswith("turn"):
            corner = profile.legs[int(phase[4:]) - 1]["start"]
            assert (sp.x, sp.y) == pytest.approx(corner) and math.isnan(sp.vx)


def test_a_turn_ends_only_when_the_nose_is_there():
    """A turn ended on the clock once let the speed-up start with the nose 29 degrees off (SITL)."""
    profile = Profile(STOP, HOME, YAW0)
    history = fly(profile)                  # to learn when turn2 starts
    t_turn = profile.times["turn2"][0]
    late = Profile(STOP, HOME, YAW0)
    pos, t = list(HOME), 100.0
    for t_, phase, sp, where in history:
        if t_ >= t_turn:
            break
        late.update(t_, where, (0.0, 0.0, 0.0), True, (0.0, 0.0, sp.yaw if not math.isnan(sp.yaw) else YAW0))
        pos, t = list(where), t_
    stuck = (0.0, 0.0, profile.legs[0]["heading"])      # the nose stays on the old leg
    for i in range(400):
        late.update(t + i * DT, tuple(pos), (0.0, 0.0, 0.0), True, stuck)
    assert late.phase == "turn2"
    there = (0.0, 0.0, profile.legs[1]["heading"])
    for i in range(400, 400 + int(TURN_ALIGNED_S / DT) + 2):
        late.update(t + i * DT, tuple(pos), (0.0, 0.0, 0.0), True, there)
    assert late.phase == "accel2"


@ROUTES
def test_it_lands_where_it_took_off(p):
    profile = Profile(p, HOME, YAW0)
    history = fly(profile)
    assert profile.hold[:2] == pytest.approx(HOME[:2])
    for _, phase, sp, _ in history:
        if phase in ("align", "descend"):
            assert (sp.x, sp.y) == pytest.approx(HOME[:2])
    arrived = next(pos for _, phase, _, pos in history if phase == "descend")
    assert math.hypot(arrived[0] - HOME[0], arrived[1] - HOME[1]) < ARRIVE_M
    lowest = min(HOME[2] - pos[2] for _, phase, _, pos in history if phase == "descend")
    assert lowest == pytest.approx(LAND_FROM_M, abs=0.2)


def test_before_descending_the_nose_turns_into_the_wind():
    over = (HOME[0], HOME[1], HOME[2] - P["alt_m"])
    def align(lean):
        landing = Profile(P, HOME, 0.0, part="landing")
        rates = []
        for i in range(60):
            aligning = landing.phase == "align"
            sp = landing.update(100.0 + i * DT, over, (0.0, 0.0, 0.0), True, (*lean, 0.0))
            if aligning:
                rates.append(sp.yawspeed)
        return landing, rates
    _, rates = align((math.radians(5.0), 0.0))          # leaning right: wind from the right
    assert rates[-1] > 0.0
    _, rates = align((math.radians(-5.0), 0.0))
    assert rates[-1] < 0.0
    _, rates = align((0.0, math.radians(5.0)))          # nose up: wind from behind, turn round
    assert rates[-1] == pytest.approx(WIND_RATE)
    landing, rates = align((0.0, 0.0))                  # calm: settles and goes down
    assert set(rates) == {0.0} and landing.phase == "descend"


def test_a_route_below_the_pusher_height_is_refused():
    with pytest.raises(ValueError, match="pusher"):
        Profile(dict(P, alt_m=5.0), HOME, YAW0)


def test_a_waypoint_to_the_side_turns_first_and_a_short_leg_still_has_a_cruise():
    p = dict(P, cruise_mps=10.0, route_m=[[0.0, 40.0]])  # 40 m to the left: too short for 10 m/s
    profile = Profile(p, HOME, YAW0)
    assert profile.phases[:3] == ("climb", "hover1", "turn1")
    v = profile.legs[0]["speed"]
    ramp = SpeedRamp(0.0, v, p["accel_mps2"], p["brake_ramp_s"])
    assert v < 10.0 and 2 * ramp.length + v * MIN_CRUISE_S == pytest.approx(40.0)
    fly(profile)
    assert profile.done
    assert profile.times["cruise1"][1] - profile.times["cruise1"][0] == pytest.approx(MIN_CRUISE_S, abs=3 * DT)
    assert profile.hold[:2] == pytest.approx(profile.legs[0]["end"])


def test_route_heading_fixes_the_square_whatever_way_the_vehicle_faces():
    """A vehicle set down facing the wrong way still flies the square that was planned."""
    profile = Profile(dict(P, route_heading_deg=90.0), HOME, YAW0)
    assert profile.legs[0]["heading"] == pytest.approx(math.pi / 2)    # first leg east
    assert profile.phases[:3] == ("climb", "hover1", "turn1")
    fly(profile)
    assert profile.done


def test_the_path_and_the_landing_can_be_flown_by_two_objects():
    """The ROS 2 nodes split the flight: the path node, then the landing node."""
    path = Profile(P, HOME, YAW0, part="path")
    history = fly(path)
    assert path.done and tuple(path.times) == THROUGH
    t, _, _, where = history[-1]

    landing = Profile(P, (where[0], where[1], HOME[2]), YAW0, part="landing")
    fly(landing, pos=where, t=t)
    assert landing.done and tuple(landing.times) == LANDING_PHASES
    assert landing.hold == where


def test_a_phase_that_takes_too_long_is_reported():
    profile = Profile(P, HOME, YAW0)
    profile.update(0.0, HOME, (0.0, 0.0, 0.0))                  # the vehicle never climbs
    assert not profile.overdue(profile.limit_s() - 1.0)
    assert profile.overdue(profile.limit_s() + 1.0)


def fly_exactly_until(profile, phase, now=100.0):
    """Follow the reference exactly until the phase starts. Returns (position, velocity, time)."""
    pos, vel = [profile.home[0], profile.home[1], profile.z_fly], [0.0, 0.0, 0.0]
    while profile.phase != phase:
        sp = profile.update(now, tuple(pos), tuple(vel))
        if not math.isnan(sp.vx):
            pos[:2], vel[:2] = [sp.x, sp.y], [sp.vx, sp.vy]
        now += DT
    return pos, vel, now


def coast(profile, pos, vel, now, drag_mps2, seconds=30.0):
    """Keep to the path but only lose speed to drag, as without a pusher, until a fault.
    Returns how far the vehicle was behind the reference then."""
    lag = 0.0
    for _ in range(int(seconds / DT)):
        sp = profile.update(now, tuple(pos), tuple(vel))
        if profile.fault:
            break
        speed = max(math.hypot(*vel[:2]) - drag_mps2 * DT, 0.0)
        heading = math.atan2(sp.vy, sp.vx)
        vel[:2] = [speed * math.cos(heading), speed * math.sin(heading)]
        pos[0], pos[1] = pos[0] + vel[0] * DT, pos[1] + vel[1] * DT
        lag = math.hypot(sp.x - pos[0], sp.y - pos[1])
        now += DT
    return lag


def test_no_forward_speed_is_a_pusher_fault():
    """With the pusher dead, assist holds the nose level and the vehicle stays put (SITL)."""
    profile = Profile(P, HOME, 0.0, part="path")
    hover, _, now = fly_exactly_until(profile, "accel1")
    for i in range(int((STALL_S + 1.0) / DT)):                  # it stays put instead of speeding up
        profile.update(now + i * DT, tuple(hover), (0, 0, 0))
    assert profile.fault and profile.fault.startswith("pusher")
    assert needs_land(profile.fault)


def test_coasting_down_in_cruise_is_a_pusher_fault_before_it_falls_far_behind():
    """A pusher that stops in cruise: assist cannot tilt the nose down, drag slows the vehicle."""
    profile = Profile(P, HOME, 0.0, part="path")
    lag = coast(profile, *fly_exactly_until(profile, "cruise1"), drag_mps2=0.3)
    assert profile.fault and profile.fault.startswith("pusher: ")
    assert lag < LAG_FAULT_M


def test_a_pusher_lost_at_7_mps_is_seen_through_the_corners_and_slow_downs():
    """At 7 m/s the middle legs only speed up and slow down. A check that restarted with every
    slow-down of the plan saw this as falling behind, and Hold without forward thrust followed:
    switched to Land after it, the vehicle pitched to 48 deg in SITL."""
    profile = Profile(dict(P, cruise_mps=7.0), HOME, 0.0, part="path")
    assert "cruise2" not in profile.phases
    lag = coast(profile, *fly_exactly_until(profile, "corner1"), drag_mps2=0.3)
    assert profile.fault and profile.fault.startswith("pusher: ")
    assert lag < LAG_FAULT_M


def test_falling_far_behind_the_reference_is_a_fault_that_lands_too():
    """Half the planned speed: a weak pusher, not a dead one. It still cannot make way forward."""
    profile = Profile(P, HOME, 0.0, part="path")
    pos, _, now = fly_exactly_until(profile, "accel1")
    vel = (0.0, 0.0, 0.0)
    for i in range(int(30.0 / DT)):                            # half the planned speed
        sp = profile.update(now + i * DT, tuple(pos), vel)
        vel = (0.5 * sp.vx, 0.5 * sp.vy, 0.0)
        pos[0] += vel[0] * DT
        pos[1] += vel[1] * DT
        if profile.fault:
            break
    assert profile.fault and profile.fault.startswith("falling behind")
    assert float(profile.fault.split(" by ")[1].split()[0]) >= LAG_FAULT_M
    assert needs_land(profile.fault) and not needs_land("timeout in hover3")


def test_a_route_without_waypoints_or_with_a_leg_under_a_metre_is_refused():
    with pytest.raises(ValueError, match="shorter than 1 m"):
        Profile(dict(P, route_m=[[250.0, 0.0], [250.0, 0.5]]), HOME, YAW0)
    with pytest.raises(ValueError, match="no waypoints"):
        Profile(dict(P, route_m=[]), HOME, YAW0)


def test_stop_brings_a_moving_vehicle_to_rest_without_a_step():
    """What the failsafe flies. Holding the point where a 5 m/s vehicle was pitched it to 48 deg."""
    stop = Stop(10.0, (1.0, 2.0, -20.0), (3.0, 4.0, 0.0), 0.3)
    ramp = SpeedRamp(5.0, 0.0, STOP_ACCEL_MPS2, STOP_RAMP_S)
    first = stop.setpoint(10.0)
    assert (first.x, first.y, first.z) == (1.0, 2.0, -20.0)
    assert math.hypot(first.vx, first.vy) == pytest.approx(5.0) and math.hypot(first.ax, first.ay) < 1e-9
    accels = [math.hypot(stop.setpoint(10.0 + k * 0.01).ax, stop.setpoint(10.0 + k * 0.01).ay)
              for k in range(int(ramp.duration * 100))]
    assert max(accels) == pytest.approx(STOP_ACCEL_MPS2)
    last = stop.setpoint(10.0 + ramp.duration + 1.0)
    assert math.hypot(last.vx, last.vy) == pytest.approx(0.0, abs=1e-9)
    assert math.hypot(last.x - 1.0, last.y - 2.0) == pytest.approx(ramp.length)
    assert stop.stopped(10.0 + ramp.duration) and last.yaw == 0.3


def test_a_moving_reference_is_sent_as_a_velocity_pulled_towards_its_point():
    """A stale setpoint must not hold a point the vehicle has passed: it braked at 29 deg (SITL)."""
    moving = Setpoint(x=10.0, y=0.0, z=-20.0, vx=5.0, vy=0.0, ax=0.5, ay=0.0, yaw=0.0)
    sent = for_px4(moving, (9.0, 0.5, -20.0))
    assert math.isnan(sent.x) and math.isnan(sent.y) and sent.z == -20.0
    assert (sent.vx, sent.vy) == pytest.approx((5.0 + TRACK_P * 1.0, -TRACK_P * 0.5))
    assert (sent.ax, sent.yaw) == (0.5, 0.0)
    far = for_px4(moving, (-40.0, 0.0, -20.0))                  # the pull is limited
    assert far.vx == pytest.approx(5.0 + TRACK_MAX_MPS)
    hover = Setpoint(x=10.0, y=0.0, z=-20.0)                     # a point to hold stays a point
    assert for_px4(hover, (9.0, 0.0, -20.0)) == hover
    resting = Setpoint(x=10.0, y=0.0, z=-20.0, vx=0.0, vy=0.0)
    assert for_px4(resting, (9.0, 0.0, -20.0)) == resting
    assert for_px4(moving, None) == moving

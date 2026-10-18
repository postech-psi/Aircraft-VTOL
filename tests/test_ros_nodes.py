"""The ROS 2 nodes' own methods, loaded from their source and run without ROS installed."""
import ast
import math
from types import SimpleNamespace

import pytest

from psi_vtol import config
from psi_vtol.profile import STOP_ACCEL_MPS2, STOP_RAMP_S, Profile, Stop, for_px4, needs_land
from psi_vtol.trajectory import stopping_distance


def method(filename, class_name, name, **namespace):
    path = config.REPO / "ros2/ws/src/vtol_mission/vtol_mission" / filename
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = {"math": math, "time": SimpleNamespace(monotonic=lambda: 100.),
             "TrajectorySetpoint": SimpleNamespace, "String": SimpleNamespace, "NANS": [math.nan] * 3,
             **namespace}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(path), "exec"), scope)
    return scope[name]


def test_hold_brakes_along_the_way_and_does_not_follow_position_drift():
    """The mux's own stop: latched once, along the vehicle's way, and not moved when it drifts."""
    clock = SimpleNamespace(t=100.0)
    hold = method("setpoint_mux.py", "SetpointMux", "hold_setpoint", Stop=Stop, for_px4=for_px4,
                  time=SimpleNamespace(monotonic=lambda: clock.t))
    state = SimpleNamespace(position=[0., 0., -20.], velocity=(5., 0., 0.), heading=.1, stop=None)
    first = hold(state)
    assert all(map(math.isnan, first.position[:2])) and first.velocity[:2] == [5., 0.]
    state.position, state.velocity, state.heading = [5., -2., -18.], (0., 0., 0.), .5
    clock.t = 120.0
    later = hold(state)
    assert later.position == pytest.approx([stopping_distance(5., STOP_ACCEL_MPS2, STOP_RAMP_S), 0., -20.])
    assert later.velocity[:2] == pytest.approx([0., 0.])
    assert later.yaw == first.yaw == .1


def test_stale_position_stops_offboard_stream():
    forward = method("setpoint_mux.py", "SetpointMux", "forward")
    published = []
    state = SimpleNamespace(position_received_at=98., state_timeout_s=1.,
                            status_pub=SimpleNamespace(publish=published.append))
    # No mode/setpoint publisher on purpose: attempting either would raise.
    forward(state)
    assert published[0].data == "STALE_LOCAL_POSITION_STOP_STREAM"


def test_invalid_position_does_not_refresh_watchdog():
    receive = method("setpoint_mux.py", "SetpointMux", "on_local_position")
    state = SimpleNamespace(position_received_at=99., last_position_timestamp=None)
    receive(state, SimpleNamespace(xy_valid=False, z_valid=True))
    assert state.position_received_at is None


def test_replayed_position_cannot_refresh_watchdog():
    receive = method("setpoint_mux.py", "SetpointMux", "on_local_position")
    state = SimpleNamespace(position_received_at=98., last_position_timestamp=1000)
    receive(state, SimpleNamespace(timestamp=1000))
    assert state.position_received_at == 98.


@pytest.mark.parametrize("filename, class_name", [("path_node.py", "PathNode"),
                                                  ("landing_node.py", "LandingNode")])
def test_start_fails_cleanly_when_the_profile_refuses_the_mission(filename, class_name):
    too_low = dict(config.profile(), alt_m=5.0)
    start = method(filename, class_name, "start", Profile=Profile,
                   config=SimpleNamespace(profile=lambda mission: too_low))
    failures = []
    state = SimpleNamespace(mission="2026_year_end", pos=(0., 0., -0.1), ground_z=-0.1, heading=0.,
                            t=0., fail=failures.append, flight=None)
    start(state)
    assert state.flight is None and "alt_m" in failures[0]


def test_missing_step_status_changes_to_failsafe():
    run = method("mission_manager.py", "MissionManager", "run_step", FAILSAFE="FAILSAFE")
    failures = []
    state = SimpleNamespace(state="PATH", step_status_time=97., state_start=90., status_timeout_s=2.,
        announce=lambda *args: None, fail_mission=failures.append,
        get_logger=lambda: SimpleNamespace(error=lambda text: None))
    run(state)
    assert failures == ["no status from the PATH node"]


OFFBOARD, POSITION = 14, 2


@pytest.mark.parametrize("failure, nav_state, expected", [
    ("pusher: no forward speed 3 s into accel1", OFFBOARD, 6),     # land where it is
    ("falling behind the route by 8 m in cruise2", OFFBOARD, 6),   # no way forward: land too
    ("timeout in hover3", OFFBOARD, 3),                             # PX4 Hold
    ("aborted: ABORT", POSITION, None),                            # the pilot has it: leave it
])
def test_failsafe_hands_the_stopped_vehicle_to_px4(failure, nav_state, expected):
    class VehicleCommand(SimpleNamespace):
        VEHICLE_CMD_DO_SET_MODE = 176
    hand_over = method("mission_manager.py", "MissionManager", "hand_over",
                       VehicleCommand=VehicleCommand, PX4_MAIN_MODE_AUTO=4,
                       PX4_SUB_MODE_AUTO_LOITER=3, PX4_SUB_MODE_AUTO_LAND=6,
                       VehicleStatus=SimpleNamespace(NAVIGATION_STATE_OFFBOARD=OFFBOARD),
                       px4=SimpleNamespace(now_us=lambda node: 0), needs_land=needs_land)
    sent = []
    state = SimpleNamespace(state_start=90., handover_s=6., handover_sent=0., failure=failure,
                            nav_state=nav_state, command_pub=SimpleNamespace(publish=sent.append),
                            get_logger=lambda: SimpleNamespace(warn=lambda text: None))
    hand_over(state)
    if expected is None:
        assert not sent
    else:
        assert sent[0].param2 == 4 and sent[0].param3 == expected

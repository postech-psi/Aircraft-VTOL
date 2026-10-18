"""Exercise actual ROS node methods without requiring ROS packages in the unit-test host."""
import ast
import math
from types import SimpleNamespace

import pytest

from psi_vtol import config
from psi_vtol.profile import Profile


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


def test_hold_target_does_not_follow_position_drift():
    hold = method("setpoint_mux.py", "SetpointMux", "hold_setpoint")
    state = SimpleNamespace(position=[0., 0., -20.], heading=.1, hold_position=None, hold_heading=None)
    first = hold(state)
    state.position, state.heading = [5., -2., -18.], .5
    second = hold(state)
    assert second.position == first.position == [0., 0., -20.]
    assert second.yaw == first.yaw == .1


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
def test_step_fails_cleanly_when_heading_into_wind_has_no_wind(filename, class_name):
    into_wind = dict(config.profile(), heading_into_wind=1)
    start = method(filename, class_name, "start", Profile=Profile,
                   config=SimpleNamespace(profile=lambda mission: into_wind))
    failures = []
    state = SimpleNamespace(mission="2026_year_end", pos=(0., 0., -0.1), ground_z=-0.1, heading=0.,
                            t=0., fail=failures.append, flight=None)
    start(state)
    assert state.flight is None and "wind" in failures[0]


def test_missing_step_status_changes_to_failsafe():
    run = method("mission_manager.py", "MissionManager", "run_step", FAILSAFE="FAILSAFE")
    transitions = []
    state = SimpleNamespace(state="PATH", step_status_time=97., state_start=90., status_timeout_s=2.,
        announce=lambda *args: None, change_state=transitions.append,
        get_logger=lambda: SimpleNamespace(error=lambda text: None))
    run(state)
    assert transitions == ["FAILSAFE"]

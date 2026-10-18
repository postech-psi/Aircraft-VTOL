"""run_case.py: what the runner does around a flight, without PX4: launching, events, values."""
import json
from types import SimpleNamespace

import pytest

from psi_vtol import config
from psi_vtol.sim import run_case

CFG = config.load()


def test_an_event_is_named_with_its_phase_and_offset():
    assert run_case.parse_event("pusher_off@cruise2+1.5") == ("pusher_off", "cruise2", 1.5)
    assert run_case.parse_event("abort@hover1") == ("abort", "hover1", 0.0)
    for bad in ("explode@cruise2+1", "abort", "abort@"):
        with pytest.raises(ValueError, match="expected one of"):
            run_case.parse_event(bad)


def test_gazebo_transport_is_unique_even_with_same_pid_and_instance(monkeypatch, tmp_path):
    environments = []
    def popen(*args, **kwargs):
        environments.append(kwargs["env"])
        kwargs["stdout"].close()
        return SimpleNamespace()
    monkeypatch.setattr(run_case.subprocess, "Popen", popen)
    for name in ("first", "second"):
        folder = tmp_path / name
        folder.mkdir()
        run_case.launch(folder, {}, {"sim": {"px4_root": "/unused"}}, 5)
    assert environments[0]["GZ_PARTITION"] != environments[1]["GZ_PARTITION"]
    assert json.loads((tmp_path / "first/runtime_env.json").read_text())["GZ_PARTITION"] == environments[0]["GZ_PARTITION"]


def test_the_pusher_is_stopped_in_this_runs_gazebo(monkeypatch):
    calls = []
    monkeypatch.setattr(run_case.subprocess, "Popen", lambda cmd, **kw: calls.append((cmd, kw["env"])))
    run_case.pusher_off(7, {"GZ_PARTITION": "psi_qp_7_x"})
    cmd, env = calls[0]
    assert cmd[cmd.index("-t") + 1] == "/model/psi_qp_7/rotor_off"
    assert "rotor_pusher_joint" in cmd[-1] and env["GZ_PARTITION"] == "psi_qp_7_x"


def test_parameter_requests_are_repeated_until_px4_answers(monkeypatch):
    """PX4 drops requests that arrive while it is still starting; MAVLink over UDP can lose any."""
    class Px4:
        target_system = target_component = 1

        def __init__(self):
            self.mav, self.reads, self.inbox = self, 0, []

        def param_request_read_send(self, system, component, key, index):
            self.reads += 1
            if self.reads > 1:                  # the first request is lost
                self.reply(key, 0.5)

        def param_set_send(self, system, component, key, value, kind):
            self.reply(key, value)

        def reply(self, key, value):
            self.inbox.append(SimpleNamespace(get_type=lambda: "PARAM_VALUE", param_id=key.decode(),
                                              param_type=9, param_value=value))   # 9: float

        def recv_match(self, blocking, timeout):
            return self.inbox.pop(0) if self.inbox else None

        def heartbeat_send(self, *args):
            pass

    ticks = iter(range(10**6))
    monkeypatch.setattr(run_case, "time", SimpleNamespace(monotonic=lambda: next(ticks) * 0.05))
    link = run_case.Link.__new__(run_case.Link)
    link.conn, link.setpoint, link.applied_params = Px4(), None, {}
    link._last_send = link._last_heartbeat = 0.0
    link.set_param("MC_YAW_P", 0.3)
    assert link.conn.reads == 2 and link.applied_params == {"MC_YAW_P": 0.3}


def test_invalid_physics_input_is_rejected_before_launch():
    with pytest.raises(ValueError, match="positive"):
        run_case.case_values(CFG, {"wing_stall_blend_deg": 0})
    with pytest.raises(ValueError, match="finite"):
        run_case.case_values(CFG, {"mass_kg": float("nan")})

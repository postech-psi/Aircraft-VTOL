"""Synthetic-log regression checks; these are not flight-dynamics results."""
from types import SimpleNamespace
import json

import numpy as np
import pytest

from psi_vtol import config, px4_setup
from psi_vtol.sim import metrics, run_case, sweep


def synthetic_log():
    t = np.arange(0, 73.01, .1)
    roll = np.zeros_like(t)
    roll[(t >= 2) & (t < 3)] = np.radians(75)
    attitude = {"timestamp": t * 1e6, "q[0]": np.cos(roll / 2), "q[1]": np.sin(roll / 2),
                "q[2]": np.zeros_like(t), "q[3]": np.zeros_like(t)}
    position = {"timestamp": t * 1e6, **{k: np.zeros_like(t) for k in ("x", "y", "vx", "vy", "vz")},
                "z": np.full_like(t, -20)}
    position["vz"][(t >= 70) & (t < 71)] = 2.  # detect a fast landing, rather than disarm velocity
    motors = {"timestamp": t * 1e6, **{f"control[{i}]": np.full_like(t, .5) for i in range(5)}}
    allocator = {"timestamp": t * 1e6, "torque_setpoint_achieved": np.ones_like(t, dtype=bool)}
    landed = {"timestamp": t * 1e6, "landed": t >= 70}
    data = [("vehicle_attitude", attitude), ("vehicle_local_position", position),
            ("actuator_motors", motors), ("control_allocator_status", allocator),
            ("vehicle_land_detected", landed)]
    flight = {"completed": True, "alt_m": 20., "cruise_mps": 10., "home": [0, 0, 0], "yaw0": 0.,
        "landing_target": [1, 0, 0],
        "legs": [{"start": [0, 0], "end": [250, 0], "heading": 0., "speed": 10.}],
        "phases": dict(zip(["climb", "hover1", "accel1", "cruise1", "decel1", "hover2", "descend", "land"],
                           [[0, 10], [10, 16], [16, 26], [26, 41], [41, 51], [51, 57], [57, 68], [68, 73]]))}
    return [SimpleNamespace(name=n, multi_id=0, data=d) for n, d in data], flight


def test_compute_catches_climb_tilt_zero_speed_and_landing_speed(monkeypatch):
    data, flight = synthetic_log()
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    result = metrics.compute("synthetic.ulg", flight)
    assert result["tilt_max_deg"] == pytest.approx(75)
    assert result["cruise_speed_err_mps"] == 10
    assert result["landed_confirmed"]
    assert result["touchdown_speed_mps"] == 2
    assert result["touchdown_err_m"] == 1
    assert result["data_gap_max_s"] < .11


def test_each_leg_is_judged_against_its_own_line_and_speed(monkeypatch):
    """Out and back: the way home is flown at -10 m/s along x, 1 m to the side of its line."""
    t = np.arange(0, 100.01, .1)
    names = ["climb", "hover1", "accel1", "cruise1", "decel1", "hover2", "turn2", "accel2", "cruise2", "decel2"]
    starts = [0, 10, 16, 26, 41, 51, 57, 68, 78, 93, 100]
    phases = {n: [a, b] for n, a, b in zip(names, starts, starts[1:])}
    vx = np.where((t >= 26) & (t <= 41), 10., 0.) + np.where((t >= 78) & (t <= 93), -10., 0.)
    position = {"timestamp": t * 1e6, "x": np.zeros_like(t), "y": np.where(t >= 68, 1., 0.),
                "z": np.full_like(t, -20.), "vx": vx, "vy": np.zeros_like(t), "vz": np.zeros_like(t)}
    attitude = {"timestamp": t * 1e6, "q[0]": np.ones_like(t), **{f"q[{i}]": np.zeros_like(t) for i in (1, 2, 3)}}
    motors = {"timestamp": t * 1e6, **{f"control[{i}]": np.full_like(t, .5) for i in range(5)}}
    data = [SimpleNamespace(name=n, multi_id=0, data=d) for n, d in
            [("vehicle_attitude", attitude), ("vehicle_local_position", position), ("actuator_motors", motors)]]
    flight = {"home": [0, 0, 0], "alt_m": 20., "phases": phases,
              "legs": [{"start": [0, 0], "end": [250, 0], "heading": 0., "speed": 10.},
                       {"start": [250, 0], "end": [0, 0], "heading": np.pi, "speed": 10.}]}
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    result = metrics.compute("synthetic.ulg", flight)
    assert result["cruise_speed_err_mps"] == pytest.approx(0, abs=1e-9)
    assert result["cross_track_max_m"] == pytest.approx(1)
    assert result["alt_err_max_m"] == 0


@pytest.mark.parametrize("contact_field", [True, False])
def test_touchdown_speed_is_the_impact_not_the_detector_delay(monkeypatch, contact_field):
    """PX4 reports contact only once the vehicle has stopped; the impact came before that."""
    data, flight = synthetic_log()
    position, landed = data[1].data, data[4].data
    t = position["timestamp"] * 1e-6
    position["vz"] = np.where((t >= 68) & (t < 70), 2.0, 0.0)  # hits the ground at 2 m/s at t=70
    landed["landed"] = t >= 71.7                               # detector delay after the impact
    if contact_field:
        landed["ground_contact"] = t >= 70.4
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    result = metrics.compute("synthetic.ulg", flight)
    assert result["touchdown_speed_mps"] == pytest.approx(2)
    assert not metrics.verdict(flight, dict(result, aero_covers_flight=True))[0]


def test_touchdown_speed_is_missing_without_the_land_detector(monkeypatch):
    data, flight = synthetic_log()
    data = [d for d in data if d.name != "vehicle_land_detected"]
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    assert "touchdown_speed_mps" not in metrics.compute("synthetic.ulg", flight)


def test_missing_flight_data_and_sustained_torque_loss_are_detected(monkeypatch):
    data, flight = synthetic_log()
    att, alloc = data[0].data, data[3].data
    keep = (att["timestamp"] < 17e6) | (att["timestamp"] > 20e6)
    data[0].data = {k: v[keep] for k, v in att.items()}
    alloc["torque_setpoint_achieved"][(alloc["timestamp"] >= 30e6) & (alloc["timestamp"] < 32e6)] = False
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    result = metrics.compute("synthetic.ulg", flight)
    assert result["data_gap_max_s"] > 3
    assert result["torque_unachieved_max_s"] == pytest.approx(2)


def test_aborted_climb_keeps_tilt_diagnostic_without_passing(monkeypatch):
    data, flight = synthetic_log()
    flight.update(completed=False, fail_reason="tumbled in climb", phases={"climb": [0, 3]})
    monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    result = metrics.compute("synthetic.ulg", flight)
    assert result["tilt_max_deg"] == pytest.approx(75)
    assert not metrics.verdict(flight, result)[0]


def test_ulog_duplicate_ticks_do_not_hide_gaps_or_time_reversal():
    data = {"timestamp": np.array([0, 100000, 100000, 200000])}
    assert metrics._data_gap(data, (0, .2)) == pytest.approx(.1)
    assert np.isnan(metrics._data_gap({"timestamp": np.array([0, 200000, 100000])}, (0, .2)))
    assert np.isnan(metrics._data_gap({"timestamp": np.array([100000, 100000])}, (0, .2)))
    assert metrics._data_gap({"timestamp": np.array([0, 0, 2000000])}, (0, 2)) == 2


def test_aero_telemetry_must_be_finite_and_monotonic(tmp_path):
    path = tmp_path / "aero.csv"
    header = "sim_time_s,airspeed_mps,wing_lift_fraction,voltage_ratio\n"
    path.write_text(header + "0,10,.2,1\n.05,11,.3,.99\n")
    assert metrics.aero_metrics(path)["wing_lift_fraction_max"] == .3
    path.write_text(header + "0,10,.2,1\n.05,nan,.3,.99\n")
    with pytest.raises(ValueError, match="non-finite"):
        metrics.aero_metrics(path)
    path.write_text(header + "0,10,.2,1\n0,10,.3,.99\n")
    with pytest.raises(ValueError, match="non-monotonic"):
        metrics.aero_metrics(path)


def test_hardware_candidate_excludes_simulation_failsafe_overrides():
    text = px4_setup.candidate()
    assert "CA_ROTOR4_AX\t1.0" in text
    assert "VT_FWD_THRUST_EN\t5\t6" in text
    for name in config._read(config.REPO / "px4/params.yaml")["sim_only"]:
        assert f"\t{name}\t" not in text


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


def test_runner_failure_never_reuses_old_success(monkeypatch, tmp_path):
    import queue
    monkeypatch.setattr(config, "REPO", tmp_path)
    output = tmp_path / "out/test" / sweep.MODEL_REVISION / "oat/nominal.json"
    output.parent.mkdir(parents=True)
    output.write_text('{"passed": true}')
    monkeypatch.setattr(sweep.subprocess, "run", lambda *a, **kw: SimpleNamespace(returncode=1, stderr="failed"))
    instances = queue.Queue()
    instances.put(5)
    result = sweep.run_one("nominal", {}, {}, "test", instances, "oat")
    assert not result["passed"] and "runner failed" in result["reasons"][0]
    assert instances.get_nowait() == 5


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
        run_case.case_values(config.load(), {"wing_stall_blend_deg": 0})
    with pytest.raises(ValueError, match="finite"):
        run_case.case_values(config.load(), {"mass_kg": float("nan")})

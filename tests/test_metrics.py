"""metrics.py: figures from a PX4 log, and the verdicts on them. The logs here are synthetic."""
from types import SimpleNamespace

import numpy as np
import pytest

from psi_vtol.sim import metrics


def level(t):
    """Attitude topic of a vehicle that stays level."""
    return {"timestamp": t * 1e6, "q[0]": np.ones_like(t), **{f"q[{i}]": np.zeros_like(t) for i in (1, 2, 3)}}


def motors_at_half(t):
    return {"timestamp": t * 1e6, **{f"control[{i}]": np.full_like(t, .5) for i in range(5)}}


def synthetic_log():
    """A 73 s straight-leg flight: a 75 deg roll in the climb, a 2 m/s touchdown, never moving."""
    t = np.arange(0, 73.01, .1)
    roll = np.zeros_like(t)
    roll[(t >= 2) & (t < 3)] = np.radians(75)
    attitude = dict(level(t), **{"q[0]": np.cos(roll / 2), "q[1]": np.sin(roll / 2)})
    position = {"timestamp": t * 1e6, **{k: np.zeros_like(t) for k in ("x", "y", "vx", "vy", "vz")},
                "z": np.full_like(t, -20)}
    position["vz"][(t >= 70) & (t < 71)] = 2.  # detect a fast landing, rather than disarm velocity
    allocator = {"timestamp": t * 1e6, "torque_setpoint_achieved": np.ones_like(t, dtype=bool)}
    landed = {"timestamp": t * 1e6, "landed": t >= 70}
    data = [("vehicle_attitude", attitude), ("vehicle_local_position", position),
            ("actuator_motors", motors_at_half(t)), ("control_allocator_status", allocator),
            ("vehicle_land_detected", landed)]
    flight = {"completed": True, "alt_m": 20., "cruise_mps": 10., "home": [0, 0, 0], "yaw0": 0.,
        "landing_target": [1, 0, 0],
        "legs": [{"start": [0, 0], "end": [250, 0], "heading": 0., "speed": 10.}],
        "phases": dict(zip(["climb", "hover1", "accel1", "cruise1", "decel1", "hover2", "descend", "land"],
                           [[0, 10], [10, 16], [16, 26], [26, 41], [41, 51], [51, 57], [57, 68], [68, 73]]))}
    return [SimpleNamespace(name=n, multi_id=0, data=d) for n, d in data], flight


@pytest.fixture
def log(monkeypatch):
    """Serve synthetic topics to metrics.compute in place of a ULog file."""
    def serve(data):
        monkeypatch.setattr(metrics, "ULog", lambda *a, **kw: SimpleNamespace(data_list=data))
    return serve


def within_limits(**changes):
    """Figures of a flight that passes every check, with some changed."""
    return {**{name: 0.0 for name in metrics.LIMITS}, "landed_confirmed": True,
            "aero_covers_flight": True, **changes}


def test_compute_catches_climb_tilt_zero_speed_and_landing_speed(log):
    data, flight = synthetic_log()
    log(data)
    result = metrics.compute("synthetic.ulg", flight)
    assert result["tilt_max_deg"] == pytest.approx(75)
    assert result["cruise_speed_err_mps"] == 10
    assert result["landed_confirmed"]
    assert result["touchdown_speed_mps"] == 2
    assert result["touchdown_err_m"] == 1
    assert result["data_gap_max_s"] < .11


def test_each_leg_is_judged_against_its_own_line_and_speed(log):
    """Out and back: the way home is flown at -10 m/s along x, 1 m to the side of its line."""
    t = np.arange(0, 100.01, .1)
    names = ["climb", "hover1", "accel1", "cruise1", "decel1", "hover2", "turn2", "accel2", "cruise2", "decel2"]
    starts = [0, 10, 16, 26, 41, 51, 57, 68, 78, 93, 100]
    phases = {n: [a, b] for n, a, b in zip(names, starts, starts[1:])}
    vx = np.where((t >= 26) & (t <= 41), 10., 0.) + np.where((t >= 78) & (t <= 93), -10., 0.)
    position = {"timestamp": t * 1e6, "x": np.zeros_like(t), "y": np.where(t >= 68, 1., 0.),
                "z": np.full_like(t, -20.), "vx": vx, "vy": np.zeros_like(t), "vz": np.zeros_like(t)}
    attitude, motors = level(t), motors_at_half(t)
    data = [SimpleNamespace(name=n, multi_id=0, data=d) for n, d in
            [("vehicle_attitude", attitude), ("vehicle_local_position", position), ("actuator_motors", motors)]]
    flight = {"home": [0, 0, 0], "alt_m": 20., "phases": phases,
              "legs": [{"start": [0, 0], "end": [250, 0], "heading": 0., "speed": 10.},
                       {"start": [250, 0], "end": [0, 0], "heading": np.pi, "speed": 10.}]}
    log(data)
    result = metrics.compute("synthetic.ulg", flight)
    assert result["cruise_speed_err_mps"] == pytest.approx(0, abs=1e-9)
    assert result["cross_track_max_m"] == pytest.approx(1)
    assert result["alt_err_max_m"] == 0


@pytest.mark.parametrize("contact_field", [True, False])
def test_touchdown_speed_is_the_impact_not_the_detector_delay(log, contact_field):
    """PX4 reports contact only once the vehicle has stopped; the impact came before that."""
    data, flight = synthetic_log()
    position, landed = data[1].data, data[4].data
    t = position["timestamp"] * 1e-6
    position["vz"] = np.where((t >= 68) & (t < 70), 2.0, 0.0)  # hits the ground at 2 m/s at t=70
    landed["landed"] = t >= 71.7                               # detector delay after the impact
    if contact_field:
        landed["ground_contact"] = t >= 70.4
    log(data)
    result = metrics.compute("synthetic.ulg", flight)
    assert result["touchdown_speed_mps"] == pytest.approx(2)
    assert not metrics.verdict(flight, dict(result, aero_covers_flight=True))[0]


def test_touchdown_speed_is_missing_without_the_land_detector(log):
    data, flight = synthetic_log()
    data = [d for d in data if d.name != "vehicle_land_detected"]
    log(data)
    assert "touchdown_speed_mps" not in metrics.compute("synthetic.ulg", flight)


def test_missing_flight_data_and_sustained_torque_loss_are_detected(log):
    data, flight = synthetic_log()
    att, alloc = data[0].data, data[3].data
    keep = (att["timestamp"] < 17e6) | (att["timestamp"] > 20e6)
    data[0].data = {k: v[keep] for k, v in att.items()}
    alloc["torque_setpoint_achieved"][(alloc["timestamp"] >= 30e6) & (alloc["timestamp"] < 32e6)] = False
    log(data)
    result = metrics.compute("synthetic.ulg", flight)
    assert result["data_gap_max_s"] > 3
    assert result["torque_unachieved_max_s"] == pytest.approx(2)


def test_a_flight_cut_short_keeps_the_figures_of_the_phases_it_flew(log):
    data, flight = synthetic_log()
    flight.update(completed=False, fail_reason="tumbled in climb", phases={"climb": [0, 3]})
    log(data)
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


def test_verdict_needs_a_complete_flight_inside_every_limit():
    good = within_limits()
    assert metrics.verdict({"completed": True}, good) == (True, [])
    assert not metrics.verdict({"completed": False, "fail_reason": "tumbled in cruise"}, good)[0]
    assert not metrics.verdict({"completed": True}, within_limits(tilt_max_deg=31.0))[0]
    assert not metrics.verdict({"completed": True}, within_limits(landed_confirmed=False))[0]
    assert not metrics.verdict({"completed": True}, {})[0]      # a missing figure is not a pass


def test_a_plan_without_a_cruise_is_not_judged_on_its_cruise_speed():
    """The 50 m square flown through at 6 m/s or more only speeds up and slows down."""
    no_speed = {k: v for k, v in within_limits().items() if k != "cruise_speed_err_mps"}
    no_cruise = {"completed": True, "phases": {"accel1": [0, 5], "decel1": [5, 10]}}
    assert metrics.verdict(no_cruise, no_speed)[0]
    cruise = {"completed": True, "phases": {"accel1": [0, 5], "cruise1": [5, 8]}}
    assert not metrics.verdict(cruise, no_speed)[0]


@pytest.mark.parametrize("name", list(metrics.LIMITS))
def test_nonfinite_metrics_never_pass(name):
    assert not metrics.verdict({"completed": True}, within_limits(**{name: float("nan")}))[0]


def test_a_failsafe_is_safe_when_it_ends_on_the_ground_unharmed():
    down = {"landed_confirmed": True, "tilt_max_deg": 12.0, "touchdown_speed_mps": 0.5}
    assert metrics.outcome({"fail_reason": "mission aborted"}, down) == (True, [])
    assert not metrics.outcome({"fail_reason": "mission aborted; tumbled"}, down)[0]
    assert not metrics.outcome({"fail_reason": None}, dict(down, landed_confirmed=False))[0]
    assert not metrics.outcome({"fail_reason": None}, dict(down, touchdown_speed_mps=1.5))[0]

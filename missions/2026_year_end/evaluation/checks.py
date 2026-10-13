"""Run current analytical/synthetic checks; results.json preserves the pre-fix audit.

Run from any directory with the project's Python dependencies installed. These checks
audit assumptions and acceptance logic; they do not simulate aircraft dynamics.
"""
import ast
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from psi_vtol import config  # noqa: E402
from psi_vtol.profile import Profile  # noqa: E402
from psi_vtol.sim import gz_model, metrics, sweep  # noqa: E402


def synthetic_log_check(p):
    """A synthetic log with excessive tilt only during climb and landing."""
    t = np.array([5, 12, 20, 39, 45, 54, 62, 70], dtype=float) * 1e6
    roll = np.radians([75, 0, 0, 0, 0, 0, 0, 75])
    att = {"timestamp": t, "q[0]": np.cos(roll / 2), "q[1]": np.sin(roll / 2),
           "q[2]": np.zeros(8), "q[3]": np.zeros(8)}
    pos = {"timestamp": t, "x": np.zeros(8), "y": np.zeros(8),
           "z": np.full(8, -p["alt_m"]), "vx": np.zeros(8), "vy": np.zeros(8)}
    mot = {"timestamp": t, **{f"control[{i}]": np.full(8, .5) for i in range(4)},
           "control[4]": np.zeros(8)}
    alloc = {"timestamp": t, "torque_setpoint_achieved": np.ones(8)}
    datasets = [SimpleNamespace(name=n, multi_id=0, data=v) for n, v in
                [("vehicle_attitude", att), ("vehicle_local_position", pos),
                 ("actuator_motors", mot), ("control_allocator_status", alloc)]]
    flight = {"completed": True, "home": [0, 0, 0], "yaw0": 0, "alt_m": p["alt_m"],
              "cruise_mps": p["cruise_mps"],
              "phases": dict(zip(
                  ["climb", "hover1", "accel", "cruise", "decel", "hover2", "descend", "land"],
                  [[0, 10], [10, 16], [16, 26], [26, 41], [41, 51], [51, 57], [57, 68], [68, 73]]))}
    with patch.object(metrics, "ULog", return_value=SimpleNamespace(data_list=datasets)):
        figures = metrics.compute("synthetic-no-file.ulg", flight)
    return {"actual_full_log_tilt_deg": 75, "reported_tilt_max_deg": figures["tilt_max_deg"],
            "reported_cruise_speed_mps": figures["cruise_speed_mps"],
            "passed": metrics.verdict(flight, figures)[0]}


def hold_check():
    """Execute the actual hold method without needing a ROS installation."""
    path = ROOT / "ros2/ws/src/vtol_mission/vtol_mission/setpoint_mux.py"
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "SetpointMux")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "hold_setpoint")
    namespace = {"TrajectorySetpoint": SimpleNamespace, "NANS": [math.nan] * 3, "math": math}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
    state = SimpleNamespace(position=[0., 0., -20.], heading=0., hold_position=None, hold_heading=None)
    first = list(namespace["hold_setpoint"](state).position)
    state.position = [5., 0., -20.]
    second = list(namespace["hold_setpoint"](state).position)
    return {"first_target_m": first, "target_after_5m_drift_m": second,
            "target_is_latched": first == second}


def main():
    cfg = config.load()
    # The polar/drag audit below intentionally keeps the LEGACY coefficients.
    # Current all-angle production physics is checked by tests/aero_physics_test.cc.
    p = dict(cfg["nominal"], wing_cd0=.08, wing_cp_x_m=-.08)
    d = gz_model.derived(p)
    weight, area, rho = d["weight_n"], d["wing_area_m2"], p["air_density"]
    envelope = []
    level_speed_limits = []
    for incidence in (0, 4):
        wing = gz_model.derived(dict(p, wing_incidence_deg=incidence))
        level_speed_limits.append({"incidence_deg": incidence,
                                   "airspeed_at_40pct_lift_mps": math.sqrt(
                                       .4 * weight / (.5 * rho * area * wing["cl_level"]))})
        for speed in (10, 13, 18):
            for pitch in (0, 6.4):
                cl = wing["wing_cla"] * (wing["wing_alpha0_rad"] + math.radians(pitch))
                share = .5 * rho * speed ** 2 * area * cl / weight
                envelope.append({"incidence_deg": incidence, "airspeed_mps": speed,
                                 "pitch_deg": pitch, "wing_lift_fraction": share})
    authority = []
    for share in (0, .4, .59, .8):
        total = (1 - share) * weight
        difference = min(total, 4 * p["lift_thrust_max_n"] - total)
        authority.append({"wing_lift_fraction": share,
                          "ideal_yaw_bound_nm": p["lift_moment_m"] * difference})
    drag = []
    for speed in (10, 18):
        lift = .5 * rho * speed ** 2 * area * d["cl_level"]
        command = (weight - lift) / (4 * p["lift_thrust_max_n"])
        omega = gz_model.LIFT_MAX_RADPS * math.sqrt(command)
        rotor_drag = 4 * omega * .000106428 * speed
        wing_drag = .5 * rho * speed ** 2 * area * p["wing_cd0"]
        drag.append({"airspeed_mps": speed, "lift_rotor_radps": omega,
                     "lift_rotor_drag_n": rotor_drag, "wing_drag_n": wing_drag,
                     "approx_pusher_command": (rotor_drag + wing_drag) / p["pusher_thrust_max_n"]})
    polar = []
    for pitch in (-4, 0, 6.4):
        alpha = d["wing_alpha0_rad"] + math.radians(pitch)
        cl = d["wing_cla"] * alpha
        polar.append({"pitch_deg": pitch, "cl": cl,
                      "cd": abs(p["wing_cd0"] / d["wing_alpha0_rad"] * alpha),
                      "cm_from_lift_offset_only": p["wing_cp_x_m"] / p["wing_chord_m"] * cl})
    good = {name: 0. for name in metrics.LIMITS}
    poor = dict(good, cruise_speed_mps=0, cross_track_max_m=100,
                fwd_yaw_err_max_deg=180, pusher_sat_frac=1.)
    flight = Profile(p, (0, 0, 0), 0)
    for t in range(2000):
        pos = (0, 0, 0 if flight.phase in ("descend", "land") else -p["alt_m"])
        flight.update(t * .1, pos, (0, 0, 0), armed=flight.phase != "land")
        if flight.done:
            break
    # Wilson interval describes only hypothetical iid draws from the chosen toy distribution.
    n, successes, z = 40, 38, 1.959963984540054
    fraction = successes / n
    center = (fraction + z * z / (2 * n)) / (1 + z * z / n)
    delta = z * math.sqrt(fraction * (1 - fraction) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    print(json.dumps({
        "scope": "legacy analytical assumptions plus current software checks; no SITL or flight",
        "nominal": d, "pitch_envelope": envelope, "level_speed_at_40pct_lift": level_speed_limits,
        "isolated_axis_yaw_bounds": authority, "approx_level_drag": drag, "prestall_polar": polar,
        "ideal_stopping_distance_m": p["cruise_mps"] ** 2 / (2 * p["accel_mps2"]),
        "acceptance_checks": {
            "all_nan_metrics_pass": metrics.verdict({"completed": True}, {k: math.nan for k in good})[0],
            "poor_tracking_pass": metrics.verdict({"completed": True, "touchdown_err_m": 100}, poor)[0],
            "synthetic_log": synthetic_log_check(p),
            "profile_completes_without_horizontal_motion": flight.done,
            "hold": hold_check()},
        "sweep": {"dimensions": len(cfg["sweep"]), "oat_cases_including_nominal": len(sweep.oat_cases(cfg)),
                  "wind_cases": len(sweep.wind_cases()), "reported_random_successes": successes,
                  "reported_random_trials": n, "wilson_95_interval": [center - delta, center + delta]},
    }, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()

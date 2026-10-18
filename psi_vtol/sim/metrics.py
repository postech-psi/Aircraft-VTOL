"""Figures of one run, read from the PX4 log (ULog), and the pass/fail verdict on them."""
import math
import csv

import numpy as np
from pyulog import ULog

# A run passes when it completes and stays inside all of these.
LIMITS = {
    "tilt_max_deg": 30.0,           # largest roll or pitch while airborne
    "alt_err_max_m": 3.0,           # largest height error between take-off and descent
    "lift_sat_frac": 0.05,          # share of forward flight with a lift motor at its maximum
    "torque_unachieved_frac": 0.10,
    "lift_low_frac": 0.05,         # any lift rotor near its minimum during forward flight
    "pusher_sat_frac": 0.10,
    "fwd_yaw_err_max_deg": 20.0,
    "cross_track_max_m": 3.0,
    "cruise_speed_err_mps": 1.0,
    "touchdown_err_m": 2.0,
    "touchdown_speed_mps": 1.0,
    "torque_unachieved_max_s": 1.0,
    "data_gap_max_s": 1.0,        # missing primary ULog data cannot hide a bad maneuver
    "aero_gap_max_s": 0.5,
    "wing_lift_fraction_max": 0.40,  # initial stock-controller envelope, not a universal limit
}

# PX4's land detector reports contact only after the vehicle has stopped moving, and landed
# later still. The touchdown speed is the fastest the vehicle moved in this long before that.
TOUCHDOWN_LOOKBACK_S = 3.0

TOPICS = ["vehicle_attitude", "vehicle_attitude_setpoint", "vehicle_local_position",
          "actuator_motors", "rate_ctrl_status", "control_allocator_status", "airspeed_validated",
          "vehicle_land_detected"]


def _longest_interval(timestamps, flags):
    """Duration including the following sample interval; assumes monotonic ULog timestamps."""
    t = timestamps * 1e-6
    if len(t) < 2:
        return math.nan
    duration = np.diff(t, append=t[-1] + np.median(np.diff(t)))
    longest = current = 0.0
    for dt, flagged in zip(duration, flags):
        current = current + dt if flagged else 0.0
        longest = max(longest, current)
    return float(longest)


def _topic(ulog, name, multi_id=0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == multi_id:
            return d.data
    return None


def _window(data, span):
    t = data["timestamp"] * 1e-6
    return (t >= span[0]) & (t <= span[1])


def _euler_deg(att, field="q"):
    w, x, y, z = (att[f"{field}[{i}]"] for i in range(4))
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _data_gap(data, span):
    t = data["timestamp"][_window(data, span)] * 1e-6
    # uORB can be logged twice at the same simulator tick. Repeated timestamps
    # add no time coverage; a backward jump remains invalid.
    if len(t) < 2 or np.any(np.diff(t) < 0):
        return math.nan
    t = np.unique(t)
    if len(t) < 2:
        return math.nan
    return float(max(t[0] - span[0], span[1] - t[-1], np.diff(t).max()))


def aero_metrics(path):
    """Independent simulator truth. Absence of this file must fail the run."""
    with open(path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < 2:
        raise ValueError("aerodynamic telemetry needs at least two samples")
    values = np.array([[float(r[k]) for k in r] for r in rows])
    if not np.isfinite(values).all():
        raise ValueError("non-finite aerodynamic telemetry")
    t = np.array([float(r["sim_time_s"]) for r in rows])
    if np.any(np.diff(t) <= 0):
        raise ValueError("non-monotonic aerodynamic telemetry")
    return {"aero_duration_s": float(t[-1] - t[0]), "aero_samples": len(rows),
            "aero_gap_max_s": float(np.diff(t).max()),
            "wing_lift_fraction_max": max(float(r["wing_lift_fraction"]) for r in rows),
            "true_airspeed_max_mps": max(float(r["airspeed_mps"]) for r in rows),
            "voltage_ratio_min": min(float(r["voltage_ratio"]) for r in rows)}


def compute(ulog_path, flight):
    """flight: what run_case.fly() returned (phases in seconds since PX4 boot, home, yaw0)."""
    ulog = ULog(str(ulog_path), message_name_filter_list=TOPICS)
    ph = flight["phases"]
    out = {}

    def span(first, last):
        if first in ph and last in ph:
            return (ph[first][0], ph[last][1])
        return None

    # Keep diagnostic metrics for an aborted flight; completion/landing remain mandatory.
    def available_span(first, last_names):
        last = next((name for name in last_names if name in ph), None)
        return span(first, last) if first in ph and last is not None else None

    airborne = available_span("climb", ("land", "descend", "hover2", "decel", "cruise", "accel", "hover1", "climb"))
    level = available_span("hover1", ("hover2", "decel", "cruise", "accel", "hover1"))
    forward = available_span("accel", ("decel", "cruise", "accel"))
    cruise = ph.get("cruise")

    att = _topic(ulog, "vehicle_attitude")
    pos = _topic(ulog, "vehicle_local_position")
    mot = _topic(ulog, "actuator_motors")
    if att is None or pos is None or mot is None or airborne is None:
        return out
    roll, pitch, yaw = _euler_deg(att)
    out["data_gap_max_s"] = float(np.max([_data_gap(d, airborne) for d in (att, pos, mot)]))

    m = _window(att, airborne)
    if m.any():
        out["tilt_max_deg"] = float(np.abs(np.stack([roll[m], pitch[m]])).max())
    m = _window(pos, level) if level else np.zeros(len(pos["timestamp"]), dtype=bool)
    if m.any():
        out["alt_err_max_m"] = float(np.abs(pos["z"][m] - (flight["home"][2] - flight["alt_m"])).max())

    lift = np.stack([mot[f"control[{i}]"] for i in range(4)])
    pusher = np.nan_to_num(mot["control[4]"])      # PX4 logs a stopped motor as NaN
    m = _window(mot, ph["hover1"]) if "hover1" in ph else np.zeros(len(mot["timestamp"]), dtype=bool)
    if m.any():
        out["hover_lift_cmd"] = float(lift[:, m].mean())

    if forward:
        m = _window(att, forward)
        if m.any():
            out["fwd_roll_max_deg"] = float(np.abs(roll[m]).max())
            out["fwd_pitch_max_deg"] = float(np.abs(pitch[m]).max())
            # heading error against the heading PX4 was asked to hold at that moment
            asked = _topic(ulog, "vehicle_attitude_setpoint")
            if asked is not None:
                yaw_sp = np.unwrap(np.radians(_euler_deg(asked, "q_d")[2]))
                yaw_sp = np.degrees(np.interp(att["timestamp"][m], asked["timestamp"], yaw_sp))
                out["fwd_yaw_err_max_deg"] = float(np.abs(_wrap_deg(yaw[m] - yaw_sp)).max())
        m = _window(pos, forward)
        if m.any():
            # distance to the left or right of the line flown, from home along the first heading
            dn, de = pos["x"][m] - flight["home"][0], pos["y"][m] - flight["home"][1]
            cross = -dn * math.sin(flight["yaw0"]) + de * math.cos(flight["yaw0"])
            out["cross_track_max_m"] = float(np.abs(cross).max())
        m = _window(mot, forward)
        if m.any():
            out["pusher_max_cmd"] = float(pusher[m].max())
            out["pusher_sat_frac"] = float((pusher[m] >= 0.88).mean())
            out["lift_max_cmd"] = float(lift[:, m].max())
            out["lift_min_cmd"] = float(lift[:, m].min())
            out["lift_sat_frac"] = float((lift[:, m].max(axis=0) >= 0.98).mean())
            out["lift_low_frac"] = float((lift[:, m].min(axis=0) <= 0.05).mean())
        rate = _topic(ulog, "rate_ctrl_status")
        if rate is not None:
            m = _window(rate, forward)
            if m.any():
                for axis in ("roll", "pitch", "yaw"):
                    out[f"{axis}_integ_max"] = float(np.abs(rate[f"{axis}speed_integ"][m]).max())
        alloc = _topic(ulog, "control_allocator_status")
        if alloc is not None:
            m = _window(alloc, forward)
            if m.any():
                out["torque_unachieved_frac"] = float(1.0 - alloc["torque_setpoint_achieved"][m].mean())
                out["torque_unachieved_max_s"] = _longest_interval(
                    alloc["timestamp"][m], ~alloc["torque_setpoint_achieved"][m].astype(bool))

    if cruise:
        late = (max(cruise[0], cruise[1] - 5.0), cruise[1])     # after the speed has settled
        m = _window(pos, late)
        if m.any():
            ground = np.hypot(pos["vx"][m], pos["vy"][m])
            out["cruise_speed_mps"] = float(ground.mean())
            along = pos["vx"][m] * math.cos(flight["yaw0"]) + pos["vy"][m] * math.sin(flight["yaw0"])
            out["cruise_speed_err_mps"] = float(np.abs(along - flight["cruise_mps"]).max())
        m = _window(att, late)
        if m.any():
            out["cruise_pitch_deg"] = float(pitch[m].mean())
        m = _window(mot, late)
        if m.any():
            out["cruise_pusher_cmd"] = float(pusher[m].mean())
            out["cruise_lift_cmd"] = float(lift[:, m].mean())
        air = _topic(ulog, "airspeed_validated")
        if air is not None:
            m = _window(air, late)
            if m.any():
                out["cruise_airspeed_mps"] = float(np.nanmean(air["true_airspeed_m_s"][m]))
    if "touchdown_err_m" in flight:
        out["touchdown_err_m"] = flight["touchdown_err_m"]
    landed = _topic(ulog, "vehicle_land_detected")
    if landed is not None and "land" in ph:
        m = _window(landed, ph["land"])
        out["landed_confirmed"] = bool(m.any() and landed["landed"][m][-1])
        flags = [landed[f].astype(bool) for f in ("ground_contact", "maybe_landed", "landed") if f in landed]
        contact = m & np.logical_or.reduce(flags)
        if contact.any():
            touchdown_t = landed["timestamp"][contact][0]
            i = int(np.argmin(np.abs(pos["timestamp"].astype(float) - touchdown_t)))
            before = ((pos["timestamp"] >= touchdown_t - TOUCHDOWN_LOOKBACK_S * 1e6)
                      & (pos["timestamp"] <= touchdown_t))
            if "vz" in pos and before.any():
                speed = np.sqrt(pos["vx"][before] ** 2 + pos["vy"][before] ** 2 + pos["vz"][before] ** 2)
                out["touchdown_speed_mps"] = float(speed.max())
            if "landing_target" in flight:
                out["touchdown_err_m"] = float(math.hypot(pos["x"][i] - flight["landing_target"][0],
                                                         pos["y"][i] - flight["landing_target"][1]))
    return out


def verdict(flight, figures):
    """(passed, reasons)."""
    reasons = []
    if not flight["completed"]:
        reasons.append(flight["fail_reason"] or "not completed")
    if not figures.get("landed_confirmed", False):
        reasons.append("touchdown not confirmed by land detector")
    if not figures.get("aero_covers_flight", False):
        reasons.append("aerodynamic telemetry missing or incomplete")
    for name, limit in LIMITS.items():
        if name not in figures:
            reasons.append(f"{name} missing")
        elif not math.isfinite(figures[name]):
            reasons.append(f"{name} non-finite")
        elif figures[name] > limit:
            reasons.append(f"{name} {figures[name]:.2f} > {limit}")
    return (not reasons, reasons)

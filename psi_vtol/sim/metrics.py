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


def _distance_to_path(north, east, path):
    """Distance from each point to the nearest of the path's segments; path: (n, 2) array."""
    a, b = path[:-1], path[1:]
    d = b - a
    length2 = np.maximum((d ** 2).sum(axis=1), 1e-12)
    p = np.stack([north, east], axis=1)[:, None, :]
    u = np.clip(((p - a) * d).sum(axis=2) / length2, 0.0, 1.0)
    nearest = a + u[..., None] * d
    return np.sqrt(((p - nearest) ** 2).sum(axis=2)).min(axis=1)


def _mask(data, spans):
    m = np.zeros(len(data["timestamp"]), dtype=bool)
    for span in spans:
        m |= _window(data, span)
    return m


def compute(ulog_path, flight):
    """flight: what run_case.fly() returned (phases in seconds since PX4 boot, home, legs)."""
    ulog = ULog(str(ulog_path), message_name_filter_list=TOPICS)
    ph = flight["phases"]
    legs = flight.get("legs", [])
    out = {}

    # Spans of the phases flown, also of an aborted flight: its diagnostics are kept, and the
    # missing completion and landing still fail it.
    airborne = (ph["climb"][0], max(end for _, end in ph.values())) if "climb" in ph else None
    route = [ph[name][1] for name in ph if name not in ("climb", "descend", "land", "stop", "px4")]
    level = (ph["hover1"][0], max(route)) if "hover1" in ph else None
    forward = {}            # leg number -> from the first to the last moment it was flown forward
    for name, (start, end) in ph.items():
        kind = name.rstrip("0123456789")
        if kind in ("accel", "cruise", "decel", "corner") and name[len(kind):].isdigit():
            i = int(name[len(kind):])         # corner<i> is the turn from leg i to leg i+1
            first, last = forward.get(i, (start, end))
            forward[i] = (min(first, start), max(last, end))
    late = {i: (max(ph[f"cruise{i}"][0], ph[f"cruise{i}"][1] - 5.0), ph[f"cruise{i}"][1])
            for i in forward if f"cruise{i}" in ph}         # after the speed has settled

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
        m = _mask(att, forward.values())
        if m.any():
            out["fwd_roll_max_deg"] = float(np.abs(roll[m]).max())
            out["fwd_pitch_max_deg"] = float(np.abs(pitch[m]).max())
            # heading error against the heading PX4 was asked to hold at that moment
            asked = _topic(ulog, "vehicle_attitude_setpoint")
            if asked is not None:
                yaw_sp = np.unwrap(np.radians(_euler_deg(asked, "q_d")[2]))
                yaw_sp = np.degrees(np.interp(att["timestamp"][m], asked["timestamp"], yaw_sp))
                out["fwd_yaw_err_max_deg"] = float(np.abs(_wrap_deg(yaw[m] - yaw_sp)).max())
        cross = []          # distance from the path planned, corners included
        if flight.get("path"):
            m = _mask(pos, forward.values())
            if m.any():
                cross.append(_distance_to_path(pos["x"][m], pos["y"][m], np.asarray(flight["path"])).max())
        else:               # older records: to the left or right of each leg's line
            for i, span in forward.items():
                m = _window(pos, span)
                if m.any() and i <= len(legs):
                    leg = legs[i - 1]
                    dn, de = pos["x"][m] - leg["start"][0], pos["y"][m] - leg["start"][1]
                    cross.append(np.abs(-dn * math.sin(leg["heading"]) + de * math.cos(leg["heading"])).max())
        if cross:
            out["cross_track_max_m"] = float(max(cross))
        m = _mask(mot, forward.values())
        if m.any():
            out["pusher_max_cmd"] = float(pusher[m].max())
            out["pusher_sat_frac"] = float((pusher[m] >= 0.88).mean())
            out["lift_max_cmd"] = float(lift[:, m].max())
            out["lift_min_cmd"] = float(lift[:, m].min())
            out["lift_sat_frac"] = float((lift[:, m].max(axis=0) >= 0.98).mean())
            out["lift_low_frac"] = float((lift[:, m].min(axis=0) <= 0.05).mean())
        rate = _topic(ulog, "rate_ctrl_status")
        if rate is not None:
            m = _mask(rate, forward.values())
            if m.any():
                for axis in ("roll", "pitch", "yaw"):
                    out[f"{axis}_integ_max"] = float(np.abs(rate[f"{axis}speed_integ"][m]).max())
        alloc = _topic(ulog, "control_allocator_status")
        if alloc is not None:
            m = _mask(alloc, forward.values())
            if m.any():
                out["torque_unachieved_frac"] = float(1.0 - alloc["torque_setpoint_achieved"][m].mean())
                out["torque_unachieved_max_s"] = max(
                    _longest_interval(alloc["timestamp"][w], ~alloc["torque_setpoint_achieved"][w].astype(bool))
                    for w in (_window(alloc, span) for span in forward.values()) if w.any())

    if late:
        m = _mask(pos, late.values())
        if m.any():
            out["cruise_speed_mps"] = float(np.hypot(pos["vx"][m], pos["vy"][m]).mean())
        errors = []         # speed along each leg against the speed asked for on it
        for i, span in late.items():
            m = _window(pos, span)
            if m.any() and i <= len(legs):
                leg = legs[i - 1]
                along = pos["vx"][m] * math.cos(leg["heading"]) + pos["vy"][m] * math.sin(leg["heading"])
                errors.append(np.abs(along - leg["speed"]).max())
        if errors:
            out["cruise_speed_err_mps"] = float(max(errors))
        m = _mask(att, late.values())
        if m.any():
            out["cruise_pitch_deg"] = float(pitch[m].mean())
        m = _mask(mot, late.values())
        if m.any():
            out["cruise_pusher_cmd"] = float(pusher[m].mean())
            out["cruise_lift_cmd"] = float(lift[:, m].mean())
        air = _topic(ulog, "airspeed_validated")
        if air is not None:
            m = _mask(air, late.values())
            if m.any():
                out["cruise_airspeed_mps"] = float(np.nanmean(air["true_airspeed_m_s"][m]))
    # a failsafe: the stop, and PX4 flying on its own until it is down
    if "stop" in ph:
        m = _window(att, ph["stop"])
        if m.any():
            out["stop_tilt_max_deg"] = float(np.abs(np.stack([roll[m], pitch[m]])).max())
        m = _window(pos, ph["stop"])
        if m.any():
            i, j = np.flatnonzero(m)[[0, -1]]
            out["stop_distance_m"] = float(math.hypot(pos["x"][j] - pos["x"][i], pos["y"][j] - pos["y"][i]))
    if "px4" in ph:
        m = _window(att, ph["px4"])
        if m.any():
            out["px4_tilt_max_deg"] = float(np.abs(np.stack([roll[m], pitch[m]])).max())
    if "lag_max_m" in flight:
        out["lag_max_m"] = flight["lag_max_m"]
    if "touchdown_err_m" in flight:
        out["touchdown_err_m"] = flight["touchdown_err_m"]
    landed = _topic(ulog, "vehicle_land_detected")
    landing = ph.get("land") or ph.get("px4")
    if landed is not None and landing:
        m = _window(landed, landing)
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


def outcome(flight, figures):
    """(safe, reasons): down on the ground unharmed, whether or not the mission was flown.

    This is what an injected event is judged by: the mission cannot pass, but the failsafe can.
    """
    reasons = []
    if "tumbled" in (flight["fail_reason"] or ""):
        reasons.append(flight["fail_reason"])
    if not figures.get("landed_confirmed", False):
        reasons.append("touchdown not confirmed by land detector")
    for name in ("tilt_max_deg", "touchdown_speed_mps"):
        if not math.isfinite(figures.get(name, math.nan)):
            reasons.append(f"{name} missing")
        elif figures[name] > LIMITS[name]:
            reasons.append(f"{name} {figures[name]:.2f} > {LIMITS[name]}")
    return (not reasons, reasons)


def verdict(flight, figures):
    """(passed, reasons)."""
    reasons = []
    if not flight["completed"]:
        reasons.append(flight["fail_reason"] or "not completed")
    if not figures.get("landed_confirmed", False):
        reasons.append("touchdown not confirmed by land detector")
    if not figures.get("aero_covers_flight", False):
        reasons.append("aerodynamic telemetry missing or incomplete")
    # a plan that only speeds up and slows down between corners has no cruise to judge
    cruise = any(n.startswith("cruise") for n in flight.get("phases", {})) or not flight.get("phases")
    for name, limit in LIMITS.items():
        if name == "cruise_speed_err_mps" and not cruise:
            continue
        if name not in figures:
            reasons.append(f"{name} missing")
        elif not math.isfinite(figures[name]):
            reasons.append(f"{name} non-finite")
        elif figures[name] > limit:
            reasons.append(f"{name} {figures[name]:.2f} > {limit}")
    return (not reasons, reasons)

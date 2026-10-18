#!/usr/bin/env python3
"""Run many simulation cases and tabulate them.

    python3 -m psi_vtol.sim.sweep oat             # each swept value at its low and high end, alone
    python3 -m psi_vtol.sim.sweep wind            # steady wind by speed and direction
    python3 -m psi_vtol.sim.sweep scenarios       # speeds, both route modes, wind, failures

Ranges come from missions/<mission>/study.yaml (sweep). Results go to out/<mission>/<study>.csv
and .json; each case's files and PX4 log stay under the run folder (PSI_RUN_ROOT).
"""
import argparse
import csv
import fnmatch
import json
import queue
import subprocess
import sys
import uuid
from concurrent.futures import ThreadPoolExecutor

from psi_vtol import config
from psi_vtol.sim.gz_model import MODEL_REVISION

COLUMNS = ["tilt_max_deg", "alt_err_max_m", "fwd_roll_max_deg", "fwd_pitch_max_deg",
           "fwd_yaw_err_max_deg", "cross_track_max_m", "cruise_speed_mps", "cruise_pusher_cmd",
           "pusher_max_cmd", "pusher_sat_frac", "hover_lift_cmd", "cruise_lift_cmd",
           "lift_max_cmd", "lift_min_cmd", "lift_sat_frac", "roll_integ_max", "pitch_integ_max",
           "yaw_integ_max", "torque_unachieved_frac", "torque_unachieved_max_s",
           "lift_low_frac", "cruise_speed_err_mps", "touchdown_err_m", "touchdown_speed_mps",
           "wing_lift_fraction_max", "true_airspeed_max_mps", "data_gap_max_s", "aero_gap_max_s",
           "stop_tilt_max_deg", "stop_distance_m", "px4_tilt_max_deg", "lag_max_m"]


def oat_cases(cfg):
    """The nominal vehicle, then one value at a time at each end of its range."""
    cases = [("nominal", {})]
    top_wind = cfg["sweep"]["wind_speed_mps"][1]
    for name, (low, high) in cfg["sweep"].items():
        if name == "wind_speed_mps":        # a wind needs a direction to mean anything
            for direction in (0, 90, 180, 270):
                cases.append((f"wind{top_wind:g}_from{direction}",
                              {"wind_speed_mps": top_wind, "wind_from_deg": float(direction)}))
            continue
        for label, value in (("lo", low), ("hi", high)):
            if value != cfg["nominal"][name]:
                extra = {"wind_speed_mps": 5.0} if name == "wind_gust_pct" else {}
                cases.append((f"{name}_{label}", {name: float(value), **extra}))
    return cases


def wind_cases():
    """Steady wind by speed and direction; the rest nominal."""
    return [(f"wind{speed}_from{direction}",
             {"wind_speed_mps": float(speed), "wind_from_deg": float(direction)})
            for speed in (2, 4, 6, 8) for direction in range(0, 360, 45)]


# What each injected event should end in. Every case, event or not, must end on the ground
# unharmed (metrics.outcome); an event case also needs the response it was injected to test.
EXPECTED = {
    "abort": "stopped, then PX4 Hold",
    "silence": "PX4 Hold by itself (offboard lost)",
    "pusher_off": "pusher fault seen, stopped, then PX4 Land",
    "transition": "transition refused or undone, mission flown on as a quad",
    "battery": "PX4 returns home and lands there",
}


def expected_response(record):
    """(met, what happened) for an event case; (True, "") for a case without one."""
    event = (record.get("event") or "").partition("@")[0]
    flight = record.get("flight", {})
    failsafe, modes = flight.get("failsafe") or {}, flight.get("px4_modes") or []
    if not event:
        return True, ""
    if "event" not in flight:
        return False, "the event never fired"
    if event == "abort":
        return failsafe.get("handed_to") == "Hold" and "Hold" in modes, f"handed to {failsafe.get('handed_to')}, then {modes}"
    if event == "silence":
        return bool(modes) and modes[0] == "Hold", f"PX4 flew {modes}"
    if event == "pusher_off":
        return (failsafe.get("reason", "").startswith("pusher") and failsafe.get("handed_to") == "Land",
                f"{failsafe.get('reason')}, handed to {failsafe.get('handed_to')}")
    if event == "battery":
        home_m = (flight.get("after") or {}).get("from_home_m", float("nan"))
        return "Return" in modes and home_m < 3.0, f"PX4 flew {modes}, down {home_m:.1f} m from home"
    if event == "transition":
        return flight.get("completed", False), "completed" if flight.get("completed") else flight.get("fail_reason")
    return False, f"unknown event {event}"


def case_ok(record):
    """A case without an event passes its mission; one with an event ends safely, as expected."""
    if record.get("event"):
        return bool(record.get("safe")) and record.get("response_met", expected_response(record)[0])
    return bool(record["passed"])


def scenario_cases():
    """The flight as flown on the day: each cruise speed in both route modes, wind, and the
    failures the onboard software has a response for. Run alone (--jobs 1)."""
    cases = []
    for through in (1, 0):
        mode = "through" if through else "stop"
        for speed in (3, 4, 5, 6, 7):
            cases.append((f"{mode}_v{speed}", {"fly_through": through, "cruise_mps": float(speed)}))
    gusty = {"wind_speed_mps": 6.0, "wind_gust_pct": 0.3}
    for direction in (0, 90, 180, 270):
        cases.append((f"through_v5_wind6_from{direction}", {**gusty, "wind_from_deg": float(direction)}))
    cases += [
        ("through_v7_wind6_from0", {**gusty, "cruise_mps": 7.0, "wind_from_deg": 0.0}),
        ("through_v3_wind6_from90", {**gusty, "cruise_mps": 3.0, "wind_from_deg": 90.0}),
        ("abort_cruise", {"event": "abort@cruise2+1.5"}),
        ("abort_corner", {"event": "abort@corner2+2"}),
        # at 7 m/s the middle legs are all speeding up and slowing down: no cruise
        ("abort_v7", {"cruise_mps": 7.0, "event": "abort@accel2+2"}),
        ("silence_cruise", {"event": "silence@cruise2+1.5"}),
        ("silence_v7", {"cruise_mps": 7.0, "event": "silence@accel2+2"}),
        ("silence_corner_wind6", {**gusty, "wind_from_deg": 90.0, "event": "silence@corner1+2"}),
        ("pusher_off_hover", {"event": "pusher_off@hover1+2"}),
        ("pusher_off_cruise", {"event": "pusher_off@cruise2+1"}),
        ("pusher_off_v7", {"cruise_mps": 7.0, "event": "pusher_off@accel2+1"}),
        ("pusher_off_headwind6", {**gusty, "wind_from_deg": 0.0, "event": "pusher_off@cruise1+1"}),
        ("pusher_off_stop_mode", {"fly_through": 0, "event": "pusher_off@cruise2+1"}),
        ("battery_cruise", {"event": "battery@cruise2+1"}),
        ("transition_hover", {"event": "transition@hover1+2"}),
        ("transition_cruise", {"event": "transition@cruise2+1"}),
    ]
    return cases


def run_one(case, overrides, common, mission, instances, study):
    instance = instances.get()
    try:
        out = config.REPO / "out" / mission / MODEL_REVISION / study / f"{case}.json"
        temporary = out.with_name(f"{case}.{uuid.uuid4().hex[:12]}.json")
        cmd = [sys.executable, "-m", "psi_vtol.sim.run_case", "--mission", mission, "--case",
               f"{study}/{case}", "--instance", str(instance), "--json", str(temporary)]
        for name, value in {**common, **overrides}.items():
            if name == "event":
                cmd += ["--event", value]
            else:
                cmd += ["--set", f"{name}={json.dumps(value) if isinstance(value, list) else value}"]
        done = subprocess.run(cmd, capture_output=True, text=True, cwd=config.REPO)
        if done.returncode != 0 or not temporary.exists():
            temporary.unlink(missing_ok=True)
            return {"case": case, "passed": False, "overrides": overrides, "figures": {},
                    "reasons": ["runner failed: " + done.stderr.strip()[-300:]]}
        record = json.loads(temporary.read_text())
        temporary.replace(out)
        record["case"] = case
        return record
    finally:
        instances.put(instance)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("study", choices=["oat", "wind", "scenarios"])
    parser.add_argument("--mission", default=config.DEFAULT_MISSION,
                        help="folder under missions/")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="a vehicle value for every case of the study; may be repeated")
    parser.add_argument("--name", help="name of the output files; default: the study")
    parser.add_argument("--jobs", type=int, default=1,
                        help="cases flown at the same time. Keep 1 unless the machine has CPUs to "
                             "spare: Gazebo does not wait for PX4, and on 4 CPUs runs beside "
                             "another simulation showed a 1 Hz wobble that runs alone never did")
    parser.add_argument("--first-instance", type=int, default=5)
    parser.add_argument("--only", action="append", default=[], metavar="PATTERN",
                        help="fly only the cases named like this (e.g. 'pusher_off*'), and put them "
                             "in place of their earlier records in the study's table; may be repeated")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be positive")

    cfg = config.load(args.mission)
    cases = {"oat": lambda: oat_cases(cfg), "wind": wind_cases,
             "scenarios": scenario_cases}[args.study]()
    study = args.name or args.study
    order = [case for case, _ in cases]
    if args.only:
        cases = [(c, o) for c, o in cases if any(fnmatch.fnmatch(c, pat) for pat in args.only)]
        if not cases:
            parser.error("--only matches no case of this study")
    try:
        common = config.assignments(args.set)
    except ValueError as error:
        parser.error(str(error))
    out = config.REPO / "out" / args.mission / MODEL_REVISION
    (out / study).mkdir(parents=True, exist_ok=True)

    instances = queue.Queue()
    for i in range(args.jobs):
        instances.put(args.first_instance + i)
    records = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_one, case, overrides, common, args.mission, instances, study)
                   for case, overrides in cases]
        for future in futures:
            record = future.result()
            record["response_met"], record["response"] = expected_response(record)
            records.append(record)
            f = record["figures"]
            if record.get("event"):             # judged by its outcome, not by the mission
                verdict = "ok" if case_ok(record) else "BAD"
                notes = record["response"] + "".join(f"; {r}" for r in record.get("safe_reasons", []))
                if not record["response_met"]:
                    notes += f" (expected: {EXPECTED[record['event'].partition('@')[0]]})"
            else:
                verdict, notes = ("pass" if record["passed"] else "FAIL"), "; ".join(record["reasons"])
            print(f"{record['case']:28s} {verdict:4s}"
                  f"  tilt {f.get('tilt_max_deg', float('nan')):5.1f} deg"
                  f"  alt {f.get('alt_err_max_m', float('nan')):4.2f} m"
                  f"  pusher {f.get('pusher_max_cmd', float('nan')):4.2f}"
                  f"  lift {f.get('lift_max_cmd', float('nan')):4.2f}"
                  f"  {notes}", flush=True)

    if args.only and (out / f"{study}.json").exists():  # the cases flown again replace the old
        flown = {r["case"]: r for r in records}
        earlier = {r["case"]: r for r in json.loads((out / f"{study}.json").read_text())}
        records = [flown.get(c) or earlier[c] for c in order if c in flown or c in earlier]
    (out / f"{study}.json").write_text(json.dumps(records, indent=1))
    with open(out / f"{study}.csv", "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["case", "passed", "reasons", "overrides", "event", "safe", "response_met",
                         "response"] + COLUMNS)
        for r in records:
            writer.writerow([r["case"], int(r["passed"]), "; ".join(r["reasons"]),
                             json.dumps(r["overrides"]), r.get("event") or "",
                             int(bool(r.get("safe"))), int(r["response_met"]), r["response"]]
                            + [r["figures"].get(c, "") for c in COLUMNS])
    good = sum(case_ok(r) for r in records)
    print(f"\n{good} of {len(records)} as expected (a mission passed, or an event ended safely "
          f"with its expected response); table in {out / study}.csv")


if __name__ == "__main__":
    main()

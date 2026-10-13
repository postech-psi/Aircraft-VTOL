#!/usr/bin/env python3
"""Fly one simulation case and write its figures.

Starts PX4 SITL with a headless Gazebo, flies the mission profile (psi_vtol/profile.py) in
offboard mode over MAVLink (the same position/velocity setpoints the ROS 2 nodes send), lands,
and reads the PX4 log. Runs in the project's Linux Docker image (docs/03_simulation.md).

    python3 -m psi_vtol.sim.run_case                             # the nominal vehicle
    python3 -m psi_vtol.sim.run_case --set wind_speed_mps=6 --set wind_from_deg=90
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
import os
import platform
import signal
import struct
import subprocess
import time
import uuid
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from psi_vtol import config  # noqa: E402
from psi_vtol.profile import Profile  # noqa: E402
from psi_vtol.sim import gz_model, metrics  # noqa: E402

OFFBOARD = 6                    # PX4 custom main mode
AUTO, AUTO_LAND = 4, 6          # PX4 custom main and sub mode
# SET_POSITION_TARGET_LOCAL_NED type mask: a set bit means "ignore this field". Bits 0 to 5 are
# x, y, z, vx, vy, vz; these four are the acceleration and the yaw rate, never commanded.
NEVER_COMMANDED = 64 | 128 | 256 | 2048


class Abort(Exception):
    pass


class Link:
    """MAVLink connection to one PX4 instance, and the latest state it reported."""

    def __init__(self, instance):
        self.conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{14540 + instance}",
                                               source_system=255, source_component=190)
        self.t = None               # seconds since PX4 boot, simulation time
        self.pos = None             # north, east, down [m]
        self.vel = None
        self.att = None             # roll, pitch, yaw [rad]
        self.armed = False
        self.main_mode = None
        self.texts = []
        self._last_send = 0.0
        self._last_heartbeat = 0.0
        self.setpoint = None        # profile.Setpoint
        self.last_position_received = self.last_attitude_received = time.monotonic()
        self.applied_params = {}
        self.last_position_boot_ms = self.last_attitude_boot_ms = -1

    def pump(self, timeout=0.02):
        """Read one message and keep the setpoint stream alive (PX4 leaves offboard without it)."""
        msg = self.conn.recv_match(blocking=True, timeout=timeout)
        if msg is not None:
            kind = msg.get_type()
            if kind == "LOCAL_POSITION_NED":
                if msg.time_boot_ms > self.last_position_boot_ms:
                    self.last_position_received = time.monotonic()
                    self.last_position_boot_ms = msg.time_boot_ms
                self.t = msg.time_boot_ms * 1e-3
                self.pos = (msg.x, msg.y, msg.z)
                self.vel = (msg.vx, msg.vy, msg.vz)
            elif kind == "ATTITUDE":
                if msg.time_boot_ms > self.last_attitude_boot_ms:
                    self.last_attitude_received = time.monotonic()
                    self.last_attitude_boot_ms = msg.time_boot_ms
                self.att = (msg.roll, msg.pitch, msg.yaw)
            elif kind == "HEARTBEAT" and msg.get_srcComponent() == 1:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.main_mode = (msg.custom_mode >> 16) & 0xFF
            elif kind == "STATUSTEXT":
                self.texts.append(msg.text)
        now = time.monotonic()
        if self.setpoint is not None and now - self._last_send >= 0.01:
            sp = self.setpoint
            fields = (sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz)
            mask = NEVER_COMMANDED | sum(1 << i for i, v in enumerate(fields) if math.isnan(v))
            self.conn.mav.set_position_target_local_ned_send(
                0, self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                *(0.0 if math.isnan(v) else v for v in fields), 0, 0, 0, sp.yaw, 0)
            self._last_send = now

        if now - self._last_heartbeat >= 0.5:       # PX4 counts this as its ground station
            self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self._last_heartbeat = now
        return msg

    def command(self, command, *params):
        params = list(params) + [0] * (7 - len(params))
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                        command, 0, *params)

    def set_param(self, name, value):
        """Set one PX4 parameter and wait for PX4 to report the new value.

        A value given to PX4 at start-up that equals the firmware default is not kept: the
        airframe file's own default then wins. Setting it here, after boot, always holds."""
        target = (self.conn.target_system, self.conn.target_component)
        key = name.encode()

        def reply(send):
            deadline = time.monotonic() + 5.0
            send()
            while time.monotonic() < deadline:
                msg = self.pump(0.05)
                if msg is not None and msg.get_type() == "PARAM_VALUE" and msg.param_id == name:
                    return msg
            raise Abort(f"PX4 did not answer for parameter {name}")

        current = reply(lambda: self.conn.mav.param_request_read_send(*target, key, -1))
        is_int = current.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32
        # PX4 carries an integer parameter as its four bytes inside the float field.
        raw = struct.unpack("<f", struct.pack("<i", int(value)))[0] if is_int else float(value)
        received = reply(lambda: self.conn.mav.param_set_send(*target, key, raw, current.param_type))
        actual = struct.unpack("<i", struct.pack("<f", received.param_value))[0] if is_int else received.param_value
        if not math.isclose(actual, value, rel_tol=1e-5, abs_tol=1e-5):
            raise Abort(f"PX4 parameter {name}: requested {value}, received {actual}")
        self.applied_params[name] = actual

    def parameter_snapshot(self):
        """Request the full effective configuration; mark partial dumps explicitly."""
        self.conn.mav.param_request_list_send(self.conn.target_system, self.conn.target_component)
        values, expected = {}, None
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            msg = self.pump(.05)
            if msg is None or msg.get_type() != "PARAM_VALUE":
                continue
            expected = msg.param_count
            value = msg.param_value
            if msg.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                value = struct.unpack("<i", struct.pack("<f", value))[0]
            values[msg.param_id] = value
            if len(values) == expected:
                break
        return {"complete": len(values) == expected, "expected_count": expected, "values": values}


def wind_ne(p, yaw0):
    """The case's mean wind as (north, east). wind_from_deg is measured from the nose."""
    a = math.radians(p["wind_from_deg"])
    cos0, sin0 = math.cos(yaw0), math.sin(yaw0)
    return (-p["wind_speed_mps"] * (math.cos(a) * cos0 + math.sin(a) * sin0),
            -p["wind_speed_mps"] * (math.cos(a) * sin0 - math.sin(a) * cos0))


def fly(p, params, instance, wall_limit_s=420.0):
    """Fly the profile. Returns what happened, with the phase times metrics.compute() needs."""
    result = {"completed": False, "fail_reason": None, "phases": {}, "alt_m": p["alt_m"],
              "cruise_mps": p["cruise_mps"]}
    link = Link(instance)
    flight = None
    result["applied_params"] = link.applied_params
    wall_start = time.monotonic()
    try:
        if link.conn.wait_heartbeat(timeout=90) is None:
            raise Abort("no heartbeat from PX4")
        while link.pos is None or link.att is None:
            link.pump(0.2)
            if time.monotonic() - wall_start > 120:
                raise Abort("no position estimate")
        for name, value in params.items():
            link.set_param(name, value)
        result["effective_params"] = link.parameter_snapshot()
        home, yaw0 = link.pos, link.att[2]
        result["home"], result["yaw0"] = list(home), yaw0
        # With heading_into_wind the true mean wind is used; the vehicle will need an estimate.
        flight = Profile(p, home, yaw0, wind_ne(p, yaw0))
        result["phases"] = flight.times

        # --- arm and take off: PX4 wants the setpoint stream before it accepts offboard -------
        link.setpoint = flight.takeoff_setpoint()
        t_wait, t_ask = link.t, -1e9
        while not (link.armed and link.main_mode == OFFBOARD):
            link.pump()
            if time.monotonic() - wall_start > wall_limit_s:
                raise Abort("wall-clock limit during arming")
            if time.monotonic() - link.last_position_received > 2.0:
                raise Abort("position telemetry stale during arming")
            if link.t - t_wait > 60:
                raise Abort("could not arm in offboard: " + " | ".join(link.texts[-4:]))
            if link.t - t_wait > 1.5 and link.t - t_ask > 1.0:
                link.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, OFFBOARD)
                link.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
                t_ask = link.t

        land_asked = False
        touchdown_speed = None
        while True:
            if not link.armed and flight.phase == "land" and touchdown_speed is None:
                touchdown_speed = float(velocity_norm(link.vel))
            link.setpoint = flight.update(link.t, link.pos, link.vel, link.armed)
            if flight.done:
                break
            what = flight.phase
            if what == "land" and not land_asked:
                link.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, AUTO, AUTO_LAND)
                land_asked = True
            link.pump()
            if max(time.monotonic() - link.last_position_received,
                   time.monotonic() - link.last_attitude_received) > 2.0:
                raise Abort(f"state telemetry stale in {what}")
            if time.monotonic() - wall_start > wall_limit_s:
                raise Abort(f"wall-clock limit during {what}")
            if flight.overdue(link.t):
                raise Abort(f"timeout in {what}")
            if max(abs(link.att[0]), abs(link.att[1])) > math.radians(60):
                raise Abort(f"tumbled in {what}")
            if what not in ("climb", "land"):
                if not link.armed:
                    raise Abort(f"disarmed in {what}")
                if link.main_mode != OFFBOARD:
                    raise Abort(f"left offboard in {what}")
                if home[2] - link.pos[2] < 1.0:
                    raise Abort(f"hit the ground in {what}")
        result["touchdown_err_m"] = math.hypot(link.pos[0] - flight.hold[0],
                                               link.pos[1] - flight.hold[1])
        result["touchdown_speed_mps"] = touchdown_speed if touchdown_speed is not None else math.nan
        result["landing_target"] = list(flight.hold)
        result["completed"] = True
    except Abort as stop:
        result["fail_reason"] = str(stop)
        if flight is not None and flight._t0 is not None:
            result["aborted_phase"] = flight.phase
            result["phases"][flight.phase] = [flight._t0, link.t]
    result["texts"] = link.texts[-12:]
    time.sleep(1.0)                 # let the logger close the file after the disarm
    return result


def launch(case_dir, env_add, cfg, instance):
    build = Path(cfg["sim"]["px4_root"]).expanduser() / "build/px4_sitl_default"
    env = dict(os.environ, **env_add)
    # PIDs repeat in separate Docker containers. Use a UUID to isolate transport too.
    env["GZ_PARTITION"] = f"psi_qp_{instance}_{uuid.uuid4().hex}"
    # A terminal with ROS 2 sourced puts ROS's own Gazebo tools first, and that `gz` has no
    # `sim`: PX4 would wait for a Gazebo that never starts. PX4 gets the system's Gazebo.
    for name in ("PATH", "LD_LIBRARY_PATH", "GZ_CONFIG_PATH"):
        kept = [d for d in env.get(name, "").split(os.pathsep) if d and "/opt/ros/" not in d]
        env.pop(name, None)
        if kept:
            env[name] = os.pathsep.join(kept)
    (case_dir / "runtime_env.json").write_text(json.dumps({name: env.get(name) for name in
        ("GZ_PARTITION", "PX4_SIM_MODEL", "PX4_GZ_WORLD", "PX4_SIM_SPEED_FACTOR",
         "GZ_SIM_SYSTEM_PLUGIN_PATH")}, indent=2))
    log = open(case_dir / "px4.log", "w")
    return subprocess.Popen([str(build / "bin/px4"), "-i", str(instance), "-d", str(build / "etc")],
                            cwd=case_dir, env=env, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)


def stop(proc, case_dir):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            break
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            continue
    # Gazebo's server is started by a launcher and can outlive the group; its command line
    # names this case's world file.
    subprocess.run(["pkill", "-9", "-f", str(case_dir / "worlds")], check=False)


def case_values(cfg, overrides):
    """The nominal values of a case with its overrides applied."""
    p = dict(cfg["nominal"])
    for name, value in overrides.items():
        if name not in p:
            raise KeyError(f"unknown vehicle value: {name}")
        p[name] = value
    if any(not math.isfinite(v) for v in p.values() if isinstance(v, (int, float))):
        raise ValueError("case values must be finite")
    for name in ("mass_kg", "air_density", "wing_span_m", "wing_chord_m", "wing_efficiency",
                 "wing_stall_blend_deg", "lift_zero_thrust_mps", "pusher_zero_thrust_mps",
                 "motor_voltage_decay_s", "lift_thrust_max_n", "pusher_thrust_max_n"):
        if p[name] <= 0:
            raise ValueError(f"{name} must be positive")
    return p


def run(cfg, overrides, case_id, instance):
    p = case_values(cfg, overrides)
    plugin = gz_model.config_plugin_path(cfg) / "libpsi-aerodynamics.so"
    if not plugin.is_file():
        raise RuntimeError(f"Build the Gazebo plugin first: {plugin}; see docs/03_simulation.md")
    case_dir = Path(cfg["sim"]["run_root"]) / case_id / ("run-" + uuid.uuid4().hex[:12])
    case_dir.mkdir(parents=True, exist_ok=False)
    manifest = provenance(cfg, p)
    (case_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))

    proc = launch(case_dir, gz_model.write_case(case_dir, p, cfg), cfg, instance)
    try:
        flight = fly(p, gz_model.px4_params(cfg), instance)
    finally:
        stop(proc, case_dir)
    # PX4's own warnings, for a run that did not go as planned
    seen = [line for line in (case_dir / "px4.log").read_text(errors="replace").splitlines()
            if line.startswith(("WARN", "ERROR")) and "Resolve system health" not in line]
    flight["px4_warnings"] = list(dict.fromkeys(seen))[-15:]

    figures = {}
    logs = sorted(case_dir.glob("log/*/*.ulg"))
    if logs:
        try:
            figures = metrics.compute(logs[-1], flight)
        except Exception as error:      # a log cut short by a crash must not lose the result
            flight["log_error"] = repr(error)
    try:
        figures.update(metrics.aero_metrics(case_dir / "aero.csv"))
        phases = flight["phases"]
        figures["aero_covers_flight"] = bool("climb" in phases and "land" in phases and
            figures["aero_duration_s"] >= phases["land"][1] - phases["climb"][0] - .5)
    except (OSError, ValueError, KeyError) as error:
        flight["aero_log_error"] = repr(error)
    passed, reasons = metrics.verdict(flight, figures)
    record = {"case": case_id, "passed": passed, "reasons": reasons, "overrides": overrides,
              "model_revision": gz_model.MODEL_REVISION, "manifest": str(case_dir / "manifest.json"),
              "derived": gz_model.derived(p), "flight": flight, "figures": figures,
              "log": str(logs[-1]) if logs else None}
    (case_dir / "result.json").write_text(json.dumps(record, indent=1))
    return record


def velocity_norm(values):
    return math.sqrt(sum(v * v for v in values))


def provenance(cfg, p):
    def command(args, cwd=None):
        try:
            return subprocess.run(args, cwd=cwd, capture_output=True, text=True, timeout=5,
                                  check=True).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return None
    sources = [f for folder in ("simulation/gazebo", "psi_vtol", "ros2/ws/src/vtol_mission")
               for f in (config.REPO / folder).rglob("*")
               if f.suffix in (".cc", ".hh", ".py", ".txt") and "build" not in f.parts]
    plugin = gz_model.config_plugin_path(cfg) / "libpsi-aerodynamics.so"
    px4_root = Path(cfg["sim"]["px4_root"]).expanduser()
    px4_diff = command(["git", "diff", "HEAD"], px4_root)
    return {"model_revision": gz_model.MODEL_REVISION, "values": p, "sim": cfg["sim"],
            "requested_px4_params": gz_model.px4_params(cfg), "acceptance_limits": metrics.LIMITS,
            "repository_commit": command(["git", "rev-parse", "HEAD"], config.REPO),
            "repository_dirty": bool(command(["git", "status", "--porcelain"], config.REPO)),
            "px4_commit": command(["git", "rev-parse", "HEAD"], px4_root),
            "px4_dirty": bool(command(["git", "status", "--porcelain"], px4_root)),
            "px4_tracked_diff_sha256": hashlib.sha256(px4_diff.encode()).hexdigest() if px4_diff is not None else None,
            "runtime_image": os.environ.get("PSI_RUNTIME_IMAGE"),
            "runtime_image_id": os.environ.get("PSI_RUNTIME_IMAGE_ID"),
            "runtime_platform": platform.platform(), "python_version": platform.python_version(),
            "gazebo_versions": command(["gz", "sim", "--versions"]),
            "source_sha256": {str(f.relative_to(config.REPO)): hashlib.sha256(f.read_bytes()).hexdigest() for f in sources},
            "plugin_sha256": hashlib.sha256(plugin.read_bytes()).hexdigest() if plugin.is_file() else None,
            "python_packages": {n: importlib.metadata.version(n) for n in ("numpy", "pyyaml", "pyulog", "pymavlink")}}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--mission", default=config.DEFAULT_MISSION,
                        help="folder under missions/")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                        help="override one vehicle value; may be repeated")
    parser.add_argument("--px4", action="append", default=[], metavar="NAME=VALUE",
                        help="override one PX4 parameter; may be repeated")
    parser.add_argument("--case", default="single")
    parser.add_argument("--instance", type=int, default=5,
                        help="PX4 instance number; sets the MAVLink port (14540 + instance)")
    parser.add_argument("--json", help="also write the record to this file")
    args = parser.parse_args()

    cfg = config.load(args.mission)
    cfg["px4_params"].update({k: float(v) for k, v in (item.split("=", 1) for item in args.px4)})
    overrides = {k: float(v) for k, v in (item.split("=", 1) for item in args.set)}
    record = run(cfg, overrides, args.case, args.instance)
    if args.json:
        Path(args.json).write_text(json.dumps(record, indent=1))
    print(json.dumps({k: record[k] for k in ("case", "passed", "reasons", "figures")}, indent=1))
    print("derived:", json.dumps(record["derived"]))
    print("phases:", json.dumps(record["flight"]["phases"]))
    for line in record["flight"]["px4_warnings"]:
        print("px4:", line)
    print("log:", record["log"])


if __name__ == "__main__":
    main()

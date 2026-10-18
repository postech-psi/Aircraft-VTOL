#!/usr/bin/env python3
"""Fly one simulation case and write its figures.

Starts PX4 SITL with a headless Gazebo, flies the mission profile (psi_vtol/profile.py) in
offboard mode over MAVLink (the same setpoints the ROS 2 nodes send: profile.for_px4), lands,
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
import sys
import time
import uuid
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from psi_vtol import config  # noqa: E402
from psi_vtol.profile import Profile, Stop, for_px4, needs_land  # noqa: E402
from psi_vtol.sim import gz_model, metrics  # noqa: E402

OFFBOARD = 6                    # PX4 custom main mode
AUTO, AUTO_LOITER, AUTO_LAND = 4, 3, 6     # PX4 custom main mode, and sub modes of AUTO
MODE_NAMES = {(1, 0): "Manual", (2, 0): "Altitude", (3, 0): "Position", (4, 3): "Hold",
              (4, 4): "Mission", (4, 5): "Return", (4, 6): "Land", (6, 0): "Offboard"}
# SET_POSITION_TARGET_LOCAL_NED type mask: a set bit means "ignore this field". Bits 0 to 8 are
# x, y, z, vx, vy, vz, ax, ay, az; bit 10 is the yaw and bit 11 the yaw rate.
YAW_IGNORED, YAW_RATE_IGNORED = 1 << 10, 1 << 11


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
        self.main_mode = self.sub_mode = None
        self.texts = []
        self._last_send = 0.0
        self._last_heartbeat = 0.0
        self.setpoint = None        # profile.Setpoint
        self.last_position_received = self.last_attitude_received = time.monotonic()
        self.applied_params = {}
        self.last_position_boot_ms = self.last_attitude_boot_ms = -1

    def mode_name(self):
        return MODE_NAMES.get((self.main_mode, self.sub_mode), f"{self.main_mode}.{self.sub_mode}")

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
                self.sub_mode = (msg.custom_mode >> 24) & 0xFF if self.main_mode == AUTO else 0
            elif kind == "STATUSTEXT":
                self.texts.append(msg.text)
        now = time.monotonic()
        if self.setpoint is not None and now - self._last_send >= 0.01:
            sp = for_px4(self.setpoint, self.pos)
            fields = (sp.x, sp.y, sp.z, sp.vx, sp.vy, sp.vz, sp.ax, sp.ay, sp.az)
            mask = sum(1 << i for i, v in enumerate(fields) if math.isnan(v))
            mask |= (YAW_IGNORED if math.isnan(sp.yaw) else 0) | (
                YAW_RATE_IGNORED if math.isnan(sp.yawspeed) else 0)
            self.conn.mav.set_position_target_local_ned_send(
                0, self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
                *(0.0 if math.isnan(v) else v for v in fields),
                0.0 if math.isnan(sp.yaw) else sp.yaw, 0.0 if math.isnan(sp.yawspeed) else sp.yawspeed)
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
            # PX4 drops requests that arrive while it is still starting, and UDP may lose any
            # message, so ask again every second until the deadline.
            deadline, asked = time.monotonic() + 5.0, -math.inf
            while time.monotonic() < deadline:
                if time.monotonic() - asked >= 1.0:
                    send()
                    asked = time.monotonic()
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


# Events a case can inject, at a phase plus seconds ("abort@cruise2+1.5"), to see what a real
# flight could meet. GPS loss cannot be injected: PX4 v1.17 simulates no GPS failure in Gazebo,
# and its motor failure injection only blanks the ESC telemetry, so the pusher is stopped in
# the aerodynamics plugin instead.
EVENTS = {
    "abort": "the mission is aborted: the onboard failsafe stops the vehicle and hands it to Hold",
    "silence": "the setpoints stop, as if the Jetson died: PX4's offboard-loss failsafe acts",
    "transition": "a fixed-wing transition is commanded, as from a button or a switch",
    "pusher_off": "the pusher stops producing thrust (Gazebo; PX4 is not told)",
    "battery": "the battery runs down to its critical level within seconds: PX4's battery failsafe acts",
}
# PX4's simulated battery drains only while armed, down to SIM_BAT_MIN_PCT. The event lowers that
# floor to between the critical (BAT_CRIT_THR, 7 %) and the emergency level (BAT_EMERGEN_THR, 5 %).
BATTERY_FLOOR_PCT, BATTERY_DRAIN_S = 6.0, 10.0
HOLD_WATCH_S = 15.0     # how long PX4's Hold is followed before the run lands it, as a pilot would
LAND_WATCH_S = 120.0    # how long a landing by PX4 may take
VTOL_FW = 4             # MAV_VTOL_STATE_FW


def parse_event(text):
    """'name@phase+seconds' -> (name, phase, seconds)."""
    name, _, when = text.partition("@")
    phase, _, offset = when.partition("+")
    if name not in EVENTS or not phase:
        raise ValueError(f"event {text!r}: expected one of {sorted(EVENTS)} as name@phase+seconds")
    return name, phase, float(offset or 0.0)


def fly(p, params, instance, wall_limit_s=420.0, event=None, gz_env=None):
    """Fly the profile. Returns what happened, with the phase times metrics.compute() needs.

    When the profile reports a fault, a phase overruns or the event aborts the mission, the
    runner does what the onboard nodes do: stop smoothly (profile.Stop), then hand the vehicle to
    PX4's Hold, or its Land when it cannot make way forward (profile.needs_land). It follows PX4 until the vehicle is down: a Hold
    for HOLD_WATCH_S, then a Land, as the pilot would command. gz_env: the environment of the
    Gazebo this flight runs in, for events that act on the simulation.
    """
    result = {"completed": False, "fail_reason": None, "phases": {}, "alt_m": p["alt_m"],
              "cruise_mps": p["cruise_mps"]}
    link = Link(instance)
    flight = None
    result["applied_params"] = link.applied_params
    wall_start = time.monotonic()

    def watch(reason, land_after_s=HOLD_WATCH_S):
        """Follow PX4 while it flies on its own, and stop the run with reason once it is down. A
        Hold is landed after land_after_s (None: never); PX4's other modes are left to it."""
        t_start, land_asked, held_at = link.t, land_after_s is None, link.pos
        result["phases"]["px4"] = [t_start, t_start]
        modes = result["px4_modes"] = [link.mode_name()]       # the modes PX4 flew, in order
        while link.armed and link.t - t_start < (land_after_s or 0.0) + LAND_WATCH_S:
            link.pump()
            result["phases"]["px4"][1] = link.t
            if link.mode_name() != modes[-1]:
                modes.append(link.mode_name())
            if time.monotonic() - wall_start > wall_limit_s:
                reason += "; wall-clock limit while PX4 flew"
                break
            if max(abs(link.att[0]), abs(link.att[1])) > math.radians(60):
                reason += "; tumbled"
                break
            if not land_asked and link.mode_name() == "Hold" and link.t - t_start >= land_after_s:
                result["land_commanded_t"] = link.t
                link.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, AUTO, AUTO_LAND)
                land_asked = True
        result["after"] = {"armed": link.armed, "mode": link.main_mode,
                           "height_m": home[2] - link.pos[2], "texts": link.texts[-6:],
                           "landed": not link.armed,
                           "drift_m": math.hypot(link.pos[0] - held_at[0], link.pos[1] - held_at[1]),
                           "from_home_m": math.hypot(link.pos[0] - home[0], link.pos[1] - home[1])}
        raise Abort(reason)

    def failsafe(reason):
        """What setpoint_mux and mission_manager do: stop without a step, then hand over."""
        brake = Stop(link.t, link.pos, link.vel, link.att[2])
        result["failsafe"] = {"reason": reason, "t": link.t, "speed_mps": math.hypot(*link.vel[:2])}
        if flight._t0 is not None:
            result["phases"][flight.phase] = [flight._t0, link.t]
        result["phases"]["stop"] = [link.t, link.t]
        while not brake.stopped(link.t - 1.0):
            link.setpoint = brake.setpoint(link.t)
            link.pump()
            result["phases"]["stop"][1] = link.t
            if max(abs(link.att[0]), abs(link.att[1])) > math.radians(60):
                raise Abort(f"{reason}; tumbled while stopping")
        land = needs_land(reason)
        result["failsafe"]["handed_to"] = "Land" if land else "Hold"
        for _ in range(3):
            link.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, AUTO, AUTO_LAND if land else AUTO_LOITER)
        watch(reason, land_after_s=None if land else HOLD_WATCH_S)

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
        result["legs"] = [{k: leg[k] for k in ("start", "end", "heading", "speed")} for leg in flight.legs]
        result["path"] = [[round(n, 2), round(e, 2)] for n, e in flight.polyline(1.0)]

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

        land_asked = event_phase = False
        while True:
            what = flight.phase
            event_phase = event_phase or (event is not None and what == event[1])
            if (event_phase and "event" not in result and flight._t0 is not None
                    and (what != event[1] or link.t - flight._t0 >= event[2])):
                # at the offset into its phase, or as the phase ends if it is shorter
                result["event"] = {"name": event[0], "phase": what, "t": link.t,
                                   "speed_mps": math.hypot(*link.vel[:2])}
                if event[0] == "abort":
                    failsafe("mission aborted")
                if event[0] == "silence":
                    link.setpoint = None
                    result["phases"][what] = [flight._t0, link.t]
                    while link.main_mode == OFFBOARD and link.t - result["event"]["t"] < 30.0:
                        link.pump()
                    watch(f"setpoints stopped in {what}")
                if event[0] == "transition":
                    link.command(mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION, VTOL_FW)
                if event[0] == "pusher_off":
                    pusher_off(instance, gz_env)
                if event[0] == "battery":
                    link.set_param("SIM_BAT_MIN_PCT", BATTERY_FLOOR_PCT)
                    link.set_param("SIM_BAT_DRAIN", BATTERY_DRAIN_S)
            link.setpoint = flight.update(link.t, link.pos, link.vel, link.armed, link.att)
            if flight.done:
                break
            if flight.fault:
                failsafe(flight.fault)
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
                failsafe(f"timeout in {what}")
            if max(abs(link.att[0]), abs(link.att[1])) > math.radians(60):
                raise Abort(f"tumbled in {what}")
            if what not in ("climb", "land"):
                if not link.armed:
                    raise Abort(f"disarmed in {what}")
                if link.main_mode != OFFBOARD:
                    result["phases"][what] = [flight._t0, link.t]
                    watch(f"PX4 left offboard in {what}: " + " | ".join(link.texts[-2:]))
                if home[2] - link.pos[2] < 1.0:
                    raise Abort(f"hit the ground in {what}")
        result["touchdown_err_m"] = math.hypot(link.pos[0] - flight.hold[0],
                                               link.pos[1] - flight.hold[1])
        result["landing_target"] = list(flight.hold)
        result["completed"] = True
    except Abort as stop:
        result["fail_reason"] = str(stop)
        if flight is not None and flight._t0 is not None:
            result["aborted_phase"] = flight.phase
            result["phases"].setdefault(flight.phase, [flight._t0, link.t])
    if flight is not None:
        result["lag_max_m"] = flight.lag_max
    result["texts"] = link.texts[-12:]
    time.sleep(1.0)                 # let the logger close the file after the disarm
    return result


def pusher_off(instance, gz_env):
    """Stop the pusher's thrust in the aerodynamics plugin (simulation/gazebo/AeroSystem.cc)."""
    topic = f"/model/{gz_model.MODEL_NAME}_{instance}/rotor_off"
    for _ in range(2):          # gz transport may drop the first message of a new publisher
        subprocess.Popen(["gz", "topic", "-t", topic, "-m", "gz.msgs.StringMsg",
                          "-p", 'data: "rotor_pusher_joint"'], env=gz_env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


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
    proc = subprocess.Popen([str(build / "bin/px4"), "-i", str(instance), "-d", str(build / "etc")],
                            cwd=case_dir, env=env, stdout=log, stderr=subprocess.STDOUT,
                            stdin=subprocess.DEVNULL, start_new_session=True)
    proc.gz_env = env           # for events that act on this Gazebo
    return proc


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


def run(cfg, overrides, case_id, instance, event=None):
    """Fly one case. event: (name, phase, seconds) from parse_event, or None."""
    p = case_values(cfg, overrides)
    plugin = gz_model.config_plugin_path(cfg) / "libpsi-aerodynamics.so"
    if not plugin.is_file():
        raise RuntimeError(f"Build the Gazebo plugin first: {plugin}; see docs/03_simulation.md")
    case_dir = Path(cfg["sim"]["run_root"]) / case_id / ("run-" + uuid.uuid4().hex[:12])
    case_dir.mkdir(parents=True, exist_ok=False)
    manifest = provenance(cfg, p)
    (case_dir / "manifest.json").write_text(json.dumps(manifest, indent=2))

    alongside = other_simulators()
    proc = launch(case_dir, gz_model.write_case(case_dir, p, cfg), cfg, instance)
    try:
        flight = fly(p, gz_model.px4_params(cfg, p), instance, event=event, gz_env=proc.gz_env)
        flight["other_simulators"] = max(alongside, other_simulators() - 1)
    finally:
        stop(proc, case_dir)
    if flight["other_simulators"]:
        print(f"note: {flight['other_simulators']} other PX4 simulation(s) ran alongside this case; "
              "see other_simulators()", file=sys.stderr)
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
    safe, unsafe = metrics.outcome(flight, figures)
    record = {"case": case_id, "passed": passed, "reasons": reasons, "safe": safe,
              "safe_reasons": unsafe, "overrides": overrides,
              "event": None if event is None else "{}@{}+{:g}".format(*event),
              "model_revision": gz_model.MODEL_REVISION, "manifest": str(case_dir / "manifest.json"),
              "derived": gz_model.derived(p), "flight": flight, "figures": figures,
              "log": str(logs[-1]) if logs else None}
    (case_dir / "result.json").write_text(json.dumps(record, indent=1))
    return record


def other_simulators():
    """PX4 SITL processes running on this machine besides the one about to start.

    PX4's clock follows Gazebo's, but Gazebo steps on without waiting for PX4's motor commands, so
    a PX4 that falls behind sends them late in simulated time. On 4 CPUs, 6 of 11 squares flown
    beside other simulations showed a 1 Hz wobble that 13 runs alone never did. The count is
    recorded with each result so such runs can be told apart.
    """
    count = 0
    for comm in Path("/proc").glob("[0-9]*/comm"):
        try:
            count += comm.read_text().strip() == "px4"
        except OSError:
            pass
    return count


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
            "requested_px4_params": gz_model.px4_params(cfg, p), "acceptance_limits": metrics.LIMITS,
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
                        help="override one case value, e.g. wind_speed_mps=4 or "
                             "route_m=[[50,0],[0,0]]; may be repeated")
    parser.add_argument("--px4", action="append", default=[], metavar="NAME=VALUE",
                        help="override one PX4 parameter; may be repeated")
    parser.add_argument("--case", default="single")
    parser.add_argument("--instance", type=int, default=5,
                        help="PX4 instance number; sets the MAVLink port (14540 + instance)")
    parser.add_argument("--json", help="also write the record to this file")
    parser.add_argument("--event", metavar="NAME@PHASE+SECONDS",
                        help="inject an event: " + "; ".join(f"{k}: {v}" for k, v in EVENTS.items()))
    args = parser.parse_args()

    cfg = config.load(args.mission)
    try:
        cfg["px4_params"].update(config.assignments(args.px4))
        overrides = config.assignments(args.set)
        event = parse_event(args.event) if args.event else None
    except ValueError as error:
        parser.error(str(error))
    record = run(cfg, overrides, args.case, args.instance, event)
    if args.json:
        Path(args.json).write_text(json.dumps(record, indent=1))
    print(json.dumps({k: record[k] for k in ("case", "passed", "reasons", "safe", "safe_reasons",
                                             "figures")}, indent=1))
    print("derived:", json.dumps(record["derived"]))
    print("phases:", json.dumps(record["flight"]["phases"]))
    for line in record["flight"]["px4_warnings"]:
        print("px4:", line)
    print("log:", record["log"])


if __name__ == "__main__":
    main()

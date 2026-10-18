"""The mission profile: what to command at each moment of the flight.

Climb and hover, fly the route, then align over the last waypoint, descend and land on it.

The route is flown on a planned reference (psi_vtol/trajectory.py): a point that moves along
the path at a planned speed and stops exactly on the waypoint. PX4 gets its velocity and
acceleration, pulled towards the point (for_px4), so the vehicle follows the point instead of
chasing a speed, and stops where the reference stops. Speed changes build up and fade out (brake_ramp_s), so the tilt
setpoint never steps: a step in it pitched the nose to 7.2 degrees and cut the pusher within
0.2 s in SITL. Two ways to fly the route:

    stop and turn   stop over every waypoint and turn the nose towards the next in a hover,
                    where the lift rotors have the most yaw authority
    fly through     (fly_through: 1) round each corner on a curve, banked by the lift rotors
                    like an airplane, with the nose along the path; a corner sharper than
                    MAX_THROUGH_TURN is still a stop and a turn

The "transition" is not a mode change: it is the planned speed rising, and the "back
transition" the planned speed falling to zero on the waypoint.

The simulation runner (psi_vtol/sim/run_case.py, over MAVLink) and the ROS 2 nodes
(ros2/ws/src/vtol_mission) both fly from this module, so what was simulated is what the vehicle
is asked to do. Nothing here knows about MAVLink or ROS.

Positions and velocities are in PX4's local frame: north, east, down. Times are the timestamps
of PX4's own messages in seconds, so a simulation running faster than the clock is flown
correctly; only differences between them are used. Attitudes are (roll, pitch, yaw) in radians.
"""
import math
from dataclasses import dataclass, replace

from psi_vtol.trajectory import Path, SpeedPlan, SpeedRamp

NAN = math.nan
MIN_ALT_M = 6.0                 # pusher assist works above 5 m (VT_FWD_THRUST_EN 6, MPC_LAND_ALT2)
DESCENT_MPS = 1.5
LAND_FROM_M = 4.0               # below this height the landing is handed to PX4's Land mode
TURN_RATE = math.radians(20.0)  # how fast the commanded heading may change in a hover
TURN_SETTLE_S = 2.0             # without an attitude to watch: hover this long after a turn
TURN_ALIGNED = math.radians(3.0)    # a turn ends when the nose is this close to its heading
TURN_ALIGNED_S = 1.0                # ... for this long
ARRIVE_M = 1.0                  # a hover over a waypoint ends only this close to it
MIN_CRUISE_S = 2.0              # a short leg is flown slower, so it still holds its speed this long
MAX_THROUGH_TURN = math.radians(120.0)  # sharper corners are flown stop and turn
MIN_TURN_RADIUS_M = 5.0         # a corner squeezed between short legs is not made tighter
# Faults the profile can see from the position alone. The runner and the nodes act on them.
LAG_FAULT_M = 8.0               # the vehicle this far behind the reference
STALL_S, STALL_MPS = 3.0, 0.5   # this long into a speed-up and still slower than this
# A pusher that stops in flight: the vehicle coasts down on its drag, or creeps along, whatever
# the plan does. It is stuck when it is
STUCK_MPS = 1.5                 # this much slower than planned,
STUCK_GAP_MPS = 0.5             # further behind the plan than STUCK_S ago by this much,
STUCK_GAIN_MPS = 0.1            # and has gained no more speed than this itself
STUCK_S = 2.0
# Landing: settle over the point and turn the nose into the wind before descending.
ALIGN_M, ALIGN_MPS, ALIGN_HOLD_S = 0.3, 0.2, 2.0
ALIGN_TIMEOUT_S = 25.0          # then descend anyway, if within ARRIVE_M
WIND_ROLL = math.radians(1.5)   # the lean that still counts as facing the wind
WIND_PITCH = math.radians(2.0)  # nose-up lean past this: the wind is from behind
WIND_YAW_GAIN = 1.0             # yaw rate per radian of roll, as PX4's weathervane (WV_GAIN)
WIND_RATE = math.radians(15.0)  # as WV_YRATE_MAX
# Stopping a moving vehicle when the mission is cut short (Stop).
STOP_ACCEL_MPS2, STOP_RAMP_S = 1.5, 0.5
# A moving reference is sent to PX4 as a velocity pulled towards its point (for_px4).
TRACK_P = 0.8                   # 1/s, as PX4's own MPC_XY_P (0.8 for airframe 4004)
TRACK_MAX_MPS = 2.0             # the pull is at most this fast

LANDING_PHASES = ("align", "descend", "land")
FLY_KINDS = ("accel", "cruise", "decel", "corner")


@dataclass
class Setpoint:
    """One setpoint for PX4. A NaN field is not commanded, as in PX4. The acceleration and the
    yaw rate are feedforward: PX4 still closes its loops on the position and velocity."""
    x: float = NAN
    y: float = NAN
    z: float = NAN
    vx: float = NAN
    vy: float = NAN
    vz: float = NAN
    yaw: float = NAN
    ax: float = NAN
    ay: float = NAN
    az: float = NAN
    yawspeed: float = NAN


def for_px4(sp, pos):
    """The setpoint as PX4 is sent it, at the vehicle position pos (north, east, down).

    A moving reference goes without its horizontal point: as its velocity, pulled towards the
    point here as PX4 would pull it (TRACK_P). If the setpoints stop coming, PX4 keeps flying
    the last one until it calls the link lost (COM_OF_LOSS_T) and holds, braking smoothly. With
    the point in it, the last setpoint would have PX4 brake back to a point the vehicle has
    already passed, as hard as its tilt limit allows: 29 deg nose up at 5 m/s in SITL.
    """
    if (pos is None or not all(math.isfinite(v) for v in (sp.x, sp.y, sp.vx, sp.vy))
            or sp.vx == sp.vy == 0.0):
        return sp
    north, east = sp.x - pos[0], sp.y - pos[1]
    gain = TRACK_P * min(1.0, TRACK_MAX_MPS / max(TRACK_P * math.hypot(north, east), 1e-9))
    return replace(sp, x=NAN, y=NAN, vx=sp.vx + gain * north, vy=sp.vy + gain * east)


def needs_land(reason):
    """Whether a failsafe for this fault hands the stopped vehicle to PX4's Land, not its Hold.

    Every fault that means the vehicle cannot make way forward: a pusher fault, or falling far
    behind (a pusher that is weak or saturated). In Hold, pusher assist keeps the nose level and
    PX4 holds a dead pusher at full; switched to Land after 15 s of that, the vehicle pitched to
    48 deg in SITL. Land switches pusher assist off (VT_FWD_THRUST_EN 6), so the vehicle can tilt
    to hold its place, and lands where it is.
    """
    return reason.startswith(("pusher", "falling behind"))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Stop:
    """Bring a moving vehicle to a hover without a step in its tilt, then hold where it stopped.

    What to fly when the mission is cut short: a position setpoint where the vehicle happens to
    be, sent at 5 m/s, pitched it to 48 degrees in SITL. This slows it along its own direction of
    travel at STOP_ACCEL_MPS2, eased in and out, and holds its height and heading.
    """

    def __init__(self, t, pos, vel, yaw, accel=STOP_ACCEL_MPS2, ramp_s=STOP_RAMP_S):
        speed = math.hypot(vel[0], vel[1])
        self.dir = (vel[0] / speed, vel[1] / speed) if speed > 0.05 else (1.0, 0.0)
        self.ramp = SpeedRamp(speed, 0.0, accel, ramp_s)
        self.t0, self.start, self.yaw = t, tuple(pos), yaw
        self.end = (pos[0] + self.dir[0] * self.ramp.length, pos[1] + self.dir[1] * self.ramp.length)

    def stopped(self, t):
        return t - self.t0 >= self.ramp.duration

    def setpoint(self, t):
        d, v, a = self.ramp.state(t - self.t0)
        (dn, de), (n0, e0) = self.dir, self.start[:2]
        return Setpoint(x=n0 + dn * d, y=e0 + de * d, z=self.start[2], vx=dn * v, vy=de * v,
                        ax=dn * a, ay=de * a, yaw=self.yaw)


class Profile:
    """Steps through the phases of the flight, one setpoint per call of update().

    p        the profile values of missions/<mission>/mission.yaml
    home     position where the vehicle stands when the profile is made; z is the ground level
    yaw0     heading at that moment; route_m is written in the frame of this heading, unless
             route_heading_deg (>= 0) fixes the frame's heading from north
    part     "all", "path" (take-off and route) or "landing" (align, descend and land where it is)

    The phases are climb, hover1, then per stretch of the route flown without stopping:
    turn<i> (when the nose must turn first), then accel<i>, cruise<i>, decel<i> on leg i and
    corner<i> between legs i and i+1 (fly-through), and hover<j> over the waypoint it stops on.
    Then align, descend, land.

    After update(), fault is None or why the flight cannot go on as planned; needs_land(fault)
    tells whether the vehicle should land where it is.
    """

    def __init__(self, p, home, yaw0, part="all"):
        self.alt, self.cruise, self.accel = p["alt_m"], p["cruise_mps"], p["accel_mps2"]
        if self.alt < MIN_ALT_M:
            raise ValueError(f"alt_m {self.alt:g} is below {MIN_ALT_M:g} m: PX4 switches the pusher "
                             "off under 5 m, and the vehicle would cruise nose-down as a quad")
        self.hover_s, self.ramp_s = p["hover_s"], p["brake_ramp_s"]
        self.through = p["fly_through"] >= 0.5
        self.radius, self.lateral = p["turn_radius_m"], p["lateral_mps2"]
        self.home, self.yaw0 = tuple(home), yaw0
        frame = math.radians(p["route_heading_deg"]) if p["route_heading_deg"] >= 0.0 else yaw0
        self.z_fly = home[2] - self.alt
        self.legs = self._legs(p["route_m"], frame)
        self.runs = self._runs()
        self._target_yaw = self._yaw_cmd = yaw0
        self._t_yaw = self._turned_at = self._aligned_at = None
        self._lean = None                   # filtered (roll, pitch) while aligning
        self._point = self.home[:2]         # the point a hover holds

        self._piece_of = {}                 # phase -> (run, piece index)
        if part == "landing":
            self.phases = LANDING_PHASES
        else:
            self.phases = ["climb", "hover1"]
            heading = yaw0
            for r, run in enumerate(self.runs):
                first, last = run["legs"][0], run["legs"][-1]
                if self.legs[first]["turn"] or (r == 0 and self._needs_turn(heading, first)):
                    self.phases.append(f"turn{first + 1}")
                for k, piece in enumerate(run["plan"].pieces):
                    label, index = piece[0], first + piece[1]
                    name = f"{label}{index + 1}"
                    self.phases.append(name)
                    self._piece_of[name] = (r, k)
                self.phases.append(f"hover{last + 2}")
                heading = self.legs[last]["heading"]
            self.phases = tuple(self.phases) + (LANDING_PHASES if part == "all" else ())
        self.phase = self.phases[0]
        self.times = {}             # finished phases: name -> [start, end]
        self.hold = None            # the point aligned over, descended over and landed on
        self.done = False
        self.fault = None
        self.lag_max = 0.0          # farthest the vehicle has been behind the reference [m]
        self._t0 = self._run_t0 = None      # start of the current phase, and of its stretch
        self._speeds = []           # (t, along-track speed, planned speed) of the last STUCK_S
        self._last = self.takeoff_setpoint()

    def _legs(self, route, frame):
        """The legs of the route in the local frame: start, end, direction, heading."""
        c, s = math.cos(frame), math.sin(frame)
        legs, start, heading = [], self.home[:2], self.yaw0
        for forward, left in route:
            end = (self.home[0] + forward * c + left * s, self.home[1] + forward * s - left * c)
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length < 1.0:
                raise ValueError(f"route leg to {[forward, left]} is shorter than 1 m")
            d = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
            leg_heading = math.atan2(d[1], d[0])
            turn = abs(_wrap(leg_heading - heading))
            legs.append({"start": start, "end": end, "dir": d, "length": length,
                         "heading": leg_heading, "turn_rad": turn, "turn": False})
            start, heading = end, leg_heading
        if not legs:
            raise ValueError("the route has no waypoints")
        return legs

    def _needs_turn(self, heading, leg):
        return abs(_wrap(self.legs[leg]["heading"] - heading)) > math.radians(1.0)

    def _runs(self):
        """Stretches of the route flown without stopping, each with its path and speed plan."""
        runs, first = [], 0
        for i in range(len(self.legs)):
            sharp = i + 1 < len(self.legs) and self.legs[i + 1]["turn_rad"] > MAX_THROUGH_TURN
            if not self.through or sharp or i + 1 == len(self.legs):
                legs = list(range(first, i + 1))
                points = [self.legs[first]["start"]] + [self.legs[j]["end"] for j in legs]
                speed = self.cruise
                if len(legs) == 1:          # a short leg is flown slower, to keep some cruise
                    speed = self._fitting_speed(self.legs[first]["length"])
                transition = max(1.0, speed * self.ramp_s)     # the bank builds up over ramp_s
                path = Path(points, self.radius, transition, min_radius_m=MIN_TURN_RADIUS_M)
                plan = SpeedPlan(path, speed, self.accel, self.ramp_s, self.lateral)
                runs.append({"legs": legs, "path": path, "plan": plan})
                if first > 0:
                    self.legs[first]["turn"] = self._needs_turn(self.legs[first - 1]["heading"], first)
                first = i + 1
        for run in runs:                    # the steady speed planned on each leg, for metrics
            for label, index, _, _, ramp, _, _ in run["plan"].pieces:
                leg = self.legs[run["legs"][0] + index]
                if label == "cruise":
                    leg["speed"] = ramp
            for j in run["legs"]:
                self.legs[j].setdefault("speed", run["plan"].top_speed)
        return runs

    def _fitting_speed(self, length):
        """The fastest speed up to cruise with which a leg still holds it for MIN_CRUISE_S."""
        def fits(v):
            ramps = 2 * SpeedRamp(0.0, v, self.accel, self.ramp_s).length
            return ramps + v * MIN_CRUISE_S <= length
        if fits(self.cruise):
            return self.cruise
        low, high = 0.0, self.cruise
        for _ in range(50):
            mid = (low + high) / 2
            low, high = (mid, high) if fits(mid) else (low, mid)
        return low

    def takeoff_setpoint(self):
        """Hold the flight height over home. PX4 wants this stream before it accepts offboard."""
        return Setpoint(x=self.home[0], y=self.home[1], z=self.z_fly, yaw=self.yaw0)

    def polyline(self, step_m=1.0):
        """The route the vehicle was meant to fly over, as (north, east) points."""
        return [point for run in self.runs for point in run["path"].polyline(step_m)]

    def limit_s(self):
        """Longest the current phase may take; longer means the flight is not going as planned."""
        kind = self._kind()
        if kind == "fly":
            run, k = self._piece_of[self.phase]
            return self.runs[run]["plan"].pieces[k][5] + 15.0
        if kind == "turn":
            leg = self.legs[int(self.phase[4:]) - 1]
            return leg["turn_rad"] / TURN_RATE + TURN_SETTLE_S + 15
        if kind == "hover":
            return self.hover_s + 5 if self.phase == "hover1" else self.hover_s + 15
        return {"climb": 60.0, "align": ALIGN_TIMEOUT_S + 20, "descend": self.alt / DESCENT_MPS + 20,
                "land": 60.0}[kind]

    def overdue(self, t):
        return self._t0 is not None and t - self._t0 > self.limit_s()

    def update(self, t, pos, vel, armed=True, att=None):
        """The setpoint for this moment. Moves on to the next phase when this one is finished."""
        if self.done:
            return self._last
        if self._t0 is None:
            self._enter(t, pos)
        setpoint, finished = getattr(self, "_" + self._kind())(t, pos, vel, armed, att)
        self._last = setpoint
        if finished:
            self.times[self.phase] = [self._t0, t]
            following = self.phases.index(self.phase) + 1
            if following == len(self.phases):
                self.done = True
            else:
                self.phase = self.phases[following]
                self._enter(t, pos)
        return setpoint

    def _kind(self):
        kind = self.phase.rstrip("0123456789")
        return "fly" if kind in FLY_KINDS else kind

    def _enter(self, t, pos):
        """The current phase starts now."""
        self._t0 = t
        kind = self._kind()
        if kind == "turn":
            leg = self.legs[int(self.phase[4:]) - 1]
            self._target_yaw, self._turned_at = leg["heading"], None
            self._point = leg["start"]
        if kind == "fly" and self._piece_of[self.phase][1] == 0:
            self._run_t0 = t                            # a stretch starts with its first piece
            self._speeds = []
        if kind == "hover" and self.phase != "hover1":
            self._point = self.legs[int(self.phase[5:]) - 2]["end"]
        if self.phase == f"hover{len(self.legs) + 1}":
            self.hold = (*self.legs[-1]["end"], self.z_fly)     # the route ends here: land on it
        if kind == "align":
            self._aligned_at, self._lean = None, None
            if self.hold is None:
                self.hold = tuple(pos)                  # landing part alone: land where it is

    # --- one method per kind of phase: (setpoint, finished) ----------------------------------
    def _climb(self, t, pos, vel, armed, att):
        reached = self.home[2] - pos[2] > self.alt - 0.5 and abs(vel[2]) < 0.3
        return self.takeoff_setpoint(), reached

    def _hover(self, t, pos, vel, armed, att):
        if self.phase == "hover1":
            return self._hover_over(self.home, t), t - self._t0 > self.hover_s
        arrived = math.hypot(self._point[0] - pos[0], self._point[1] - pos[1]) < ARRIVE_M
        return self._hover_over(self._point, t), t - self._t0 > self.hover_s and arrived

    def _turn(self, t, pos, vel, armed, att):
        """Turn the nose towards the next leg over the point reached. Done when the nose is there."""
        before = self._yaw_cmd
        setpoint = self._hover_over(self._point, t)
        turning = abs(_wrap(self._target_yaw - before)) > 1e-9
        if turning and self._t_yaw is not None:
            setpoint.yawspeed = math.copysign(TURN_RATE, _wrap(self._target_yaw - before))
        if self._turned_at is None and not turning:
            self._turned_at = t
        if self._turned_at is None:
            return setpoint, False
        if att is None:
            return setpoint, t - self._turned_at >= TURN_SETTLE_S
        if abs(_wrap(att[2] - self._target_yaw)) > TURN_ALIGNED:
            self._turned_at = t                         # wait for the nose itself
        return setpoint, t - self._turned_at >= TURN_ALIGNED_S

    def _fly(self, t, pos, vel, armed, att):
        """Follow the planned reference along the path of this stretch."""
        r, k = self._piece_of[self.phase]
        run = self.runs[r]
        tau = t - self._run_t0
        s, v, a, _ = run["plan"].state(tau)
        n, e, h, curv = run["path"].at(s)
        c, sn = math.cos(h), math.sin(h)
        lateral = v * v * curv              # towards the inside of the turn
        setpoint = Setpoint(x=n, y=e, z=self.z_fly, vx=v * c, vy=v * sn,
                            ax=a * c - lateral * sn, ay=a * sn + lateral * c)
        self._yaw_cmd, self._t_yaw = _wrap(h), t
        setpoint.yaw, setpoint.yawspeed = self._yaw_cmd, v * curv
        # Faults: no forward speed at all, or stuck behind the plan, mean no forward thrust (the
        # pusher). Stuck is not just slower than planned, which a slow speed-up also is, but
        # gaining no speed while falling further behind, through any speed-up, corner or
        # slow-down of the plan (at 7 m/s these follow each other within 1.4 s). Last, far behind.
        lag = (n - pos[0]) * c + (e - pos[1]) * sn
        self.lag_max = max(self.lag_max, lag)
        along = vel[0] * c + vel[1] * sn
        self._speeds = [x for x in self._speeds if t - x[0] <= STUCK_S] + [(t, along, v)]
        t_ago, along_ago, v_ago = self._speeds[0]
        stuck = (t - t_ago >= STUCK_S - 0.1 and v - along > STUCK_MPS
                 and (v - along) - (v_ago - along_ago) >= STUCK_GAP_MPS
                 and along - along_ago <= STUCK_GAIN_MPS)
        if tau > STALL_S and v > 2 * STALL_MPS and along < STALL_MPS:
            self.fault = f"pusher: no forward speed {tau:.0f} s into {self.phase}"
        elif stuck:
            self.fault = f"pusher: not speeding up, at {along:.1f} m/s of {v:.1f} in {self.phase}"
        elif lag > LAG_FAULT_M:
            self.fault = f"falling behind the route by {lag:.0f} m in {self.phase}"
        start, duration = run["plan"].pieces[k][2], run["plan"].pieces[k][5]
        return setpoint, tau >= start + duration

    def _align(self, t, pos, vel, armed, att):
        """Settle over the landing point and turn the nose into the wind, as a weathervane.

        The vehicle leans into the wind to hold its place. Leaning sideways, it turns towards
        the lean; nose up with no lean sideways, the wind is from behind and it turns round. A
        wind from ahead shows as no lean: pusher assist holds it level against the wind.
        """
        setpoint = Setpoint(x=self.hold[0], y=self.hold[1], z=self.z_fly)
        dt = 0.0 if self._t_yaw is None else min(0.1, max(0.0, t - self._t_yaw))
        self._t_yaw = t
        facing = True
        if att is not None:
            w = min(1.0, dt / 1.0)                      # one-second average of the lean
            self._lean = att[:2] if self._lean is None else tuple(
                old + (new - old) * w for old, new in zip(self._lean, att[:2]))
            roll, pitch = self._lean
            if abs(roll) > WIND_ROLL:
                rate = WIND_YAW_GAIN * (roll - math.copysign(WIND_ROLL, roll))
            elif pitch > WIND_PITCH:
                rate = WIND_RATE                        # wind from behind: turn round
            else:
                rate = 0.0
            rate = max(-WIND_RATE, min(WIND_RATE, rate))
            self._yaw_cmd = _wrap(self._yaw_cmd + rate * dt)
            setpoint.yawspeed, facing = rate, rate == 0.0
        setpoint.yaw = self._yaw_cmd
        off = math.hypot(self.hold[0] - pos[0], self.hold[1] - pos[1])
        settled = off < ALIGN_M and math.hypot(vel[0], vel[1]) < ALIGN_MPS and facing
        if not settled:
            self._aligned_at = None
        elif self._aligned_at is None:
            self._aligned_at = t
        done = self._aligned_at is not None and t - self._aligned_at >= ALIGN_HOLD_S
        timed_out = t - self._t0 > ALIGN_TIMEOUT_S and off < ARRIVE_M
        return setpoint, done or timed_out

    def _descend(self, t, pos, vel, armed, att):
        setpoint = Setpoint(x=self.hold[0], y=self.hold[1], vz=DESCENT_MPS, yaw=self._yaw_cmd)
        return setpoint, self.home[2] - pos[2] < LAND_FROM_M

    def _land(self, t, pos, vel, armed, att):
        """PX4's Land mode flies this phase; whoever calls update() switches PX4 to it."""
        return self._last, not armed

    # --- setpoints ---------------------------------------------------------------------------
    def _hover_over(self, point, t):
        return Setpoint(x=point[0], y=point[1], z=self.z_fly, yaw=self._heading(t))

    def _heading(self, t):
        """The commanded heading, turned towards its target no faster than TURN_RATE."""
        # capped, so a pause between calls cannot turn the heading in one step
        dt = 0.0 if self._t_yaw is None else min(0.1, max(0.0, t - self._t_yaw))
        self._t_yaw = t
        error = _wrap(self._target_yaw - self._yaw_cmd)
        self._yaw_cmd += max(-TURN_RATE * dt, min(TURN_RATE * dt, error))
        return self._yaw_cmd

"""The mission profile: what to command at each moment of the flight.

Climb and hover, then fly the route: for each waypoint, turn the nose towards it in a hover, speed
up, cruise, slow down, and hover over it. Then descend and land on the last waypoint. The vehicle
stops and turns at every waypoint instead of flying curves: in a hover the wing carries nothing,
so the lift rotors have the most yaw authority (docs/02_control_concept.md). The "transition" is
not a mode change: it is the commanded speed rising at the acceleration limit, and the "back
transition" is the same ramp downwards.

The simulation runner (psi_vtol/sim/run_case.py, over MAVLink) and the ROS 2 nodes
(ros2/ws/src/vtol_mission) both fly from this module, so what was simulated is what the vehicle
is asked to do. Nothing here knows about MAVLink or ROS.

Positions and velocities are in PX4's local frame: north, east, down. Times are the timestamps
of PX4's own messages in seconds, so a simulation running faster than the clock is flown
correctly; only differences between them are used.
"""
import math
from dataclasses import dataclass

NAN = math.nan
DESCENT_MPS = 1.5
LAND_FROM_M = 4.0               # below this height the landing is handed to PX4's Land mode
TURN_RATE = math.radians(20.0)  # how fast the commanded heading may change
TURN_TIME_S = 9.0               # extra hover to turn the nose into the wind
TURN_SETTLE_S = 2.0             # hover after a turn, for the heading to settle
ARRIVE_M = 1.0                  # a hover over a waypoint ends only this close to it
MIN_CRUISE_S = 2.0              # a short leg is flown slower, so it still holds its speed this long
# The vehicle stops about a metre past a waypoint. A hover starts where it stopped and moves its
# point onto the waypoint no faster than this: PX4 flies a step in the position setpoint with a
# hard tilt (20 degrees for 1 m in SITL).
APPROACH_MPS = 0.5

LANDING_PHASES = ("descend", "land")


@dataclass
class Setpoint:
    """One position/velocity setpoint. A NaN field is not commanded, as in PX4."""
    x: float = NAN
    y: float = NAN
    z: float = NAN
    vx: float = NAN
    vy: float = NAN
    vz: float = NAN
    yaw: float = NAN


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Profile:
    """Steps through the phases of the flight, one setpoint per call of update().

    p        the profile values of missions/<mission>/mission.yaml
    home     position where the vehicle stands when the profile is made; z is the ground level
    yaw0     heading at that moment; route_m is written in the frame of this heading
    wind_ne  mean wind (north, east), needed only by heading_into_wind; None when unknown
    part     "all", "path" (take-off and route) or "landing" (descend and land where it is)

    The phases are climb, hover1, then for leg i of the route: turn<i> (only when the heading
    changes), accel<i>, cruise<i>, decel<i> and hover<i+1> over its waypoint; then descend, land.
    """

    def __init__(self, p, home, yaw0, wind_ne=None, part="all"):
        self.alt, self.cruise, self.accel = p["alt_m"], p["cruise_mps"], p["accel_mps2"]
        self.hover_s = p["hover_s"]
        # Heading policy. Off: point the nose along each leg. On: point it along the velocity
        # relative to the air, as a weathervane would; there are then no turns to fly.
        self.into_wind = p["heading_into_wind"] >= 0.5
        if self.into_wind and wind_ne is None:
            raise ValueError("heading_into_wind needs the mean wind, and no wind estimate was given")
        self.hover1_s = self.hover_s + (TURN_TIME_S if self.into_wind else 0.0)
        self.home, self.yaw0 = tuple(home), yaw0
        self.wind_ne = (0.0, 0.0) if wind_ne is None else wind_ne
        self.z_fly = home[2] - self.alt
        self.legs = self._plan(p["route_m"])
        self._dir = self.legs[0]["dir"]     # direction of the leg being flown
        self._target_yaw = self._yaw_cmd = yaw0
        self._t_yaw = self._turned_at = None
        self._speed = 0.0                   # last commanded speed along the leg
        self._point, self._t_point = self.home[:2], None    # the point a hover holds

        if part == "landing":
            self.phases = LANDING_PHASES
        else:
            self.phases = ["climb", "hover1"]
            for i, leg in enumerate(self.legs, 1):
                self.phases += ([f"turn{i}"] if leg["turn"] else []) + [
                    f"accel{i}", f"cruise{i}", f"decel{i}", f"hover{i + 1}"]
            self.phases = tuple(self.phases) + (LANDING_PHASES if part == "all" else ())
        self.phase = self.phases[0]
        self.times = {}             # finished phases: name -> [start, end]
        self.hold = None            # the point descended over and landed on
        self.done = False
        self._t0 = None             # start of the current phase
        self._last = self.takeoff_setpoint()

    def _plan(self, route):
        """The legs of the route in the local frame: start, end, direction, heading, speed."""
        c, s = math.cos(self.yaw0), math.sin(self.yaw0)
        legs, start, heading = [], self.home[:2], self.yaw0
        for forward, left in route:
            end = (self.home[0] + forward * c + left * s, self.home[1] + forward * s - left * c)
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length < 1.0:
                raise ValueError(f"route leg to {[forward, left]} is shorter than 1 m")
            d = ((end[0] - start[0]) / length, (end[1] - start[1]) / length)
            leg_heading = math.atan2(d[1], d[0])
            turn = abs(_wrap(leg_heading - heading))
            # Speeding up and slowing down take v^2/a; MIN_CRUISE_S at v takes v*T. The fastest
            # speed that fits both in the leg solves v^2/a + v*T = length.
            a, T = self.accel, MIN_CRUISE_S
            fits = (-a * T + math.sqrt((a * T) ** 2 + 4 * a * length)) / 2
            legs.append({"start": start, "end": end, "dir": d, "length": length,
                         "heading": leg_heading, "turn_rad": turn,
                         "turn": turn > math.radians(1.0) and not self.into_wind,
                         "speed": min(self.cruise, fits)})
            start, heading = end, leg_heading
        if not legs:
            raise ValueError("the route has no waypoints")
        return legs

    def takeoff_setpoint(self):
        """Hold the flight height over home. PX4 wants this stream before it accepts offboard."""
        return Setpoint(x=self.home[0], y=self.home[1], z=self.z_fly, yaw=self.yaw0)

    def limit_s(self):
        """Longest the current phase may take; longer means the flight is not going as planned."""
        kind, leg = self._kind(), self._leg_of(self.phase)
        if kind in ("turn", "accel", "cruise", "decel"):
            ramp = leg["speed"] / self.accel
            return {"turn": leg["turn_rad"] / TURN_RATE + TURN_SETTLE_S + 5, "accel": ramp + 5,
                    "cruise": max(0.0, leg["length"] / leg["speed"] - ramp) + 10,
                    "decel": ramp + 25}[kind]
        if kind == "hover":
            return self.hover1_s + 5 if self.phase == "hover1" else self.hover_s + 15
        return {"climb": 60.0, "descend": self.alt / DESCENT_MPS + 20, "land": 60.0}[kind]

    def overdue(self, t):
        return self._t0 is not None and t - self._t0 > self.limit_s()

    def update(self, t, pos, vel, armed=True):
        """The setpoint for this moment. Moves on to the next phase when this one is finished."""
        if self.done:
            return self._last
        if self._t0 is None:
            self._enter(t, pos)
        setpoint, finished = getattr(self, "_" + self._kind())(t, pos, vel, armed)
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
        return self.phase.rstrip("0123456789")

    def _leg_of(self, phase):
        """The leg a turn, accel, cruise or decel phase belongs to; the hover after it, too."""
        number = int(phase[len(phase.rstrip("0123456789")):] or 0)
        return self.legs[max(0, number - (2 if phase.startswith("hover") else 1))]

    def _enter(self, t, pos):
        """The current phase starts now."""
        self._t0 = t
        kind = self._kind()
        if kind in ("turn", "accel"):
            leg = self._leg_of(self.phase)
            self._dir, self._target_yaw, self._turned_at = leg["dir"], leg["heading"], None
        if kind == "decel":
            self._speed_at_decel = self._speed
        if kind == "hover" and self.phase != "hover1":
            self._point, self._t_point = tuple(pos[:2]), None       # where it stopped
        if self.phase == f"hover{len(self.legs) + 1}":
            self.hold = (*self.legs[-1]["end"], self.z_fly)     # the route ends here: land on it
        if kind == "descend" and self.hold is None:
            self.hold = tuple(pos)                              # landing part alone: land where it is

    def _remaining(self, pos):
        leg = self._leg_of(self.phase)
        return (leg["end"][0] - pos[0]) * self._dir[0] + (leg["end"][1] - pos[1]) * self._dir[1]

    # --- one method per kind of phase: (setpoint, finished) ----------------------------------
    def _climb(self, t, pos, vel, armed):
        reached = self.home[2] - pos[2] > self.alt - 0.5 and abs(vel[2]) < 0.3
        return self.takeoff_setpoint(), reached

    def _hover(self, t, pos, vel, armed):
        if self.phase == "hover1":
            return self._hover_over(self.home, t), t - self._t0 > self.hover1_s
        point = self._leg_of(self.phase)["end"]
        arrived = math.hypot(point[0] - pos[0], point[1] - pos[1]) < ARRIVE_M
        return self._hover_over(self._approach(point, t), t), t - self._t0 > self.hover_s and arrived

    def _turn(self, t, pos, vel, armed):
        """Turn the nose towards the next waypoint over the point reached."""
        setpoint = self._hover_over(self._approach(self._leg_of(self.phase)["start"], t), t)
        if self._turned_at is None and abs(_wrap(self._target_yaw - self._yaw_cmd)) < 1e-9:
            self._turned_at = t
        return setpoint, self._turned_at is not None and t - self._turned_at >= TURN_SETTLE_S

    def _accel(self, t, pos, vel, armed):
        """The "transition": the commanded speed rises at the acceleration limit."""
        top = self._leg_of(self.phase)["speed"]
        self._speed = min(top, self.accel * (t - self._t0))
        elapsed = t - self._t0 > top / self.accel
        along = vel[0] * self._dir[0] + vel[1] * self._dir[1]
        reached = abs(along - top) <= max(0.5, 0.1 * top)
        braking = self._remaining(pos) <= top ** 2 / (2 * self.accel)
        return self._forward(self._speed, t), (elapsed and reached) or braking

    def _cruise(self, t, pos, vel, armed):
        """Hold the speed until the waypoint is one braking distance away."""
        braking = self._remaining(pos) <= self._speed ** 2 / (2 * self.accel)
        return self._forward(self._speed, t), braking

    def _decel(self, t, pos, vel, armed):
        """The "back transition": the commanded speed falls, then the vehicle holds a point."""
        self._speed = max(0.0, self._speed_at_decel - self.accel * (t - self._t0))
        return self._forward(self._speed, t), self._speed == 0.0 and math.hypot(vel[0], vel[1]) < 0.5

    def _descend(self, t, pos, vel, armed):
        setpoint = Setpoint(x=self.hold[0], y=self.hold[1], vz=DESCENT_MPS,
                            yaw=self._heading(0.0, t))
        return setpoint, self.home[2] - pos[2] < LAND_FROM_M

    def _land(self, t, pos, vel, armed):
        """PX4's Land mode flies this phase; whoever calls update() switches PX4 to it."""
        return self._last, not armed

    # --- setpoints ---------------------------------------------------------------------------
    def _approach(self, target, t):
        """The hover point, moved onto target no faster than APPROACH_MPS."""
        dt = 0.0 if self._t_point is None else min(0.1, max(0.0, t - self._t_point))
        self._t_point = t
        dn, de = target[0] - self._point[0], target[1] - self._point[1]
        distance = math.hypot(dn, de)
        if distance > 0.0:
            step = min(distance, APPROACH_MPS * dt)
            self._point = (self._point[0] + dn / distance * step, self._point[1] + de / distance * step)
        return self._point

    def _hover_over(self, point, t):
        return Setpoint(x=point[0], y=point[1], z=self.z_fly, yaw=self._heading(0.0, t))

    def _forward(self, speed, t):
        return Setpoint(z=self.z_fly, vx=speed * self._dir[0], vy=speed * self._dir[1],
                        yaw=self._heading(speed, t))

    def _heading(self, speed, t):
        """The commanded heading, turned towards its target no faster than TURN_RATE."""
        target = self._target_yaw
        if self.into_wind:
            air_n = speed * self._dir[0] - self.wind_ne[0]
            air_e = speed * self._dir[1] - self.wind_ne[1]
            target = math.atan2(air_e, air_n) if math.hypot(air_n, air_e) > 1.0 else None
        # capped, so the first call after the climb cannot turn the heading in one step
        dt = 0.1 if self._t_yaw is None else min(0.1, max(0.0, t - self._t_yaw))
        self._t_yaw = t
        if target is not None:
            error = _wrap(target - self._yaw_cmd)
            self._yaw_cmd += max(-TURN_RATE * dt, min(TURN_RATE * dt, error))
        return self._yaw_cmd

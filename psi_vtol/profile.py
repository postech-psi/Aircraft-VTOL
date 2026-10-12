"""The mission profile: what to command at each moment of the flight.

Climb, hover, speed up, cruise, slow down, hover, descend, land. The "transition" is not a mode
change: it is the commanded speed rising at the acceleration limit, and the "back transition" is
the same ramp downwards (docs/02_control_concept.md).

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

PATH_PHASES = ("climb", "hover1", "accel", "cruise", "decel", "hover2")
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


class Profile:
    """Steps through the phases of the flight, one setpoint per call of update().

    p        the profile values of missions/<mission>/mission.yaml
    home     position where the vehicle stands when the profile is made; z is the ground level
    yaw0     heading at that moment; the vehicle flies along it
    wind_ne  mean wind (north, east), used only by heading_into_wind
    phases   the part of the flight this object flies; the default is all of it
    """

    def __init__(self, p, home, yaw0, wind_ne=(0.0, 0.0), phases=PATH_PHASES + LANDING_PHASES):
        self.alt, self.cruise, self.accel = p["alt_m"], p["cruise_mps"], p["accel_mps2"]
        self.cruise_s, self.hover_s = p["cruise_s"], p["hover_s"]
        # Heading policy. Off: hold the first heading, which is the direction of travel.
        # On: point the nose along the velocity relative to the air, as a weathervane would.
        self.into_wind = p["heading_into_wind"] >= 0.5
        self.hover1_s = self.hover_s + (TURN_TIME_S if self.into_wind else 0.0)
        self.home, self.yaw0, self.wind_ne = tuple(home), yaw0, wind_ne
        self.z_fly = home[2] - self.alt
        self._cos0, self._sin0 = math.cos(yaw0), math.sin(yaw0)
        self._yaw_cmd, self._t_yaw = yaw0, None

        self.phases = tuple(phases)
        self.phase = self.phases[0]
        self.times = {}             # finished phases: name -> [start, end]
        self.hold = None            # the point hovered over after the slow-down, and landed on
        self.done = False
        self._t0 = None             # start of the current phase
        self._last = self.takeoff_setpoint()

    def takeoff_setpoint(self):
        """Hold the flight height over home. PX4 wants this stream before it accepts offboard."""
        return Setpoint(x=self.home[0], y=self.home[1], z=self.z_fly, yaw=self.yaw0)

    def limit_s(self):
        """Longest the current phase may take; longer means the flight is not going as planned."""
        ramp = self.cruise / self.accel
        return {"climb": 60.0, "hover1": self.hover1_s + 5, "accel": ramp + 5,
                "cruise": self.cruise_s + 5, "decel": ramp + 25, "hover2": self.hover_s + 5,
                "descend": self.alt / DESCENT_MPS + 20, "land": 60.0}[self.phase]

    def overdue(self, t):
        return self._t0 is not None and t - self._t0 > self.limit_s()

    def update(self, t, pos, vel, armed=True):
        """The setpoint for this moment. Moves on to the next phase when this one is finished."""
        if self.done:
            return self._last
        if self._t0 is None:
            self._enter(t, pos)
        setpoint, finished = getattr(self, "_" + self.phase)(t, pos, vel, armed)
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

    def _enter(self, t, pos):
        """The current phase starts now. The first phase that hovers or lands fixes the point."""
        self._t0 = t
        if self.hold is None and self.phase in ("hover2",) + LANDING_PHASES:
            self.hold = tuple(pos)

    # --- one method per phase: (setpoint, finished) ------------------------------------------
    def _climb(self, t, pos, vel, armed):
        reached = self.home[2] - pos[2] > self.alt - 0.5 and abs(vel[2]) < 0.3
        return self.takeoff_setpoint(), reached

    def _hover1(self, t, pos, vel, armed):
        return self._hover_over(self.home, t), t - self._t0 > self.hover1_s

    def _accel(self, t, pos, vel, armed):
        """The "transition": the commanded speed rises at the acceleration limit."""
        speed = min(self.cruise, self.accel * (t - self._t0))
        return self._forward(speed, t), t - self._t0 > self.cruise / self.accel

    def _cruise(self, t, pos, vel, armed):
        return self._last, t - self._t0 > self.cruise_s

    def _decel(self, t, pos, vel, armed):
        """The "back transition": the commanded speed falls, then the vehicle holds a point."""
        speed = max(0.0, self.cruise - self.accel * (t - self._t0))
        return self._forward(speed, t), speed == 0.0 and math.hypot(vel[0], vel[1]) < 0.5

    def _hover2(self, t, pos, vel, armed):
        return self._hover_over(self.hold, t), t - self._t0 > self.hover_s

    def _descend(self, t, pos, vel, armed):
        setpoint = Setpoint(x=self.hold[0], y=self.hold[1], vz=DESCENT_MPS,
                            yaw=self._heading(0.0, t))
        return setpoint, self.home[2] - pos[2] < LAND_FROM_M

    def _land(self, t, pos, vel, armed):
        """PX4's Land mode flies this phase; whoever calls update() switches PX4 to it."""
        return self._last, not armed

    # --- setpoints ---------------------------------------------------------------------------
    def _hover_over(self, point, t):
        return Setpoint(x=point[0], y=point[1], z=self.z_fly, yaw=self._heading(0.0, t))

    def _forward(self, speed, t):
        return Setpoint(z=self.z_fly, vx=speed * self._cos0, vy=speed * self._sin0,
                        yaw=self._heading(speed, t))

    def _heading(self, speed, t):
        if not self.into_wind:
            return self.yaw0
        air_n = speed * self._cos0 - self.wind_ne[0]
        air_e = speed * self._sin0 - self.wind_ne[1]
        # capped, so the first call after the climb cannot turn the heading in one step
        dt = 0.1 if self._t_yaw is None else min(0.1, max(0.0, t - self._t_yaw))
        self._t_yaw = t
        if math.hypot(air_n, air_e) > 1.0:
            error = math.atan2(air_e, air_n) - self._yaw_cmd
            error = math.atan2(math.sin(error), math.cos(error))
            self._yaw_cmd += max(-TURN_RATE * dt, min(TURN_RATE * dt, error))
        return self._yaw_cmd

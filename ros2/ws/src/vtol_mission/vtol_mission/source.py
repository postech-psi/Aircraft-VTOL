"""The base of every node that flies one step of the mission.

The contract with mission_manager and setpoint_mux:

    in   /mission/state                 the node is active while this equals its step name
    out  /<step>/trajectory_setpoint    setpoints, forwarded to PX4 by setpoint_mux
    out  /<step>/status                 ACTIVE:<detail>, DONE, or FAILED:<reason>

A subclass gives the step name and two methods: start(), called once when the step becomes
active, and step(), called at the setpoint rate while it is. PathNode and LandingNode are the
two steps of the year-end flight; a vision landing would be a third.
"""
import math

from px4_msgs.msg import (TrajectorySetpoint, VehicleCommand, VehicleLocalPosition,
                          VehicleStatus)
from rclpy.node import Node
from std_msgs.msg import String

from vtol_mission import px4

NANS = [math.nan, math.nan, math.nan]
PX4_MAIN_MODE_OFFBOARD = 6
PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LAND = 4, 6


class SetpointSource(Node):

    def __init__(self, step):
        super().__init__(f"{step.lower()}_node")
        self.step_name = step.upper()
        rate_hz = self.declare_parameter("rate_hz", 20.0).value

        # the vehicle, as PX4 last reported it
        self.t = None               # time of that report [s]; only differences are used
        self.pos = None             # north, east, down [m]
        self.vel = None
        self.heading = 0.0
        self.armed = False
        self.nav_state = None
        self.ground_z = 0.0         # the height the vehicle stood at before it was armed

        self.mission_state = "INACTIVE"
        self.active = False
        self.result = None          # "DONE" or "FAILED:<reason>" once the step is over

        prefix = f"/{step.lower()}"
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/trajectory_setpoint", px4.SETPOINTS)
        self.status_pub = self.create_publisher(String, f"{prefix}/status", 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, px4.topic(self, "vehicle_command"), px4.TO_PX4)
        self.create_subscription(VehicleLocalPosition, px4.topic(self, "local_position"),
                                 self._on_local_position, px4.FROM_PX4)
        self.create_subscription(VehicleStatus, px4.topic(self, "status"),
                                 self._on_vehicle_status, px4.FROM_PX4)
        self.create_subscription(String, "/mission/state", self._on_mission_state, 10)
        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"{self.step_name} node started.")

    # --- for the subclass ----------------------------------------------------------------------
    def start(self):
        """Called once when the step becomes active."""
        raise NotImplementedError

    def step(self):
        """Called at the setpoint rate while the step is active and not over."""
        raise NotImplementedError

    @property
    def offboard(self):
        return self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def send(self, sp):
        """Publish one setpoint (psi_vtol.profile.Setpoint)."""
        msg = TrajectorySetpoint()
        msg.timestamp = px4.now_us(self)
        msg.position = [float(sp.x), float(sp.y), float(sp.z)]
        msg.velocity = [float(sp.vx), float(sp.vy), float(sp.vz)]
        msg.acceleration = NANS
        msg.yaw = float(sp.yaw)
        msg.yawspeed = math.nan
        self.setpoint_pub.publish(msg)

    def command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.timestamp = px4.now_us(self)
        msg.command = command
        msg.param1, msg.param2, msg.param3 = float(param1), float(param2), float(param3)
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def ask_offboard(self):
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_MAIN_MODE_OFFBOARD)

    def ask_arm(self):
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def ask_land(self):
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, PX4_MAIN_MODE_AUTO,
                     PX4_SUB_MODE_AUTO_LAND)

    def report(self, text):
        self.status_pub.publish(String(data=text))

    def finish(self):
        self.get_logger().info(f"{self.step_name} done.")
        self.result = "DONE"

    def fail(self, reason):
        self.get_logger().error(f"{self.step_name} failed: {reason}")
        self.result = f"FAILED:{reason}"

    # --- machinery -----------------------------------------------------------------------------
    def _on_local_position(self, msg):
        self.t = msg.timestamp * 1e-6
        self.pos = (float(msg.x), float(msg.y), float(msg.z))
        self.vel = (float(msg.vx), float(msg.vy), float(msg.vz))
        self.heading = float(msg.heading)
        if not self.armed:
            self.ground_z = self.pos[2]

    def _on_vehicle_status(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.nav_state = msg.nav_state

    def _on_mission_state(self, msg):
        self.mission_state = msg.data.strip().upper()

    def _tick(self):
        if self.mission_state != self.step_name:
            self.active = False
            return
        if self.pos is None or self.nav_state is None:
            self.report("ACTIVE:waiting for PX4")
            return
        if not self.active:
            self.active, self.result = True, None
            self.start()
        if self.result is None:
            self.step()
        if self.result is not None:
            self.report(self.result)

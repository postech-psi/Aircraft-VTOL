"""Forwards the setpoints of one mission step to PX4.

This is the only node that publishes PX4's trajectory setpoint. mission_manager names the step
whose setpoints go through (/setpoint_mux/source). With no step selected, or when the selected
step stops sending, the vehicle is brought to a hover where it is (psi_vtol.profile.Stop): slowed
along its own direction of travel with the braking eased in, never told to be at a point it has
already passed. Holding the point it was at, at 5 m/s, pitched it to 48 degrees in SITL.
"""
import math
import time

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition
from rclpy.node import Node
from std_msgs.msg import String

from psi_vtol.profile import Stop, for_px4
from vtol_mission import px4



def has_finite(values):
    return any(math.isfinite(float(v)) for v in values)


class SetpointMux(Node):

    def __init__(self):
        super().__init__("setpoint_mux")
        self.steps = [s.upper() for s in self.declare_parameter("steps", ["PATH", "LANDING"]).value]
        # a step silent for this long is replaced by a position hold
        self.timeout_s = self.declare_parameter("setpoint_timeout_s", 0.5).value
        rate_hz = self.declare_parameter("rate_hz", 20.0).value

        self.source = "HOLD"
        self.latest = {}            # step -> (its last setpoint, when it arrived)
        self.position = None        # north, east, down
        self.velocity = (0.0, 0.0, 0.0)
        self.heading = 0.0
        self.stop = None            # the stop being flown while holding
        self.position_received_at = None
        self.last_position_timestamp = None
        self.state_timeout_s = self.declare_parameter("state_timeout_s", 1.0).value

        self.create_subscription(String, "/setpoint_mux/source", self.on_source, 10)
        for step in self.steps:
            self.create_subscription(TrajectorySetpoint, f"/{step.lower()}/trajectory_setpoint",
                                     lambda msg, step=step: self.on_setpoint(step, msg),
                                     px4.SETPOINTS)
        self.create_subscription(VehicleLocalPosition, px4.topic(self, "local_position"),
                                 self.on_local_position, px4.FROM_PX4)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, px4.topic(self, "offboard_control_mode"), px4.TO_PX4)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, px4.topic(self, "trajectory_setpoint"), px4.TO_PX4)
        self.status_pub = self.create_publisher(String, "/setpoint_mux/status", 10)
        # PX4 leaves offboard mode if this stream stops, so it runs from start-up to shutdown.
        self.create_timer(1.0 / rate_hz, self.forward)
        self.get_logger().info(f"Setpoint mux started for steps {self.steps}.")

    def on_source(self, msg):
        source = msg.data.strip().upper()
        if source not in self.steps + ["HOLD"]:
            self.get_logger().warn(f"Unknown mux source: {source}")
            return
        if source != self.source:
            self.get_logger().info(f"Mux source changed: {self.source} -> {source}")
            self.stop = None
        self.source = source

    def on_setpoint(self, step, msg):
        self.latest[step] = (msg, time.monotonic())

    def on_local_position(self, msg):
        if (self.last_position_timestamp is not None and msg.timestamp <= self.last_position_timestamp):
            return  # repeated old packets must not refresh the watchdog
        if not (msg.xy_valid and msg.z_valid and
                all(math.isfinite(v) for v in (msg.x, msg.y, msg.z, msg.heading))):
            self.position_received_at = None
            return
        self.position = [float(msg.x), float(msg.y), float(msg.z)]
        if msg.v_xy_valid and all(math.isfinite(v) for v in (msg.vx, msg.vy, msg.vz)):
            self.velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
        self.heading = float(msg.heading)
        self.position_received_at = time.monotonic()
        self.last_position_timestamp = msg.timestamp

    def hold_setpoint(self):
        """Come to a hover where the vehicle is. None until PX4 has reported a position."""
        if self.position is None:
            return None
        now = time.monotonic()
        if self.stop is None:
            self.stop = Stop(now, self.position, self.velocity, self.heading)
        sp = for_px4(self.stop.setpoint(now), self.position)
        msg = TrajectorySetpoint()
        msg.position = [sp.x, sp.y, sp.z]
        msg.velocity = [sp.vx, sp.vy, sp.vz]
        msg.acceleration = [sp.ax, sp.ay, sp.az]
        msg.yaw = sp.yaw
        msg.yawspeed = math.nan
        return msg

    def select(self):
        """The setpoint to send now, and a word on where it came from."""
        if self.source == "HOLD":
            return self.hold_setpoint(), "HOLD_SELECTED"
        setpoint, arrived = self.latest.get(self.source, (None, 0.0))
        if setpoint is None or time.monotonic() - arrived > self.timeout_s:
            return self.hold_setpoint(), f"{self.source}_STALE_HOLD"
        self.stop = None
        return setpoint, f"{self.source}_SELECTED"

    def forward(self):
        if (self.position_received_at is None or
                time.monotonic() - self.position_received_at > self.state_timeout_s):
            self.status_pub.publish(String(data="STALE_LOCAL_POSITION_STOP_STREAM"))
            return  # let the configured PX4 offboard-loss action take over
        selected, status = self.select()
        if selected is None:
            self.status_pub.publish(String(data="NO_LOCAL_POSITION"))
            return

        # PX4 is told which of position, velocity or acceleration the setpoint carries. A
        # setpoint with any position field is flown by the position controller, which also
        # follows the velocity fields on the axes whose position is NaN.
        mode = OffboardControlMode()
        mode.timestamp = px4.now_us(self)
        mode.position = has_finite(selected.position)
        mode.velocity = not mode.position and has_finite(selected.velocity)
        mode.acceleration = (not mode.position and not mode.velocity
                             and has_finite(selected.acceleration))
        if not (mode.position or mode.velocity or mode.acceleration):
            mode.position = True
        self.mode_pub.publish(mode)

        # a copy with a fresh timestamp, so that PX4 does not discard it as old
        out = TrajectorySetpoint()
        out.timestamp = px4.now_us(self)
        out.position = list(selected.position)
        out.velocity = list(selected.velocity)
        out.acceleration = list(selected.acceleration)
        out.yaw = float(selected.yaw)
        out.yawspeed = float(selected.yawspeed)
        self.setpoint_pub.publish(out)
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = SetpointMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

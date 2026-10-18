"""Runs the mission as a list of steps, one after another.

    WAIT_START -> step 1 -> step 2 -> ... -> MISSION_DONE        (FAILSAFE from anywhere)

For the active step it publishes the step's name on /mission/state, which switches that step's
node on, and on /setpoint_mux/source, which lets that node's setpoints through to PX4. It moves
on when the node reports DONE on /<step>/status and goes to FAILSAFE when it reports FAILED.

In FAILSAFE setpoint_mux brings the vehicle to a hover. Once it has stopped, the vehicle is handed
to PX4: Land when it cannot move forward (a pusher fault, or far behind the route: it could not
fly home either; psi_vtol.profile.needs_land), Hold otherwise, where PX4's own failsafes and the pilot take over. Only while PX4 still flies our
setpoints: a pilot or a PX4 failsafe that has taken over keeps the vehicle.

The steps are a parameter. A new kind of step (for example a vision landing) is a new node that
follows this contract (see source.py) and one more name in the list; this file does not change.

    ros2 topic pub --once /mission/start std_msgs/msg/String "{data: START}"
    ros2 topic pub --once /mission/abort std_msgs/msg/String "{data: ABORT}"
"""
import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import String

from psi_vtol.profile import needs_land
from vtol_mission import px4

WAIT_START, MISSION_DONE, FAILSAFE = "WAIT_START", "MISSION_DONE", "FAILSAFE"
PX4_MAIN_MODE_AUTO, PX4_SUB_MODE_AUTO_LOITER, PX4_SUB_MODE_AUTO_LAND = 4, 3, 6


class MissionManager(Node):

    def __init__(self):
        super().__init__("mission_manager")
        self.steps = [s.upper() for s in self.declare_parameter("steps", ["PATH", "LANDING"]).value]
        # a step silent for this long triggers a latched mission failure
        self.status_timeout_s = self.declare_parameter("status_timeout_s", 2.0).value
        # the mux's stop from 5 m/s takes 3.8 s (psi_vtol.profile.Stop)
        self.handover_s = self.declare_parameter("failsafe_handover_s", 6.0).value

        self.state = WAIT_START
        self.state_start = time.monotonic()
        self.step_status = ""       # last report of the active step's node
        self.step_status_time = 0.0
        self.failure = ""           # why the mission went to FAILSAFE
        self.nav_state = None       # PX4's flight mode, as it last reported it
        self.handover_sent = 0.0

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.source_pub = self.create_publisher(String, "/setpoint_mux/source", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
        self.command_pub = self.create_publisher(
            VehicleCommand, px4.topic(self, "vehicle_command"), px4.TO_PX4)
        self.create_subscription(VehicleStatus, px4.topic(self, "status"),
                                 self.on_vehicle_status, px4.FROM_PX4)
        for step in self.steps:
            self.create_subscription(String, f"/{step.lower()}/status",
                                     lambda msg, step=step: self.on_step_status(step, msg), 10)
        self.create_subscription(String, "/mission/start", self.on_start, 10)
        self.create_subscription(String, "/mission/abort", self.on_abort, 10)
        self.create_timer(0.1, self.loop)
        self.get_logger().info(f"Mission manager started. Steps: {' -> '.join(self.steps)}.")
        self.get_logger().info("Publish START to /mission/start to begin the mission.")

    def on_start(self, msg):
        if msg.data.strip().upper() == "START" and self.state == WAIT_START:
            self.change_state(self.steps[0])

    def on_abort(self, msg):
        command = msg.data.strip().upper()
        if command in ("ABORT", "STOP", "FAILSAFE"):
            self.get_logger().warn(f"Mission abort command received: {command}")
            self.fail_mission(f"aborted: {command}")

    def on_vehicle_status(self, msg):
        self.nav_state = msg.nav_state

    def fail_mission(self, reason):
        self.failure = reason
        self.change_state(FAILSAFE)

    def on_step_status(self, step, msg):
        if step == self.state:
            self.step_status = msg.data.strip()
            self.step_status_time = time.monotonic()

    def change_state(self, new_state):
        if new_state == self.state:
            return
        self.get_logger().info(f"Mission state changed: {self.state} -> {new_state}")
        self.state = new_state
        self.state_start = time.monotonic()
        self.step_status = ""
        self.step_status_time = 0.0

    def announce(self, mission_state, source, status):
        self.state_pub.publish(String(data=mission_state))
        self.source_pub.publish(String(data=source))
        self.status_pub.publish(String(data=status))

    def loop(self):
        if self.state == WAIT_START:
            self.announce("INACTIVE", "HOLD", WAIT_START)
        elif self.state == MISSION_DONE:
            self.announce(MISSION_DONE, "HOLD", MISSION_DONE)
        elif self.state == FAILSAFE:
            # The mux stops the vehicle; then PX4 gets it. PX4's own failsafes (RC loss,
            # battery, geofence) stay in charge of anything beyond that.
            self.announce(FAILSAFE, "HOLD", "FAILSAFE_HOLD")
            self.hand_over()
        else:
            self.run_step()

    def run_step(self):
        step = self.state
        self.announce(step, step, f"{step}_ACTIVE")

        last_heard = self.step_status_time or self.state_start
        if time.monotonic() - last_heard > self.status_timeout_s:
            self.get_logger().error(f"No status from the {step} node; entering FAILSAFE.")
            self.fail_mission(f"no status from the {step} node")
            return

        result = self.step_status.split(":", 1)[0]
        if result == "DONE":
            following = self.steps.index(step) + 1
            self.change_state(self.steps[following] if following < len(self.steps)
                              else MISSION_DONE)
        elif result == "FAILED":
            self.get_logger().error(f"{step} failed: {self.step_status}")
            self.fail_mission(self.step_status.split(":", 1)[-1])

    def hand_over(self):
        """Give the stopped vehicle to PX4's Land (no way forward) or Hold, while still offboard."""
        now = time.monotonic()
        if now - self.state_start < self.handover_s or now - self.handover_sent < 1.0:
            return
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return                  # already PX4's, the pilot's or never ours: leave it
        land = needs_land(self.failure)
        self.get_logger().warn(f"FAILSAFE ({self.failure}): handing over to PX4 "
                               f"{'Land' if land else 'Hold'}.")
        msg = VehicleCommand()
        msg.timestamp = px4.now_us(self)
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1, msg.param2 = 1.0, float(PX4_MAIN_MODE_AUTO)
        msg.param3 = float(PX4_SUB_MODE_AUTO_LAND if land else PX4_SUB_MODE_AUTO_LOITER)
        msg.target_system = msg.target_component = 1
        msg.source_system = msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)
        self.handover_sent = now


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
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

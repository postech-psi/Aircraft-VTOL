"""Runs the mission as a list of steps, one after another.

    WAIT_START -> step 1 -> step 2 -> ... -> MISSION_DONE        (FAILSAFE from anywhere)

For the active step it publishes the step's name on /mission/state, which switches that step's
node on, and on /setpoint_mux/source, which lets that node's setpoints through to PX4. It moves
on when the node reports DONE on /<step>/status and goes to FAILSAFE when it reports FAILED.

The steps are a parameter. A new kind of step (for example a vision landing) is a new node that
follows this contract (see source.py) and one more name in the list; this file does not change.

    ros2 topic pub --once /mission/start std_msgs/msg/String "{data: START}"
    ros2 topic pub --once /mission/abort std_msgs/msg/String "{data: ABORT}"
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

WAIT_START, MISSION_DONE, FAILSAFE = "WAIT_START", "MISSION_DONE", "FAILSAFE"


class MissionManager(Node):

    def __init__(self):
        super().__init__("mission_manager")
        self.steps = [s.upper() for s in self.declare_parameter("steps", ["PATH", "LANDING"]).value]
        # a step's node silent for this long gets a warning in the log
        self.status_timeout_s = self.declare_parameter("status_timeout_s", 2.0).value

        self.state = WAIT_START
        self.state_start = time.monotonic()
        self.step_status = ""       # last report of the active step's node
        self.step_status_time = 0.0

        self.state_pub = self.create_publisher(String, "/mission/state", 10)
        self.source_pub = self.create_publisher(String, "/setpoint_mux/source", 10)
        self.status_pub = self.create_publisher(String, "/mission/status", 10)
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
            # Hold the position. PX4's own failsafes (RC loss, battery, geofence) stay in charge
            # of anything beyond that.
            self.announce(FAILSAFE, "HOLD", "FAILSAFE_HOLD")
        else:
            self.run_step()

    def run_step(self):
        step = self.state
        self.announce(step, step, f"{step}_ACTIVE")

        last_heard = self.step_status_time or self.state_start
        if time.monotonic() - last_heard > self.status_timeout_s:
            self.get_logger().warn(f"No status from the {step} node.", throttle_duration_sec=2.0)

        result = self.step_status.split(":", 1)[0]
        if result == "DONE":
            following = self.steps.index(step) + 1
            self.change_state(self.steps[following] if following < len(self.steps)
                              else MISSION_DONE)
        elif result == "FAILED":
            self.get_logger().error(f"{step} failed: {self.step_status}")
            self.change_state(FAILSAFE)


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

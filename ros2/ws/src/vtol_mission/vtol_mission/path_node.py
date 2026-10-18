"""Mission step PATH: climb, hover, then fly the route: for each waypoint turn, speed up, cruise,
slow down, and hover over it.

The whole step is flown as a multicopter, on a planned reference: position, velocity,
acceleration and heading together. It never sends a VTOL transition command: this vehicle has no
control surfaces, and PX4's fixed-wing mode would lose it (docs/02_control_concept.md). The
reference is psi_vtol.profile, the same code the simulation flew. A fault the profile sees (no
forward speed, falling behind) fails the step with its reason; mission_manager acts on it.

Parameters
    mission    folder under missions/; its mission.yaml gives height, speed, ramp and route
    auto_arm   true: the node asks PX4 for offboard mode and arms. false: it waits for the
               pilot to do both, which is the default.
"""
import rclpy

from psi_vtol import config
from psi_vtol.profile import Profile
from vtol_mission.source import SetpointSource

ARM_AFTER_S = 1.5       # PX4 wants the setpoint stream for a moment before it accepts offboard
ARM_RETRY_S = 1.0
ARM_GIVE_UP_S = 60.0


class PathNode(SetpointSource):

    def __init__(self):
        super().__init__("PATH")
        self.mission = self.declare_parameter("mission", config.DEFAULT_MISSION).value
        self.auto_arm = self.declare_parameter("auto_arm", False).value
        self.flight = None
        self.flying = False         # armed and in offboard: the profile's clock is running
        self.t_start = self.t_asked = 0.0

    def start(self):
        try:                        # no wind estimate here: heading_into_wind cannot be flown
            self.flight = Profile(config.profile(self.mission), self.pos, self.heading,
                                  part="path")
        except ValueError as error:
            self.fail(str(error))
            return
        self.flying = False
        self.t_start, self.t_asked = self.t, -1e9
        self.get_logger().info(f"PATH: flying mission {self.mission}, home {self.pos}, "
                               f"heading {self.heading:.3f} rad.")

    def step(self):
        if not self.flying:
            self.wait_for_arming()
            return
        phase = self.flight.phase
        if not self.armed:
            self.fail(f"disarmed in {phase}")
        elif not self.offboard:
            self.fail(f"left offboard in {phase}")       # the pilot or a PX4 failsafe took over
        elif self.flight.overdue(self.t):
            self.fail(f"timeout in {phase}")
        else:
            self.send(self.flight.update(self.t, self.pos, self.vel, self.armed, self.att))
            if self.flight.fault:
                self.fail(self.flight.fault)
                return
            if self.flight.phase != phase:
                self.get_logger().info(f"PATH: {phase} -> {self.flight.phase}")
            if self.flight.done:
                self.get_logger().info(f"PATH phases [s, PX4 clock]: {self.flight.times}")
                self.finish()
            else:
                self.report(f"ACTIVE:{self.flight.phase}")

    def wait_for_arming(self):
        self.send(self.flight.takeoff_setpoint())
        waited = self.t - self.t_start
        if self.armed and self.offboard:
            self.flying = True
        elif self.auto_arm and waited > ARM_GIVE_UP_S:
            self.fail("could not arm in offboard")
        elif self.auto_arm and waited > ARM_AFTER_S and self.t - self.t_asked > ARM_RETRY_S:
            self.ask_offboard()
            self.ask_arm()
            self.t_asked = self.t
        self.report("ACTIVE:waiting for arming in offboard")


def main(args=None):
    rclpy.init(args=args)
    node = PathNode()
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

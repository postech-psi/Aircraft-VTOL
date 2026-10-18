"""Mission step LANDING: align over the point reached, descend, then let PX4's Land mode touch down.

Aligning settles the vehicle over the point and turns its nose into the wind, from the way it
leans to hold its place (psi_vtol.profile). No vision: the landing point is where the vehicle is
hovering when the step begins. A landing
on a detected target would be another step with the same contract (source.py), put in this
one's place in the list of steps.

Parameters
    mission    folder under missions/; its mission.yaml gives the flight profile
"""
import rclpy
from px4_msgs.msg import VehicleStatus

from psi_vtol import config
from psi_vtol.profile import Profile
from vtol_mission.source import SetpointSource

LAND_RETRY_S = 1.0


class LandingNode(SetpointSource):

    def __init__(self):
        super().__init__("LANDING")
        self.mission = self.declare_parameter("mission", config.DEFAULT_MISSION).value
        self.flight = None
        self.t_asked = 0.0

    def start(self):
        ground = (self.pos[0], self.pos[1], self.ground_z)
        try:                        # no wind estimate here: heading_into_wind cannot be flown
            self.flight = Profile(config.profile(self.mission), ground, self.heading,
                                  part="landing")
        except ValueError as error:
            self.fail(str(error))
            return
        self.t_asked = -1e9
        self.get_logger().info(f"LANDING: aligning over {self.pos[:2]}.")

    def step(self):
        phase = self.flight.phase
        if phase in ("align", "descend") and not self.armed:
            self.fail(f"disarmed in {phase}")
        elif phase in ("align", "descend") and not self.offboard:
            self.fail(f"left offboard in {phase}")      # the pilot or a PX4 failsafe took over
        elif self.flight.overdue(self.t):
            self.fail(f"timeout in {phase}")
        else:
            self.send(self.flight.update(self.t, self.pos, self.vel, self.armed, self.att))
            if self.flight.done:
                self.get_logger().info(f"LANDING phases [s, PX4 clock]: {self.flight.times}")
                self.finish()
                return
            landing = self.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            asked_recently = self.t - self.t_asked < LAND_RETRY_S
            if self.flight.phase == "land" and not landing and not asked_recently:
                self.get_logger().info("LANDING: handing over to PX4's Land mode.")
                self.ask_land()
                self.t_asked = self.t
            self.report(f"ACTIVE:{self.flight.phase}")


def main(args=None):
    rclpy.init(args=args)
    node = LandingNode()
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

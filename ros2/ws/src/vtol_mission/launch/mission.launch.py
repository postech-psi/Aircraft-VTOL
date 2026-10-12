"""Start the mission nodes: mission_manager, setpoint_mux and one node per step.

    ros2 launch vtol_mission mission.launch.py                  # the pilot arms, selects offboard
    ros2 launch vtol_mission mission.launch.py auto_arm:=true   # simulation
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The mission, in order. mission_manager runs the list and setpoint_mux listens to each entry.
STEPS = ["PATH", "LANDING"]


def generate_launch_description():
    mission = LaunchConfiguration("mission")
    auto_arm = ParameterValue(LaunchConfiguration("auto_arm"), value_type=bool)

    def node(executable, parameters):
        return Node(package="vtol_mission", executable=executable, name=executable,
                    output="screen", emulate_tty=True, parameters=[parameters])

    return LaunchDescription([
        DeclareLaunchArgument("mission", default_value="2026_year_end",
                              description="folder under missions/ with the flight profile"),
        DeclareLaunchArgument("auto_arm", default_value="false",
                              description="true: the path node asks for offboard and arms"),
        node("setpoint_mux", {"steps": STEPS}),
        node("mission_manager", {"steps": STEPS}),
        node("path_node", {"mission": mission, "auto_arm": auto_arm}),
        node("landing_node", {"mission": mission}),
    ])

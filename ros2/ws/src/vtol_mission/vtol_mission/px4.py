"""PX4 over uXRCE-DDS: the topic names and QoS every node uses.

The names are those of PX4 v1.17.0 (px4/README.md). PX4 appends _v<N> to the topic of a message
whose definition has a version above 0, so the names change between PX4 releases. This is the
one place they are written. Each is also a ROS parameter (px4_topic.<key>), so another firmware
or a namespace needs no change in the code.
"""
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

TOPICS = {
    "local_position": "/fmu/out/vehicle_local_position_v1",
    "status": "/fmu/out/vehicle_status_v1",
    "trajectory_setpoint": "/fmu/in/trajectory_setpoint",
    "offboard_control_mode": "/fmu/in/offboard_control_mode",
    "vehicle_command": "/fmu/in/vehicle_command",
}

# PX4's end of every topic is best-effort; a reliable endpoint would not connect to it.
TO_PX4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST, depth=1)
FROM_PX4 = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=1)
# Setpoints passed from a step's node to setpoint_mux travel with the same settings.
SETPOINTS = TO_PX4


def topic(node, key):
    """The name of one PX4 topic, as a parameter of the node."""
    return node.declare_parameter(f"px4_topic.{key}", TOPICS[key]).value


def now_us(node):
    """Timestamp for a message to PX4 [microseconds]."""
    return int(node.get_clock().now().nanoseconds / 1000)

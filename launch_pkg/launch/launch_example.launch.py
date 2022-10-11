
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# This function is always needed
def generate_launch_description():

    # Declare a variable Node for each node
    compute_node = Node(
        package="launch_pkg",
        executable="compute_node"
    )
    sensor_node = Node(
        package="launch_pkg",
        executable="sensor_node"
    )
    motor_node = Node(
        package="launch_pkg",
        executable="motor_node"
    )

    # Launch Foxglove Studio to monitor data
    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [compute_node,
          sensor_node,
          motor_node,
          foxglove_studio]

    return LaunchDescription(ld)
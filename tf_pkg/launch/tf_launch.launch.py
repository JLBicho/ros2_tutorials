# launch module includes elements to launch all types of processes and actions
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# launch_ros module includes elements to launch ROS 2 processes and actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



# This function is always needed
def generate_launch_description():

    # Declare a variable Node for each node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=["/home/jose/ws/ros2/foxglove_ws/src/ros2_tutorials/tf_pkg/urdf/robot.urdf"]
    )
    tf_broadcaster = Node(
        package="tf_pkg",
        executable="tf_broadcaster"
    )
    tf_listener = Node(
        package="tf_pkg",
        executable="tf_listener"
    )

    # Launch Foxglove Studio to monitor data
    foxglove_studio = ExecuteProcess(cmd=["foxglove-studio"])

    # Add the nodes and the process to the LaunchDescription list
    ld = [robot_state_publisher,
          tf_broadcaster,
          tf_listener,
          foxglove_studio]

    return LaunchDescription(ld)
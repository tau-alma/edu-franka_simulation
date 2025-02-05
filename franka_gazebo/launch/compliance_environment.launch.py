# Copyright (c) 2025 Tampere University, Autonomous Mobile Machines
#
# Licensed under the MIT License.
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Initialize Arguments
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("franka_gazebo"), "config", "franka_controllers.yaml"]
    )

    # Find the robot description file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("franka_description"), 
                "urdf",
                "effort_panda_arm_ft_sensor.urdf.xacro"]
            ),
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
        remappings=[("robot_description", "robot_description")]
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[initial_joint_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_impedance_example_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_impedance_example_controller", "--controller-manager", "/controller_manager"],
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "panda_arm",
            "-allow_renaming",
            "true",
        ],
    )

    world_file = os.path.join(
        get_package_share_directory('franka_gazebo'), 'config', 'ft_world.sdf')

    # launch arguments for Gazebo {args, world (- v 1 = log level)}
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": f" -r -v 1 {world_file}"}.items(),
    )

    gz_to_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=["/sensor_joint/force_torque@geometry_msgs/msg/Wrench@gz.msgs.Wrench"],
        output='screen'
    )

    nodes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        joint_impedance_example_controller_spawner,
        gz_spawn_entity,
        gz_launch_description,
        gz_to_ros_bridge,
        SetEnvironmentVariable('IGN_GAZEBO_VERBOSE', '1'),
    ]

    return LaunchDescription(nodes_to_start)
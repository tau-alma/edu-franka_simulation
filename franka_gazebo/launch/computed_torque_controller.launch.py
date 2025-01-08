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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    """
    Set up the launch configuration and nodes for the computed torque controller.

    Returns:
        list: A list of nodes to start.
    """
    # Initialize Arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    computed_torque_controller_spawner = LaunchConfiguration("computed_torque_controller")
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    # Find the robot description file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
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
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, initial_joint_controllers],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    computed_torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["computed_torque_controller", "--controller-manager", "/controller_manager"],
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

    # launch arguments for Gazebo {args, world (- v 1 = log level)}
    gz_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": " -r -v 1 empty.sdf"}.items(),
    )

    # Event handlers to ensure controller_manager is ready before spawners
    delay_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Event handlers to ensure controller is ready before spawners
    delay_computed_torque_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[computed_torque_controller_spawner],
        )
    )

    # Event handler to ensure Gazebo simulation is ready before launching the entity
    delay_gz_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[gz_launch_description],
        )
    )
    
    # Event handler to ensure Gazebo spawner is ready before spawning the entity
    delay_gz_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_manager_node,
            on_exit=[gz_spawn_entity],
        )
    )

    nodes_to_start = [
        robot_state_publisher_node,
        controller_manager_node,
        delay_joint_state_broadcaster_spawner,
        delay_computed_torque_controller_spawner,
        delay_gz_spawn_entity,
        delay_gz_launch,
    ]

    return nodes_to_start

def generate_launch_description():
    """
    Generate the launch description for the computed torque controller.

    This function declares the launch arguments and returns a LaunchDescription
    object that includes the necessary nodes and event handlers for launching
    the computed torque controller in a simulated environment.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="franka_gazebo",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="franka_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="franka_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="effort_panda_arm.urdf.xacro",
            description="URDF/XACRO description file with the robot with the effort interface.",
        ),
        DeclareLaunchArgument(
            "computed_torque_controller",
            default_value="computed_torque_controller",
            description="Robot controller",
        ),
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Sim ignition as physics engine",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

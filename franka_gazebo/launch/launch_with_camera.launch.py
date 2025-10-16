# file: launch/panda_with_cam.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, PathJoinSubstitution, TextSubstitution, EnvironmentVariable, FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    franka_gazebo_share = FindPackageShare("franka_gazebo")
    franka_description_share = FindPackageShare("franka_description")
    world_path = PathJoinSubstitution([franka_gazebo_share, "worlds", "cam_world2.sdf"])
    panda_with_cam_sdf = PathJoinSubstitution([franka_gazebo_share, "models", "panda_with_cam", "model.sdf"])
    models_dir = PathJoinSubstitution([FindPackageShare('franka_gazebo'), 'models'])
    controllers_yaml = PathJoinSubstitution([franka_gazebo_share, "config", "franka_controllers.yaml"])
    xacro_file = PathJoinSubstitution([franka_description_share, "urdf", "effort_panda_arm.urdf.xacro"])

    # --- URDF 
    urdf_cmd = Command([
        FindExecutable(name="xacro"), " ",
        xacro_file, " ",
        "sim_ignition:=true", " ",
        "hand:=false", " ",
        TextSubstitution(text="simulation_controllers:="), controllers_yaml,
    ])
    robot_description_param = ParameterValue(urdf_cmd, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_description_param,
        }],
        output="screen",
    )
  

    set_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]
    )
    
    

    # --- Start Gazebo (Fortress) via ros_gz_sim wrapper ---
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [TextSubstitution(text="-r -v 1 "), world_path]}.items(),
    )

    # --- Spawn the composed SDF (Panda + camera) after Gazebo starts ---
    spawn_panda_with_cam = TimerAction(
        period=3.0,  # small buffer for the server to initialize systems
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                "-file", panda_with_cam_sdf,
                "-name", "panda_with_cam",
                "-allow_renaming", "true",
            ],
        )]
    )

    # --- Spawners for controllers (Controller Manager comes from ign_ros2_control plugin) ---
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    # Replace 'computed_torque_controller' with your custom controller name(s) if different
    custom_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["computed_torque_clik_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # -- bridge the camera image to ROS ---
    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_bridge",
        arguments=[
            "/rgb_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
        output="screen",
    )
    
    # Publish trajectroy command to inverse-kinematic controller
    trajectory_cmd = Node(
        package='franka_gazebo',
        executable='tri_wave.py',
        output='screen'
    )
    
    # Aruco marker pose estimation node
    pose_estimate = Node(
        package='franka_gazebo',
        executable='pose_estimate.py',  
        output='screen',
        arguments=[
            '--ros-args',
            '-p', 'image_topic:=/rgb_camera/image',
            '-p', 'camera_info_topic:=/rgb_camera/camera_info',
            '-p', 'dictionary:=DICT_4X4_50',
            '-p', 'marker_id:=0',
            '-p', 'marker_length_m:=0.15',
        ],
    )

    return LaunchDescription([set_env,
    	trajectory_cmd,            # trajectory command node
        gz,                        # start Gazebo with your world
        robot_state_publisher,     # publish TF from the URDF
        spawn_panda_with_cam,      # spawn the composed SDF model
        joint_state_broadcaster,   # controller spawners (plugin provides controller_manager)
        custom_controller,
        camera_bridge,             # super necessary for pose estimation
        pose_estimate              # aruco marker pose estimation node    
    ])


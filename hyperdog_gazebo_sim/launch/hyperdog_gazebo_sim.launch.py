# Author: Addison Sears-Collins
# Date: September 19, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com
 
import os

from black import out
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch_ros.actions import (Node, SetParameter)

import xacro

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # LOADING URDF FILES #
    urdf_file = 'hyperdog.urdf.xacro'

    # Get Package Description and Directory #
    package_description = "hyperdog_gazebo_sim"
    package_directory = get_package_share_directory(package_description)

    # Set the Path to Robot Mesh Models for Loading in Gazebo Sim #
    # NOTE: Do this BEFORE launching Gazebo Sim #
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    # pkg_models_path = os.path.join(package_directory, "models") # add local models path
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))

    robot_desc_path = os.path.join(package_directory, "description", urdf_file)
    print("URDF Loaded !")

    # SETTING UP GAZEBO #
    gzsim_pkg = get_package_share_directory("ros_gz_sim")

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', Command(['xacro ', robot_desc_path]),
                   '-name', 'Hyperdog',
                   '-allow_renaming', 'true'],
    )

    # SETTING UP ROBOT #
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )
    
    hyperdog_gz_joint_ctrl_node = Node(
        package='hyperdog_gazebo_sim',
        executable='hyperdog_gazebo_joint_ctrl_node',
        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
            output='screen' )

    load_forward_command_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                'gazebo_joint_controller'],
            output='screen'
        )

    # ROS-Gazebo Bridge #
    ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            # "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            # "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            # "/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            # "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
        ],
        remappings=[
            # there are no remappings for this robot description
        ],
        output="screen",
    )

    # Create and Return the Launch Description Object #
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('ros_ign_gazebo'),
                                'launch', 'ign_gazebo.launch.py')]),
                launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=gz_spawn_entity,
                    on_exit=[load_joint_state_broadcaster],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_forward_command_controller],
                )
            ),
            # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo) #
            SetParameter(name="use_sim_time", value=True),
            robot_state_publisher_node,
            gz_spawn_entity,
            # hyperdog_gz_joint_ctrl_node,
            ign_bridge,
        ]
    )
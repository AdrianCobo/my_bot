import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_prefix
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name = 'my_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'robot_spawner.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true', 'output': 'screen'}.items()
    )

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # publish msgs from joystic topic (prio 1) and cmd_vels(prio 2) in /diff_cont/cmd_vel_unstamped
    twist_mux_params = os.path.join(get_package_share_directory(
        package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(package='twist_mux',
                     executable='twist_mux',
                     parameters=[twist_mux_params, {'use_sim_time': True}],
                     remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')])
                    
    twist_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    parameters=[{'use_sim_time': True}],
    remappings=[('/cmd_vel_in', '/diff_cont/cmd_vel_unstamped'),
                ('/cmd_vel_out', '/diff_cont/cmd_vel')]
    )
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'launch_gazebo.launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch them all!
    return LaunchDescription([
        LogInfo(msg='Starting Realistic Simulation'),
        LogInfo(msg='Crating URDF with Xacro and Robot State Publisher'),
        rsp,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting Joystick Node'),
        joystick,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting Twist_mux Node'),
        twist_mux,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting Twist_stamper Node'),
        twist_stamper,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting Gazebo Launcher'),
        gazebo,
        LogInfo(msg='Gazebo Launcher Done'),
        LogInfo(msg='Starting diff_drive controller'),
        diff_drive_spawner,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting joint broadcaster'),
        joint_broad_spawner,
        LogInfo(msg='Done'),
    ])

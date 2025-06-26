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

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  pkg_share_path

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(
                package_name), 'launch', 'real_robot_spawner.launch.py'
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

    # Include the Gazebo launch file, provided by the gazebo_ros package

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'my_world_empty.world'
    )

    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='world to load'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()# the last space is mandatory (before -v4)
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

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

    gz_ros_bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_ros_bridge.yaml')
    gz_ros_bridge = Node(package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=['--ros-args',
                    '-p',
                    f'config_file:={gz_ros_bridge_params}'])

    gz_ros_image_bridge = Node(package='ros_gz_image',
                    executable='image_bridge',
                    arguments=["/camera/image_raw"])

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
        LogInfo(msg='Starting Gazebo Sim'),
        world_arg,
        gazebo,
        LogInfo(msg='Done'),
        LogInfo(msg='Spawning robot at simulation'),
        spawn_entity,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting diff_drive controller'),
        diff_drive_spawner,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting joint broadcaster'),
        joint_broad_spawner,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting gz_ros_bridge'),
        gz_ros_bridge,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting gz_ros_image_bridge'),
        gz_ros_image_bridge,
        LogInfo(msg='Done'),
    ])

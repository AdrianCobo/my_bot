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

    package_name = 'my_bot'

    # add package_path so gazebo can find our 3D Models for the robot
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_name), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  pkg_share_path

    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world'
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
        LogInfo(msg='Starting Gazebo Simulation and Bridge'),
        LogInfo(msg='Starting Gazebo Sim'),
        world_arg,
        gazebo,
        LogInfo(msg='Done'),
        LogInfo(msg='Spawning robot at simulation'),
        spawn_entity,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting gz_ros_bridge'),
        gz_ros_bridge,
        LogInfo(msg='Done'),
        LogInfo(msg='Starting gz_ros_image_bridge'),
        gz_ros_image_bridge,
        LogInfo(msg='Done'),
    ])

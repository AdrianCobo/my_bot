import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

#TODO: use https://github.com/AdrianCobo/robot_localization

def generate_launch_description():
    use_sim_time=LaunchConfiguration('use_sim_time')

    odometry_node = Node(
        package='ros2_laser_scan_matcher',
        parameters=[{
                'base_frame': 'base_link', # robot base frame
                'odom_frame': 'odom_matcher', # frame used to publish the odometry (it is set where the robot started moving)
                'laser_frame': 'laser_frame', # frame where the lidar is
                'publish_odom': '/odom_matcher', # topic used for publishing the odometry, "" = dont publish it
                'publish_tf': False # publish the transform from odom_frame to base_frame False until I know how to mix 2 odometrys
            }],
        executable='laser_scan_matcher',
        name='odometry_publisher',
    )

    # Create and return the launch description
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                        description='Flag to enable use_sim_time'),
        odometry_node
    ])


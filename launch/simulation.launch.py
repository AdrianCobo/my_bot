# Copyright (c) 2021 PAL Robotics S.L.
# Modified by Adrián Cobo Merino
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from os import environ, pathsep
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource


def get_model_paths(packages_names):
    model_paths = ""
    for package_name in packages_names:
        if model_paths != "":
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, "share")

        model_paths += model_path

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ""
    for package_name in packages_names:
        if resource_paths != "":
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    return resource_paths


def generate_launch_description():
    my_bot = get_package_share_directory('my_bot')

    config = os.path.join(my_bot, 'config', 'map_params.yaml')

    with open(config, "r") as stream:
        try:
            conf = (yaml.safe_load(stream))

        except yaml.YAMLError as exc:
            print(exc)

    world_name = conf['my_bot']['world']
    world_name_arg = DeclareLaunchArgument(
        'world_name', default_value=world_name,
        description='World name'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    if not ("my_world" in world_name):

        # Default: Original path for PAL worlds
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('pal_gazebo_worlds'),
                'launch'), '/pal_gazebo.launch.py']),
        )

        model_path = ''
        resource_path = ''

        if 'GAZEBO_MODEL_PATH' in environ:
            model_path += pathsep + environ['GAZEBO_MODEL_PATH']

        if 'GAZEBO_RESOURCE_PATH' in environ:
            resource_path += pathsep + environ['GAZEBO_RESOURCE_PATH']

        ld.add_action(SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path))
        # Using this prevents shared library from being found
        # ld.add_action(SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', tiago_resource_path))
        ld.add_action(world_name_arg)
        ld.add_action(gazebo)

    else:
        map_path = os.path.join(my_bot, 'worlds', world_name + '.world')
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'use_sim_time': 'true',
                              'world': map_path}.items()
        )
        ld.add_action(gazebo)

    return ld

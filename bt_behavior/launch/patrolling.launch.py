# Copyright 2021 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pkg_dir = get_package_share_directory('bt_behavior')
    config_dir = os.path.join(pkg_dir, 'config')
    config_file = os.path.join(config_dir, 'waypoints.yaml')
    
    patrolling_cmd = Node(
        package='bt_behavior',
        executable='patrolling_main',
        parameters=[
          {'use_sim_time': True},
          config_file],
        remappings=[
          ('input_scan', '/scan_raw'),
          ('output_vel', '/nav_vel')
        ],
        output='screen'
    )

    ld = LaunchDescription()

    # Add any actions
    ld.add_action(patrolling_cmd)

    return ld

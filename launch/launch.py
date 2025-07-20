# Copyright 2025 WheelHub Intelligent
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('whi_motion_teleop'),
        'config',
        'config.yaml'
    ])

    return LaunchDescription([
        Node(
            package='whi_motion_teleop',
            executable='whi_motion_teleop_node',  # <-- must match your CMake target name!
            name='whi_motion_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[config],
        )
    ])

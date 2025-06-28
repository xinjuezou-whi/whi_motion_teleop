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

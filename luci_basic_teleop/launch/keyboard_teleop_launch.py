from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():


    use_cmd_vel = LaunchConfiguration('use_cmd_vel')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('luci_basic_bringup'),
                'launch',
                'luci_bringup.launch.py'
            )
        )
    )
    teleop_node = Node(
                        package='luci_basic_teleop',
                        executable='keyboard_teleop_node',
                        name='keyboard_teleop_node',
                        output='screen',
                        emulate_tty=True,
                        parameters=[{'use_cmd_vel': use_cmd_vel}]
                        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_cmd_vel',
            default_value='false',
            description='If true, publish Twist to /cmd_vel and enter AUTONAV'
        ),
        bringup_launch,
        teleop_node,
    ])
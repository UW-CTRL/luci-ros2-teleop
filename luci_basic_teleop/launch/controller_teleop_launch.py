from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():


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
                        executable='controller_control_node',
                        name='controller_node',
                        )
    joy_node = Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    output='screen'
                    )

    return LaunchDescription([
        bringup_launch,
        teleop_node,
        joy_node
    ])
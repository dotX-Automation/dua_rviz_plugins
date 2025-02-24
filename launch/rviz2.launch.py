import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_file = os.path.join(
        get_package_share_directory('dua_rviz_plugins'), 'config', 'config.rviz'
    )

    return launch.LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            additional_env={'XDG_RUNTIME_DIR': '/tmp/runtime-neo'}
        )
    ])

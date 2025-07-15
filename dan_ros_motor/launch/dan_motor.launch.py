from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dan_ros_motor')
    json_path = os.path.join(pkg_share, 'config/motor_param.json')

    return LaunchDescription([
        Node(
            package='dan_ros_motor',
            # executable='motor_node_daniel.py',
            executable='motor_daniel_pdo.py',
            name='dan_motor_node',
            output='screen'
        )
    ])

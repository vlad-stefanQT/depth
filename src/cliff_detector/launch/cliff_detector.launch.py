import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('cliff_detector'),
        'config',
        'params.yaml'
    )

    laserscan_node = Node(
        package='cliff_detector',
        executable='cliff_detector_node',
        parameters=[config],
        remappings=[
            ('/image', '/camera/depth/image_rect_raw'),
            ('/camera_info', '/camera/depth/camera_info'),
        ]
    )

    return launch.LaunchDescription([
        laserscan_node,
    ])
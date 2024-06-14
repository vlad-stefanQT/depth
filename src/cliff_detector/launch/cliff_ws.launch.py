import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    turn_node = Node(
        package='image_turn',
        executable='image_turn_node'
    )

    detector_config = os.path.join(
        get_package_share_directory('cliff_detector'),
        'config',
        'params.yaml'
    )

    detector_node = Node(
        package='cliff_detector',
        executable='cliff_detector_node',
        parameters=[detector_config],
        remappings=[
            ('/image', '/turned_image'),
            ('/camera_info', '/turned_camera_info'),
        ]
    )

    warning_config = os.path.join(
        get_package_share_directory('cliff_reaction'),
        'config',
        'params.yaml'
    )

    warning_node = Node(
        package='cliff_reaction',
        executable='cliff_reaction_node',
        parameters=[warning_config],
    )

    return launch.LaunchDescription([
        turn_node,
        detector_node,
        warning_node
    ])
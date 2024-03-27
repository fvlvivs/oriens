import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    params = os.path.join(
        get_package_share_directory('oriens_complementary_filter'),
        'params', 'complementary_filter_params.yaml')

    node = Node(
        package='oriens_complementary_filter',
        name='complementary_filter',
        executable='complementary_filter',
        parameters=[params]
    )

    ld.add_action(node)

    return ld

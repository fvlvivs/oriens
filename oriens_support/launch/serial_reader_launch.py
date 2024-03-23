import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    params = os.path.join(
        get_package_share_directory('oriens_support'),
        'params', 'serial_reader_params.yaml')

    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='Port name of the serial device'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='500000',
        description='Baud rate of the serial device'
    )

    period_arg = DeclareLaunchArgument(
        'period',
        default_value='5',
        description='Period of the serial reader node in milliseconds'
    )

    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='raw_data',
        description='Name of the topic to publish the serial data'
    )

    sensor_type_arg = DeclareLaunchArgument(
        'sensor_type',
        default_value='marg',
        description='imu/marg'
    )

    baud_rate = LaunchConfiguration(baud_rate_arg.name)
    port_name = LaunchConfiguration(port_name_arg.name)
    period = LaunchConfiguration(period_arg.name)
    topic_name = LaunchConfiguration(topic_name_arg.name)
    sensor_type = LaunchConfiguration(sensor_type_arg.name)

    substitutions = {
        'port_name': port_name,
        'baud_rate': baud_rate,
        'period': period,
        'topic_name': topic_name,
        'sensor_type': sensor_type
    }

    node = Node(
        package='oriens_support',
        executable='serial_reader',
        name='serial_reader',
        output='screen',
        parameters=[params, substitutions],
    )

    ld.add_action(port_name_arg)
    ld.add_action(baud_rate_arg)
    ld.add_action(period_arg)
    ld.add_action(topic_name_arg)
    ld.add_action(sensor_type_arg)
    ld.add_action(node)

    return ld

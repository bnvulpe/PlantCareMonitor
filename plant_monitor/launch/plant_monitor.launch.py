from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get the absolute path of the directory containing the current Python script
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # Get the absolute path of the parent directory of the current directory
    parent_dir = os.path.abspath(os.path.join(current_dir, os.pardir))

    # Construct the file path to the YAML configuration file relative to the parent directory
    parameters = os.path.join(parent_dir, 'conf', 'node_conf.yaml')
    return LaunchDescription([
        Node(
            package='plant_monitor',
            executable='sensor_manager',
            name='sensor_manager',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='decision_maker',
            name='decision_maker',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='light_act',
            name='light_act',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='heat_act',
            name='heat_act',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='ph_act',
            name='ph_act',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='water_act',
            name='water_act',
            parameters=[parameters]
        ),
        Node(
            package='plant_monitor',
            executable='simple_bag_recorder',
            name='simple_bag_recorder'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('realtime', default_value='false'),
        DeclareLaunchArgument('results_file', default_value='results.jsonl'),
        Node(package='tabletop_robot', executable='simulator', output='screen',
             parameters=[{'gui': ParameterValue(LaunchConfiguration('gui'), value_type=bool),
                          'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                          'realtime': ParameterValue(LaunchConfiguration('realtime'), value_type=bool)}]),
        Node(package='tabletop_robot', executable='perception', output='screen'),
        Node(package='tabletop_robot', executable='task_manager', output='screen',
             parameters=[{'results_file': LaunchConfiguration('results_file')}]),
    ])

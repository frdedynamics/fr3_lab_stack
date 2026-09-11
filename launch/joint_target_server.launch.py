from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    execute = LaunchConfiguration('execute')

    return LaunchDescription([
        DeclareLaunchArgument(
            'execute',
            default_value='false',
            description='Allow the server to send one ExecuteTrajectory goal per accepted request.',
        ),
        Node(
            package='fr3_lab_stack',
            executable='fr3_joint_target_server',
            name='fr3_joint_target_server',
            output='screen',
            parameters=[{
                'execute': ParameterValue(execute, value_type=bool),
            }],
        ),
    ])

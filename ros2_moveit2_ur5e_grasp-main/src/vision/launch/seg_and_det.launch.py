from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 声明 enable_tracking 参数，默认为 False（禁用跟踪）
        DeclareLaunchArgument(
            'enable_tracking',
            default_value='false',
            description='Enable object tracking (true/false)'
        ),

        Node(
            package='vision',
            executable='obj_detect',
            name='obj_detect',
            output='screen',
            parameters=[{
                'enable_tracking': LaunchConfiguration('enable_tracking')
            }]
        ),
        Node(
            package='vision',
            executable='det_tf',
            name='det_tf',
            output='screen'
        ),
        Node(
            package='vision',
            executable='point_cloud_processor',
            name='point_cloud_processor',
            output='screen'
        )
    ])
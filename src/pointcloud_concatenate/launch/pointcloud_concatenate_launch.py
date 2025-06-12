# launch/pointcloud_concatenate_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1) Launch arguments
    declared_args = [
        DeclareLaunchArgument('target_frame', default_value='world',
                              description='TF frame to transform clouds into'),
        DeclareLaunchArgument('cloud_in1', default_value='/camera_left/points'),
        DeclareLaunchArgument('cloud_in2', default_value='/camera_right/points'),
        DeclareLaunchArgument('clouds',   default_value='2',
                              description='Number of pointcloud topics'),
        DeclareLaunchArgument('hz',       default_value='10.0',
                              description='Update rate (Hz)'),
        DeclareLaunchArgument('cloud_out', default_value='cloud_concatenated'),
    ]



    # 3) Pointcloud concatenation node
    pc_concat = Node(
        package='pointcloud_concatenate',
        executable='pointcloud_concatenate_node',
        name='pc_concat',
        output='screen',
        parameters=[
            {'target_frame': LaunchConfiguration('target_frame')},
            {'clouds':       LaunchConfiguration('clouds')},
            {'hz':           LaunchConfiguration('hz')},
        ],
        remappings=[
            ('cloud_in1', LaunchConfiguration('cloud_in1')),
            ('cloud_in2', LaunchConfiguration('cloud_in2')),
            ('cloud_out', LaunchConfiguration('cloud_out')),
        ]
    )

    # 4) RViz2
    rviz_config = PathJoinSubstitution([
        FindPackageShare('pointcloud_concatenate'),
        'config', 'rviz', 'pointcloud_concatenate.rviz'
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pointcloud_concatenate_ui',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription(
        declared_args +
        [pc_concat, rviz_node]
    )

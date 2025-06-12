import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim  = get_package_share_directory('ros_gz_sim')
    pkg_self = get_package_share_directory('head_detector_sim')
    world    = os.path.join(pkg_self, 'worlds', 'two_cameras.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/camera_right@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_left@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_right/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera_left/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
    )

    # static TF for central cube
    static_tf_cube = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=[
            '0','0','0.5','0','0','0',
            'world','central_cube'
        ]
    )

    # camera_right_link now has no yaw (frame matches link)
    static_tf_r = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=[
            '2','0','1','3.14159','0','0',  # yaw=0
            'world','camera_right_link'
        ]
    )

    static_tf_l = Node(
        package='tf2_ros', executable='static_transform_publisher',
        output='screen',
        arguments=[
            '-2','0','1','0','0','0',
            'world','camera_left_link'
        ]
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2?'
    )
    rviz = Node(
        package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(pkg_self, 'rviz', 'person_demo.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        static_tf_cube,
        static_tf_r,
        static_tf_l,
        declare_rviz,
        rviz,
    ])

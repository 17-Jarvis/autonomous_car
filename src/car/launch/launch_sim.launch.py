import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Path to yolo_recognition package
    pkg_yolobot_recognition = get_package_share_directory('yolo_recognition')

    config_dir = os.path.join(get_package_share_directory('car'), 'config')
    config_file = os.path.join(config_dir, 'slam_toolbox_params.yaml')

    # Path to robot_state_publisher launch file
    package_name = 'car'  # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Include YOLO recognition launch file
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py')
        )
    )

    # SLAM Toolbox Node
    slam_toolbox = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'odom_frame': 'odom',  # Ensure this matches the published odometry frame
        'base_frame': 'base_footprint',  # Ensure this matches your robot's base link
        'scan_topic': 'scan',  # Ensure this matches your LIDAR topic
        'map_topic': 'map'
    }]
)

    # Launch all components
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        spawn_yolo,
        slam_toolbox,
    ])

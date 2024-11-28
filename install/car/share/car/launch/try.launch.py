import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to robot package and map
    pkg_car = get_package_share_directory('car')
    map_file = os.path.join(pkg_car, 'maps', 'new.yaml')  # Ensure this points to your map file

    # Path to robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_car, 'launch', 'rsp.launch.py')
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

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
    )

    # Static Transform Publisher (Optional: Adjust if necessary)
    static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']  # Adjust if required
    )

    # YOLO Recognition Node
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('yolo_recognition'), 'launch', 'launch_yolov8.launch.py')
        )
    )

    # AMCL (Localization Node) for localization in the map
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{'use_sim_time': True, 'param_file': os.path.join(pkg_car, 'config', 'amcl_params.yaml')}],
        remappings=[('/scan', '/robot_scan')]  # Remap to your robot's scan topic (if necessary)
    )
    

    # Launch all components
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        map_server,
        static_transform,
        spawn_yolo,
        amcl,  # Add AMCL for localization
    ])

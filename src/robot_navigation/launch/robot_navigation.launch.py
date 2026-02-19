import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # -------------------------
    # 1. Paths and Configs
    # -------------------------
    pkg_robot_navigation = get_package_share_directory('robot_navigation')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    nav_params_path = os.path.join(pkg_robot_navigation, 'config', 'navigation_params.yaml')
    obstacles_params_path = os.path.join(pkg_robot_navigation, 'config', 'obstacles.yaml')
    waypoints_path = os.path.join(pkg_robot_navigation, 'config', 'waypoints.yaml')
    rviz_config_path = os.path.join(pkg_robot_navigation, 'rviz', 'robot_navigation.rviz')
    
    # Define the model name
    model_name = 'turtlebot3_burger'

    # -------------------------
    # 2. Include TurtleBot3 Simulation Launch
    # -------------------------
    # It handles launching Gazebo, spawning the robot, and setting up the bridge and TFs.
    turtlebot3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'empty_world.launch.py')
        ),

        launch_arguments={'gui': 'false'}.items()
    )
    
    # -------------------------
    # 3. RViz Visualization
    # -------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='robot_navigation_rviz',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # -------------------------
    # 4. Your Custom Navigation Nodes
    # -------------------------
    waypoint_publisher = Node(
        package='robot_navigation',
        executable='waypoint_publisher_node',
        name='waypoint_publisher_node',
        output='screen',
        parameters=[
            obstacles_params_path,
            nav_params_path,
            {'waypoint_file_path': waypoints_path}
        ]
    )

    path_smoother = Node(
        package='robot_navigation',
        executable='path_smoother_node',
        name='path_smoother_node',
        output='screen',
        parameters=[nav_params_path, obstacles_params_path]
    )

    trajectory_tracker = Node(
        package='robot_navigation',
        executable='trajectory_tracker_node',
        name='trajectory_tracker_node',
        output='screen',
        parameters=[nav_params_path]
    )
    
    plotter_script = Node(
        package='robot_navigation',
        executable='plotter.py',
        name='path_plotter',
        output='screen'
    )

    # -------------------------
    # 5. Return LaunchDescription
    # -------------------------
    return LaunchDescription([
        turtlebot3_sim_launch,
        rviz_node,
        waypoint_publisher,
        path_smoother,
        trajectory_tracker,
        plotter_script
    ])


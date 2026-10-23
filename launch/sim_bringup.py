
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 1. Gazebo simulation
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'empty_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    # 2. RViz visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('nav'),
                'config',
                'sim.rviz'
            )
        ],
        parameters=[{
            'use_sim_time': True
        }]
    )

    # 3. Smooth path generation
    path_smoother = Node(
        package='nav',
        executable='smoothing',
        name='dynamic_smoothing_node',
        parameters=[{
            'use_sim_time': True,
            'path_file': (
                '/home/mbw/ros2_ws/src/nav/waypoints/waypoint.csv'
            ),
            'frame_id': 'odom',
            'path_resolution': 0.1,
        }]
    )

    # 4. MPPI controller
    mppi_controller = Node(
        package='nav',
        executable='mppi_controller',
        name='mppi_controller',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'samples': 300,
            'horizon_steps': 20,
            'dt': 0.1,
            'max_linear': 0.18,
            'max_angular': 1.0,
            'obstacle_margin': 0.40,
        }]
    )

    # 5. Launch each action exactly once.
    # Pure Pursuit is intentionally excluded.
    return LaunchDescription([
        turtlebot3_gazebo,
        path_smoother,
        mppi_controller,
        rviz,
    ])

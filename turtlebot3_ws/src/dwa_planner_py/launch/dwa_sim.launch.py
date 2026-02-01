from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Paths
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    dwa_pkg = get_package_share_directory('dwa_planner_py')

    world_file = os.path.join(
        dwa_pkg,
        'worlds',
        'dwa_test.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_pkg,
                'launch',
                'turtlebot3_world.launch.py'
            )
        ),
        launch_arguments={'world': world_file}.items()
    )


    # Spawn TurtleBot3
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_pkg,
                'launch',
                'spawn_turtlebot3.launch.py'
            )
        )
    )

    # DWA Planner Node
    dwa_node = Node(
        package='dwa_planner_py',
        executable='dwa_planner_node',
        name='dwa_planner',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_tb3,
        dwa_node
    ])

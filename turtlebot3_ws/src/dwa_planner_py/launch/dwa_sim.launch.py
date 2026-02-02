from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    dwa_pkg = get_package_share_directory('dwa_planner_py')

    world_file = os.path.join(
        dwa_pkg,
        'worlds',
        'obstacle.world'
    )

    # Start Gazebo with custom world
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
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

    # DWA Planner
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

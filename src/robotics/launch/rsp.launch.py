import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    xacro_file = os.path.join(get_package_share_directory('robotics'), 'urdf', 'robot.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {
        'robot_description': robot_description_config.toxml(),
        'use_sim_time': use_sim_time,
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )

    state_publisher = Node(
        package='robotics',
        executable='state_publisher',
        name='state_publisher',
        output='screen'
    )
    goal_subscriber = Node(
        package='robotics',
        executable='goal_subscriber',
        name='goal_subscriber',
        output='screen'
    )
    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('robotics'), 'rviz', 'config1.rviz')]
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time', default_value='false', description='Use sim time if true'
            ),
            node_robot_state_publisher,
            # state_publisher,
            goal_subscriber,
            rviz
        ]
    )

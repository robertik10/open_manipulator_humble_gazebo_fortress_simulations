from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', description='Start Gazebo UI?'),
        DeclareLaunchArgument(
            'paused', default_value='true', description='Start simulation paused?'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time (if true, /clock is published)'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/empty_world.launch.py']),
            launch_arguments={
                'world': get_package_share_directory('open_manipulator_gazebo') + '/worlds/empty.world',
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('open_manipulator_description'), '/launch/open_manipulator_upload.launch.py']),
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'open_manipulator', '-topic', 'robot_description', '-z', '0.0'],
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('open_manipulator_gazebo'), '/launch/controller_utils.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('open_manipulator_gazebo'), '/launch/open_manipulator_controller.launch.py']),
        ),
    ])
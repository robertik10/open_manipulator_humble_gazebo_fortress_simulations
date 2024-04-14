import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, AppendEnvironmentVariable
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
        
        # AppendEnvironmentVariable(
        # 'GZ_SIM_RESOURCE_PATH',
        # os.path.join(get_package_share_directory('open_manipulator_gazebo'),
        #              'models')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch/gz_sim.launch.py']),
            launch_arguments={
                'world': get_package_share_directory('open_manipulator_gazebo') + '/worlds/empty.world',
                'gui': LaunchConfiguration('gui'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
        # Load the URDF into the ROS Parameter Server 
        IncludeLaunchDescription(  #  open_manipulator_upload.launch.py needs to be inserted into launch directory of open_manipulator_x_description
            PythonLaunchDescriptionSource([get_package_share_directory('open_manipulator_x_description'), '/launch/open_manipulator_upload.launch.py']),
        ),
        
        # Load the bridge parameters from the config file
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={os.path.join(
                    get_package_share_directory('open_manipulator_gazebo'),
                    'params',
                    'open_manipulator_bridge.yaml'
                )}',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_sim', executable='create',  # TODO keep working here
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
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



# Directories:
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
pkg_open_manipulator_x_description = get_package_share_directory('open_manipulator_x_description')
pkg_open_manipulator_gazebo = get_package_share_directory('open_manipulator_gazebo')

# Arguments for launch description
ARGUMENTS = [DeclareLaunchArgument(
            'gui', default_value='true', description='Start Gazebo UI?'),
            DeclareLaunchArgument(
            'paused', default_value='-r', description='Start simulation paused?'), # change to " " if you want it paused
            DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='Use simulation time (if true, /clock is published)'),
            DeclareLaunchArgument(
                'world',
                default_value=os.path.join(
                        pkg_open_manipulator_gazebo,
                        'worlds',
                        'empty'),
                description='default SDF world file'),
            # DeclareLaunchArgument(
            #     'world',
            #     default_value='empty',
            #     description='default SDF world file'),
            ]

def generate_launch_description():

    # Directories:
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_open_manipulator_x_description = get_package_share_directory('open_manipulator_x_description')
    # pkg_open_manipulator_gazebo = get_package_share_directory('open_manipulator_gazebo')
    # pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    print(pkg_ros_gz_sim)

    # Paths
    path_bridge_params = os.path.join(
        pkg_open_manipulator_gazebo,
        'params',
        'open_manipulator_bridge.yaml')
    path_world = os.path.join(
        pkg_open_manipulator_gazebo,
        'worlds',
        'empty.world.sdf')

    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_open_manipulator_gazebo, 'worlds'), ':'])

    return LaunchDescription(
        ARGUMENTS + 
        [

        # Ignition gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim,
                                                       'launch',
                                                       'gz_sim.launch.py')),
            launch_arguments=[
                ('gz_args', [
                    LaunchConfiguration('world'),
                            '.sdf ',
                            #'-v 4 ',
                            #' --gui-config ',
                    LaunchConfiguration('paused'),
                    ])
            ]
        ),

        # Load the URDF into the ROS Parameter Server 
        IncludeLaunchDescription(  #  open_manipulator_upload.launch.py needs to be inserted into launch directory of open_manipulator_x_description
            PythonLaunchDescriptionSource(os.path.join(pkg_open_manipulator_x_description,
                                                       'launch',
                                                       'open_manipulator_upload.launch.py')),
        ),
        #Load the bridge parameters from the config file
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={path_bridge_params}',
            ],
            output='screen',
        ),
        
        # Create the robot model inside the world
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-entity', 'open_manipulator', '-topic', 'robot_description', '-z', '0.0'],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_open_manipulator_gazebo,
                                                       'launch', 'controller_utils.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_open_manipulator_gazebo,
                                                       'launch',
                                                       'open_manipulator_controller.launch.py')),
        ),
    ])
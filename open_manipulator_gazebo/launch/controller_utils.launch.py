# from launch import LaunchDescription
# from launch_ros.actions import Node
# import launch.actions
# import launch_ros.actions
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import ExecuteProcess

# def generate_launch_description():
#     return LaunchDescription([
#         launch_ros.actions.Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             parameters=[get_package_share_directory('open_manipulator_gazebo') + '/config/joint_state_controller.yaml'],
#             output='screen',
#             arguments=['--ros-args', '-r', '__node:=controller_manager'], #_joint_state_controller
#         ),
#         # launch.actions.ExecuteProcess(
#         #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
#         #     output='screen'
#         # ),
#         ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'joint_state_controller'],
#                     output='screen',
#                 ),
#         ExecuteProcess(
#             cmd=['ros2', 'control', 'set_controller_state', 'joint_state_controller', 'active'],
#             output='screen',
#         )
#     ])

import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the joint_state_controller.yaml file
    open_manipulator_gazebo_dir = get_package_share_directory('open_manipulator_gazebo')
    joint_state_controller_config = os.path.join(open_manipulator_gazebo_dir, 'config', 'joint_state_controller.yaml')

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[joint_state_controller_config],
            output='screen',
            name='controller_manager',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_controller'],
            output='screen',
        ),
    ])
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess, TimerAction
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='controller_manager',
#             executable='ros2_control_node',
#             parameters=[
#                 get_package_share_directory('open_manipulator_gazebo') + '/config/arm_controller.yaml',
#                 get_package_share_directory('open_manipulator_gazebo') + '/config/gripper_controller.yaml'
#             ],
#             output='screen',
#             arguments=['--ros-args', '-r', '__node:=controller_manager'],
#         ),
#         TimerAction(
#             period=5.0,
#             actions=[
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'joint1_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'joint1_position_controller', 'active'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'joint2_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'joint2_position_controller', 'active'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'joint3_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'joint3_position_controller', 'active'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'joint4_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'joint4_position_controller', 'active'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'gripper_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'gripper_position_controller', 'active'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'load_controller', 'gripper_sub_position_controller'],
#                     output='screen',
#                 ),
#                 ExecuteProcess(
#                     cmd=['ros2', 'control', 'set_controller_state', 'gripper_sub_position_controller', 'active'],
#                     output='screen',
#                 ),
#             ]
#         ),
#     ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Define the paths to the YAML files
    joint_state_controller_yaml = get_package_share_directory('open_manipulator_gazebo') + '/config/joint_state_controller.yaml'
    arm_controller_yaml = get_package_share_directory('open_manipulator_gazebo') + '/config/arm_controller.yaml'
    gripper_controller_yaml = get_package_share_directory('open_manipulator_gazebo') + '/config/gripper_controller.yaml'


    return LaunchDescription([
        # Node(
        #     package='open_manipulator_gazebo',
        #     executable='robot_description_subscriber',
        #     name='robot_description_subscriber',
        #     output='screen',
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='ros2_control_node',
        #     parameters=[
        #         #joint_state_controller_yaml,
        #         arm_controller_yaml,
        #         #gripper_controller_yaml
        #         #{'robot_description': }
        #     ],
        #     output='screen',
        #     name='controller_manager',
        #     remappings=[
        #     ("~/robot_description", "/robot_description"),],
        # ),
        # Node(
        #     package='controller_manager',
        #     executable='spawner',
        #     arguments=[
        #             'joint1_position',
        #             'joint2_position',
        #             'joint3_position',
        #             'joint4_position',
        #             'gripper_position',
        #             'gripper_sub_position',
        #             'joint_state_controller'
        #             ],
        #     output='screen',
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #         'joint1_position'],
        #     output='screen'),
        # ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #         'joint2_position'],
        #     output='screen'),
        # ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #         'joint3_position'],
        #     output='screen'),
        # ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #         'joint4_position'],
        #     output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'arm_controller'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'gripper_position'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'gripper_sub_position'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
            output='screen'),
        # ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
        #         'joint_position_controller'],
        #     output='screen'),
        Node(
            package='open_manipulator_gazebo',
            executable='omx_gripper_sub_publisher',
            output='screen',
            name='omx_gripper_sub_publisher',
        ),
    ])
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                get_package_share_directory('open_manipulator_gazebo') + '/config/arm_controller.yaml',
                get_package_share_directory('open_manipulator_gazebo') + '/config/gripper_controller.yaml'
            ],
            output='screen',
            arguments=['--ros-args', '-r', '__node:=controller_manager'],
        ),
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint1_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint1_position_controller', 'active'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint2_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint2_position_controller', 'active'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint3_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint3_position_controller', 'active'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint4_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'joint4_position_controller', 'active'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'gripper_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'gripper_position_controller', 'active'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'gripper_sub_position_controller'],
                    output='screen',
                ),
                ExecuteProcess(
                    cmd=['ros2', 'control', 'set_controller_state', 'gripper_sub_position_controller', 'active'],
                    output='screen',
                ),
            ]
        ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                get_package_share_directory('open_manipulator_gazebo') + '/config/arm_controller.yaml',
                get_package_share_directory('open_manipulator_gazebo') + '/config/gripper_controller.yaml'
            ],
            output='screen',
            arguments=['--ros-args', '-r', '__node:=controller_manager'],
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'joint1_position_controller'],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'joint2_position_controller'],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'joint3_position_controller'],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'joint4_position_controller'],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'gripper_position_controller'],
            output='screen'
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'control', 'load_start_controller', 'gripper_sub_position_controller'],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='open_manipulator_gazebo',
            executable='omx_gripper_sub_publisher',
            output='screen',
            name='omx_gripper_sub_publisher'
        ),
    ])
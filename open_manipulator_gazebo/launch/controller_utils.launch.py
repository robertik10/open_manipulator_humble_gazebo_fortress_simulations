from launch import LaunchDescription
from launch_ros.actions import Node
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[get_package_share_directory('open_manipulator_gazebo') + '/config/joint_state_controller.yaml'],
            output='screen',
            arguments=['--ros-args', '-r', '__node:=controller_manager'],
        ),
        # launch.actions.ExecuteProcess(
        #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
        #     output='screen'
        # ),
        ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', 'joint_state_controller'],
                    output='screen',
                ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'set_controller_state', 'joint_state_controller', 'active'],
            output='screen',
        )
    ])
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray
import time


import subprocess
import os
# from open_manipulator_msgs.srv import SetModelState
# from open_manipulator_msgs.msg import ModelState
from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.msg import JointPosition
import os
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

GRIPPER_OPEN_CONSTANT = 0.01
GRIPPER_CLOSED_CONSTANT = -0.01
DEFAULT_PATH_TIME_CONSTANT = 0.05 # was 0.05
DEFAULT_MAX_ACC_CONSTANT = 0.1 # was 0.1
DEFAULT_MAX_VEL_CONSTANT = 0.1 # was 0.1
DEFAULT_PLANNING_GROUP_CONSTANT = "arm"

class JointCommandPublisher(Node):

    def __init__(self):
        super().__init__('joint_command_publisher')
        self.gripper_publisher_ = self.create_publisher(Float64MultiArray, '/gripper_position/commands', 1)
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        # self.client = self.create_client(SetJointPosition, "/goal_joint_space_path")
        # while not self.client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        #self.check_absolute_position([2.0, 1.0, 1.0, 0.0])
        # for i in range(1000):
        #     self.check_gripper_2(True)
        #     time.sleep(1)
        #     self.check_gripper_2(False)
        #     time.sleep(1)        
        for i in range(1000):
            self.trajectory_test([[1.0, 0.0, 1.0, 0.0]])
            self.check_gripper_2(True)
            print("Trajectory test sent")
            time.sleep(10)
            self.trajectory_test([[0.0, 0.0, 0.0, 0.0]])
            self.check_gripper_2(False)
            print("Trajectory test sent")
            time.sleep(10)
        # self.trajectory_test([[0.0, 0.0, 0.0, 0.0]])
        #self.check_gripper([0])
    def check_gripper_2(self, gripper_state):
        msg = Float64MultiArray()
        if gripper_state:
            msg.data = [GRIPPER_OPEN_CONSTANT]
        else:
            msg.data = [GRIPPER_CLOSED_CONSTANT]
        self.gripper_publisher_.publish(msg)
        

    def send_joint_commands(self, commands):
        msg = Float64()
        msg.data = commands
        self.publisher_.publish(msg)
    
    def check_gripper(self, gripper_state):
        path_time=DEFAULT_PATH_TIME_CONSTANT
        max_acc=DEFAULT_MAX_ACC_CONSTANT
        max_vel=DEFAULT_MAX_VEL_CONSTANT
        planning_group=DEFAULT_PLANNING_GROUP_CONSTANT

        joint_name = ["gripper"]
        
        if gripper_state[0] > 0.0:
            joint_angle = [GRIPPER_OPEN_CONSTANT]
        else:
            joint_angle = [GRIPPER_CLOSED_CONSTANT]

        joint_position = JointPosition()
        joint_position.joint_name = joint_name
        joint_position.position = joint_angle
        joint_position.max_accelerations_scaling_factor = max_acc
        joint_position.max_velocity_scaling_factor = max_vel

        request = SetJointPosition.Request()
        request.planning_group = planning_group
        request.joint_position = joint_position
        request.path_time = path_time

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            return response.is_planned
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def check_absolute_position(self, joint_rad_positions):
        path_time=DEFAULT_PATH_TIME_CONSTANT
        max_acc=DEFAULT_MAX_ACC_CONSTANT
        max_vel=DEFAULT_MAX_VEL_CONSTANT
        planning_group=DEFAULT_PLANNING_GROUP_CONSTANT
        joint_name = ["joint1", "joint2", "joint3", "joint4"]
        #joint_position = JointPosition(joint_name, joint_rad_positions, max_acc, max_vel)
        joint_position = JointPosition()
        joint_position.joint_name = joint_name
        joint_position.position = joint_rad_positions
        joint_position.max_accelerations_scaling_factor = max_acc
        joint_position.max_velocity_scaling_factor = max_vel

        # Create a request
        request = SetJointPosition.Request()
        request.planning_group = planning_group
        request.joint_position = joint_position
        request.path_time = path_time

        # Send the request
        future = self.client.call_async(request)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            print("Response: %r" % response.is_planned)
            print(joint_rad_positions)
            return response.is_planned
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
    
    def trajectory_test(self, joint_rad_positions):
        # Create a JointTrajectory message
        trajectory = JointTrajectory()
        max_acc=DEFAULT_MAX_ACC_CONSTANT
        max_vel=DEFAULT_MAX_VEL_CONSTANT

        # Set the joint names
        trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4"]

        # Create a JointTrajectoryPoint message for the first point on the trajectory
        point1 = JointTrajectoryPoint()
        point1.positions = joint_rad_positions[0]  # Use the first set of positions from joint_rad_positions
        #point1.time_from_start = rclpy.time.Duration(seconds=1.0)
        point1.time_from_start = Duration(sec=1, nanosec=0)

        point1.accelerations = [max_acc, max_acc, max_acc, max_acc]
        point1.velocities = [max_vel, max_vel, max_vel, max_vel]
        # Add the first point to the trajectory
        trajectory.points.append(point1)

        # # Create a JointTrajectoryPoint message for the second point on the trajectory
        # point2 = JointTrajectoryPoint()
        # point2.positions = joint_rad_positions[1]  # Use the second set of positions from joint_rad_positions
        # point2.time_from_start = rclpy.time.Duration(seconds=2.0)

        # # Add the second point to the trajectory
        # trajectory.points.append(point2)

        # Now you can publish the trajectory message
        self.publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)

    joint_command_publisher = JointCommandPublisher()

    # # Replace with your desired joint commands
    # joint_commands = 1.0

    # joint_command_publisher.send_joint_commands(joint_commands)

    # Allow time for ROS2 to process the publication
    time.sleep(1)
    rclpy.spin(joint_command_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
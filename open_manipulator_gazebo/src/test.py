#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import threading
import math
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

GRIPPER_OPEN_CONSTANT = 0.01
GRIPPER_CLOSED_CONSTANT = -0.01
DEFAULT_PATH_TIME_CONSTANT = 0.05 # was 0.05
DEFAULT_MAX_ACC_CONSTANT = 0.1 # was 0.1
DEFAULT_MAX_VEL_CONSTANT = 0.1 # was 0.1
DEFAULT_PLANNING_GROUP_CONSTANT = "arm"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "gripper", "gripper_sub"]
JOINT_1_LIMITS = [-3.142, 3.142] # ["MIN", "MAX"]
JOINT_2_LIMITS = [-2.050, 1.571] # ["MIN", "MAX"]
JOINT_3_LIMITS = [-1.571, 1.530] # ["MIN", "MAX"]
JOINT_4_LIMITS = [-1.800, 2.000] # ["MIN", "MAX"]
class BotController(Node):
    # Constructor
    def __init__(self):
        super().__init__('bot_controller')

        self.joint_positions = [0.0, 0.0, 0.0, 0.0] # [joint1, joint2, joint3, joint4]
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0] # [joint1, joint2, joint3, joint4]
        self.gripper_open = False

        self.endeffector_position = [0.0, 0.0, 0.0]

        self.gripper_publisher_ = self.create_publisher(Float64MultiArray, '/gripper_position/commands', 1)
        self.joint_position_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 1)
        self.joint_state_sub = None

    # run method starts the subscriber (infinite loop until node is stopped)
    def run(self):
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        rclpy.spin(self)

#region Helper Methods

    def open_gripper(self, gripper_state):
        msg = Float64MultiArray()
        if gripper_state:
            msg.data = [GRIPPER_OPEN_CONSTANT]
        else:
            msg.data = [GRIPPER_CLOSED_CONSTANT]
        self.gripper_publisher_.publish(msg)   

    def set_joint_rad(self, joints_rad_position):
        if not self.check_joint_limits(joints_rad_position):
            return 

        # Create a JointTrajectory message
        trajectory = JointTrajectory()
        max_acc=DEFAULT_MAX_ACC_CONSTANT
        max_vel=DEFAULT_MAX_VEL_CONSTANT

        # Set the joint names
        trajectory.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        

        # Create a JointTrajectoryPoint message for the goal point on the trajectory
        goal_point = JointTrajectoryPoint()
        goal_point.positions = joints_rad_position
        goal_point.time_from_start = Duration(sec=1, nanosec=0)
        goal_point.accelerations = [max_acc, max_acc, max_acc, max_acc]
        goal_point.velocities = [max_vel, max_vel, max_vel, max_vel]

        # Add the goal point to the trajectory
        trajectory.points.append(goal_point)

        self.joint_position_pub.publish(trajectory)

    def joint_state_callback(self, msg):
        # create dictionary of joint names, positions, and velocities for reordering the message
        name_position_velocity_dict = {name: (position, velocity) for name, position, velocity in zip(msg.name, msg.position, msg.velocity)}
        # Use the correctly ordered list of joint names to order the message
        self.joint_positions = [name_position_velocity_dict[name][0] for name in JOINT_NAMES]
        self.joint_velocities = [name_position_velocity_dict[name][1] for name in JOINT_NAMES]

        # update gripper state
        if name_position_velocity_dict["gripper"][0] > 0.0:
            self.gripper_open = True
        else:
            self.gripper_open = False

    def get_joints_position(self):
        return self.joint_positions[0:4]

    def get_gripper_state(self):
        return self.gripper_open

    def wait_action_finished(self):
        while not all(math.isclose(velocity, 0.0, abs_tol=1e-3) for velocity in self.joint_velocities[0:4]):
            time.sleep(0.1)

    def check_joint_limits(self,joint_rad_positions):
        if joint_rad_positions[0] < JOINT_1_LIMITS[0] or joint_rad_positions[0] > JOINT_1_LIMITS[1] :
            print("Error: Joint 1 out of limits")
            return False
        if joint_rad_positions[1] < JOINT_2_LIMITS[0] or joint_rad_positions[1] > JOINT_2_LIMITS[1] :
            print("Error: Joint 2 out of limits")
            return False
        if joint_rad_positions[2] < JOINT_3_LIMITS[0] or joint_rad_positions[2] > JOINT_3_LIMITS[1] :
            print("Error: Joint 3 out of limits")
            return False
        if joint_rad_positions[3] < JOINT_4_LIMITS[0] or joint_rad_positions[3] > JOINT_4_LIMITS[1] :
            print("Error: Joint 4 out of limits")
            return False
        return True
#endregion Methods

#region Your code here
    def get_endeffector_position(self):
        pass

    def set_endeffector_position(self):
        pass
#endregion 

# Main function
def main(args=None):
    rclpy.init(args=args)

    # initialize the node
    bot_controller = BotController()
    thread = threading.Thread(target=bot_controller.run)
    thread.start()

    # Allow time for ROS2 to process the publication
    time.sleep(1)

    # Your Code Here: 
    for i in range(1000):
        bot_controller.set_joint_rad([2.0, 0.0, 0.0, 0.0])
        bot_controller.open_gripper(True)
        print("Trajectory test sent")
        bot_controller.wait_action_finished()
        print(bot_controller.get_joints_position())
        print(bot_controller.get_gripper_state())
        time.sleep(5)
        bot_controller.set_joint_rad([0.0, 0.0, 0.0, 0.0])
        bot_controller.open_gripper(False)
        print("Trajectory test sent")
        bot_controller.wait_action_finished()
        print(bot_controller.get_joints_position())
        print(bot_controller.get_gripper_state())
        time.sleep(5)
    
    # Destroy the node at the end of the program
    rclpy.shutdown()

if __name__ == '__main__':
    main()
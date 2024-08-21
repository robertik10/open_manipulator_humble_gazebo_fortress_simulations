#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
import time

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "gripper", "gripper_sub", "end_effector_joint"]
D1 = 0.077
THETA0 = np.deg2rad(np.arctan(0.024/0.128))  # Offset angle from the table ~11°
A2, A3, A4 = 0.130, 0.135, 0.126

JOINT_1_LIMITS = [-3.142, 3.142] # ["MIN", "MAX"]
JOINT_2_LIMITS = [-2.050, 1.571] # ["MIN", "MAX"]
JOINT_3_LIMITS = [-1.571, 1.530] # ["MIN", "MAX"]
JOINT_4_LIMITS = [-1.800, 2.000] # ["MIN", "MAX"]

class EEPosPublisher(Node):
    def __init__(self):
        super().__init__('end_effector_pos_publisher')
        
        #self.ee_pos_publisher = self.create_publisher(Float32MultiArray, 'end_effector_pos', 10)
        self.dirty_ee_pos_publisher = self.create_publisher(Pose, 'end_effector_pose', 10)
        self.dirty_cube_pos_publisher = self.create_publisher(Pose, 'cube_pose', 10)
        #self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.pos_publisher, 1)
        self.open_manipulator_pose_sub = self.create_subscription(PoseArray, 'open_manipulator_poses', self.open_manipulator_pose_callback, 1)

        
        
        # Test
        #self.inverse_kinematics_test_pub = self.create_publisher(Float32MultiArray, 'inverse_kinematics_test', 10)

        # ee_pos = [0.286, 0.0, 0.204] # initial end effector position
        # joints = [-10,-10,-10,-10] # initial wrong joint positions
        # ori = -90
        # while check_joint_limits(joints) == False:
        #     joints = inverse_kinematics(ee_pos, ee_ori=ori)
        #     ori += 0.1
        #     if ori > 90:
        #         break
        #     print("joints: ", joints)
        #     print("ori: ", ori)
        #     time.sleep(0.1)
        # while True:
        #     print("joints: ", joints)
        #     print("ori: ", ori)
        #     time.sleep(0.5)
        # ee_pos = forward_kinematics([0.0,0.0,0.0,0.0])
        # while True:
        #     print("ee_pos: ", ee_pos)

    def open_manipulator_pose_callback(self, msg):
        # publish the ee_pose each time a PoseArray message is received
        ee_pose = Pose()
        cube_pose = Pose()
        if len(msg.poses) < 11:
            #print("Error: Not enough poses in PoseArray message -> No end effector pose")
            return
        ee_pose.position = msg.poses[10].position
        ee_pose.orientation = msg.poses[10].orientation

        cube_pose.position = msg.poses[1].position
        cube_pose.orientation = msg.poses[1].orientation

        self.dirty_ee_pos_publisher.publish(ee_pose)
        self.dirty_cube_pos_publisher.publish(cube_pose)

    # publish the ee position each time a joint state message is received
    def pos_publisher(self, msg):
    
        ee_pos = Float32MultiArray()
        joint_positions = reorder_joint_list(msg)[0:4]

        ee_pos.data = forward_kinematics(joint_positions).astype(np.float32).tolist()

        self.ee_pos_publisher.publish(ee_pos)

        # test
        # joints = inverse_kinematics(ee_pos.data)
        # self.inverse_kinematics_test_pub.publish(Float32MultiArray(data=joints))
        # print(joints)

        pass

#TODO
def inverse_kinematics(ee_pos, ee_ori=0):
    # doesnt seem to work
    # caluclations according to:
    # https://www.researchgate.net/publication/353075915_Kinematic_Analysis_for_Trajectory_Planning_of_Open-Source_4-DoF_Robot_Arm#pf3

    x,y,z = ee_pos
    phi = np.deg2rad(ee_ori) # desired orientation of the end effector relative to the base frame
                             # phi = theta2 + theta3 + theta4

    # Calculate theta1 (base rotation)
    theta1 = np.arctan2(y, x)

    # Calculate r (projection on the x-y plane)
    r3 = np.sqrt(x**2 + y**2)
    z3 = z - D1

    r2 = r3 - A4 * np.cos(phi)
    z2 = z3 - A4 * np.sin(phi)

    # Calculate r2 and z2 squared to save computation time
    r2_sq = r2**2
    z2_sq = z2**2

    # Calculate theta3
    cos_theta3 = (r2_sq - z2_sq - A2**2 - A3**2) / (2 * A2 * A3)
    theta3 = -np.arccos(np.clip(cos_theta3, -1, 1)) # elbow up (elbow down is -theta3)

    # Calculate theta2 using the geometric approach
    cos_theta2 = (
                ((A2 + A3 * cos_theta3) * r2 + (A3*np.sin(theta3))) / 
                (r2_sq + z2_sq)
                 )
    sin_theta2 = (
                ((A2 + A3 * cos_theta3) * z2 - (A3*np.cos(theta3))) /
                (r2_sq + z2_sq)
                )
    theta2 = np.arctan2(sin_theta2, cos_theta2)

    # Calculate theta4 based on desired end-effector orientation phi
    theta4 = phi - (theta2 + theta3)

    return (theta1, theta2, theta3, theta4)

#TODO
def forward_kinematics(joint_positions):
    # expects 4 joint positions in radians
    # TODO Doesnt seem to work yet
    theta1, theta2, theta3, theta4 = joint_positions

    # DH parameters: (theta, d, a, alpha)
    dh_params = [
        [theta1, D1, 0, 90],
        [theta2 - THETA0, 0, A2, 0],
        [theta3 + THETA0, 0, A3, 0],
        [theta4, 0, A4, 0]
    ]

    # Compute the transformation matrices
    T1 = dh_transformation_matrix(*dh_params[0])
    T2 = dh_transformation_matrix(*dh_params[1])
    T3 = dh_transformation_matrix(*dh_params[2])
    T4 = dh_transformation_matrix(*dh_params[3])

    # Overall transformation matrix
    T = T1 @ T2 @ T3 @ T4

    #print("Overall Transformation Matrix from Base to End Effector:")
    #print(T)

    # return the position of the end effector (last column of the 4x4 matrix)
    return T[:3, 3] 

def dh_transformation_matrix(theta, d, a, alpha):
    # from paper: https://www.researchgate.net/publication/353075915_Kinematic_Analysis_for_Trajectory_Planning_of_Open-Source_4-DoF_Robot_Arm#pf3
    # page 772
    # theta = np.deg2rad(theta) theta already is in radians
    alpha = np.deg2rad(alpha)
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# helper function to reoder the joint list
def reorder_joint_list(msg):
    # create dictionary of joint names, positions, and velocities for reordering the message
    name_position_velocity_dict = {name: (position) for name, position in zip(msg.name, msg.position)}
    # Use the correctly ordered list of joint names to order the message
    joint_positions = [name_position_velocity_dict[name] for name in JOINT_NAMES]

    return joint_positions

# check whether the joint positions are within the allowable limits
def check_joint_limits(joint_rad_positions):
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

def main(args=None):
    rclpy.init(args=args)
    node = EEPosPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

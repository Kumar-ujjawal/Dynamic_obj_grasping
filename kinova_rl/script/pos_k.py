#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.optimize import least_squares
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler, quaternion_matrix
from sensor_msgs.msg import JointState

class Robot7DOF:
    def __init__(self):
        self.d_parameters = [0.2755, 0.2050, 0.2050, 0.2073, 0.1038, 0.1038, 0.1600, 0.0098]
        self.a_parameters = [0, 0, 0, 0, 0, 0, 0]
        self.alpha_parameters = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0]
        self.num_joints = 7
        self.initial_guess = None  # Store initial guess received from joint_states
        rospy.Subscriber('j2s7s300/joint_states', JointState, self.call_back)

    def dh_transformation(self, alpha, d, a, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def call_back(self, msg):
        self.initial_guess = msg.position

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i in range(self.num_joints):
            T = T @ self.dh_transformation(
                self.alpha_parameters[i],
                self.d_parameters[i],
                self.a_parameters[i],
                joint_angles[i]
            )
        return T

    def objective_function(self, joint_angles, target_pose):
        current_pose = self.forward_kinematics(joint_angles)
        position_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])
        orientation_error = np.linalg.norm(current_pose[:3, :3] - target_pose[:3, :3])
        return position_error + orientation_error

    def inverse_kinematics(self, target_pose):
        if self.initial_guess is None:
            initial_guess = np.zeros(self.num_joints)
        else:
            initial_guess = self.initial_guess
        
        result = least_squares(
            self.objective_function,
            initial_guess,
            args=(target_pose,),
            bounds=(-np.pi, np.pi)
        )
        
        return result.x

class KinovaController:
    def __init__(self):
        rospy.init_node('kinova_controller', anonymous=True)
        self.robot = Robot7DOF()
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
            "joint_finger_1", "joint_finger_2", "joint_finger_3"
        ]
        
        self.pub_joint_positions = {}
        for joint in self.joint_names:
            self.pub_joint_positions[joint] = rospy.Publisher(
                f'/j2s7s300/{joint}_position_controller/command', Float64, queue_size=10)
        
        rospy.loginfo("Publishers created")
        rospy.sleep(1.0)

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        # Create target pose matrix
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        target_pose = quaternion_matrix(quaternion)
        target_pose[:3, 3] = [x, y, z]

        # Calculate joint angles
        joint_angles = self.robot.inverse_kinematics(target_pose)

        rospy.loginfo(f"Moving to pose: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}")
        rospy.loginfo("Calculated joint angles:")
        for i, angle in enumerate(joint_angles):
            rospy.loginfo(f"Joint {i+1}: {angle:.4f} radians")

        # Publish commands to move each joint
        for i in range(len(self.joint_names)):
            if i < len(joint_angles):
                self.pub_joint_positions[self.joint_names[i]].publish(Float64(joint_angles[i]))
            else:
                # For finger joints, you may want to set a default value or keep them at their current position
                self.pub_joint_positions[self.joint_names[i]].publish(Float64(0.5))  # Example default value

        # Wait for movement to complete (adjust duration based on your robot's speed)
        rospy.sleep(5.0)

    def execute_goals(self):
        # Define goal poses (x, y, z, roll, pitch, yaw)
        goal_poses = [
            (3.5, 2.3, 2.7, 0, 0, 0),
            (1.4, -0.2, 0.6, np.pi/4, 0, np.pi/2),
            (3.3, 2.4, 0.5, 0, np.pi/4, 0),
            (2.6, 1.1, 2.8, np.pi/2, 0, np.pi/4),
            (3.2, -2.3, 1.4, 0, np.pi/2, 0)
        ]

        for i, pose in enumerate(goal_poses):
            rospy.loginfo(f"Executing Goal {i + 1}")
            self.move_to_pose(*pose)

        rospy.loginfo("Finished executing goals")

        # Stop joints (publish zero positions)
        for joint in self.joint_names:
            self.pub_joint_positions[joint].publish(Float64(0.0))
        rospy.loginfo("Stopping joints...")

if __name__ == '__main__':
    try:
        controller = KinovaController()
        controller.execute_goals()
    except rospy.ROSInterruptException:
        pass

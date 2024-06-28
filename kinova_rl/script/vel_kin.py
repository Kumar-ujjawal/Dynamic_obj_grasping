#!/usr/bin/env python3
import rospy
import numpy as np
from scipy.optimize import least_squares
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler, quaternion_matrix
from sensor_msgs.msg import JointState

class Robot7DOF:
    def __init__(self):
        self.d_parameters = [0.2755, 0.2050, 0.2050, 0.2073, 0.1038, 0.1038, 0.1600, 0.0098]
        self.a_parameters = [0, 0, 0, 0, 0, 0, 0]
        self.alpha_parameters = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0]
        self.num_joints = 7
        self.current_joint_angles = None
        self.current_position = None
        rospy.Subscriber('j2s7s300/joint_states', JointState, self.joint_state_callback)

    def joint_state_callback(self, msg):
        self.current_joint_angles = msg.position[:7]
        self.current_position = self.forward_kinematics(self.current_joint_angles)[:3, 3]

    def dh_transformation(self, alpha, d, a, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

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

    def calculate_jacobian(self, joint_angles):
        J = np.zeros((3, 7))
        T = np.eye(4)
        for i in range(7):
            T_i = self.dh_transformation(
                self.alpha_parameters[i],
                self.d_parameters[i],
                self.a_parameters[i],
                joint_angles[i]
            )
            T = np.dot(T, T_i)
            pos_i = T[:3, 3]
            z_i = T[:3, 2]
            J[:, i] = np.cross(z_i, (self.forward_kinematics(joint_angles)[:3, 3] - pos_i))
        return J

    def inverse_kinematics_velocity(self, target_position, time_frame=1.0):
        if self.current_joint_angles is None or self.current_position is None:
            rospy.logwarn("Current joint state not available. Skipping velocity calculation.")
            return None

        # Calculate end-effector velocity
        velocity = (target_position - self.current_position) / time_frame

        # Calculate Jacobian
        J = self.calculate_jacobian(self.current_joint_angles)

        # Calculate joint velocities using pseudo-inverse of Jacobian
        J_inv = np.linalg.pinv(J)
        joint_velocities = np.dot(J_inv, velocity)

        return joint_velocities

class KinovaController:
    def __init__(self):
        rospy.init_node('kinova_controller', anonymous=True)
        self.robot = Robot7DOF()
        
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7",
            "joint_finger_1", "joint_finger_2", "joint_finger_3"
        ]
        
        self.pub_joint_velocities = {}
        self.pub_joint_torques = {}
        for joint in self.joint_names[:7]:  # Only publish velocities and torques for the arm joints
            self.pub_joint_velocities[joint] = rospy.Publisher(
                f'/j2s7s300/{joint}_velocity_controller/command', Float64, queue_size=10)
            self.pub_joint_torques[joint] = rospy.Publisher(
                f'/j2s7s300/{joint}_effort_controller/command', Float64, queue_size=10)
        
        rospy.loginfo("Publishers created")
        rospy.sleep(1.0)

    def move_to_pose(self, x, y, z, roll, pitch, yaw):
        target_position = np.array([x, y, z])
        joint_velocities = self.robot.inverse_kinematics_velocity(target_position)

        if joint_velocities is not None:
            rospy.loginfo(f"Moving to pose: x={x}, y={y}, z={z}, roll={roll}, pitch={yaw}")
            rospy.loginfo("Calculated joint velocities:")
            for i, velocity in enumerate(joint_velocities):
                rospy.loginfo(f"Joint {i+1}: {velocity:.4f} rad/s")
                self.pub_joint_velocities[self.joint_names[i]].publish(Float64(velocity))

            # Wait for 2 seconds
            rospy.sleep(2.0)

            # Stop joints by publishing torques
            for i in range(len(self.joint_names[:7])):
                self.pub_joint_torques[self.joint_names[i]].publish(Float64(0.0))

        # Wait for movement to complete
        rospy.sleep(5.0)  # Assuming 1 second time frame

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

if __name__ == '__main__':
    try:
        controller = KinovaController()
        controller.execute_goals()
    except rospy.ROSInterruptException:
        pass

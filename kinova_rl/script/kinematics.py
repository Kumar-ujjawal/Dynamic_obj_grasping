#!/usr/bin.env/python3
import numpy as np
from scipy.optimize import least_squares

class Robot7DOF:
    def __init__(self):
        self.d_parameters = [0.2755, 0.2050, 0.2050, 0.2073, 0.1038, 0.1038, 0.1600, 0.0098]
        self.a_parameters = [0, 0, 0, 0, 0, 0, 0]
        self.alpha_parameters = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0]
        self.num_joints = 7

    def dh_transformation(self, alpha, d, a, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, joint_angles):
        T = np.eye(4)
        for i in range(7):
            T = T @ self.dh_transformation(
                self.alpha_parameters[i],
                self.d_parameters[i],
                self.a_parameters[i],
                joint_angles[i]
            )
        return T

    def objective_function(self, joint_angles, target_position):
        current_position = self.forward_kinematics(joint_angles)[:3, 3]
        return np.linalg.norm(current_position - target_position)

    def inverse_kinematics(self, target_position, initial_guess=None):
        if initial_guess is None:
            initial_guess = np.zeros(7)
        
        result = least_squares(
            self.objective_function,
            initial_guess,
            args=(target_position,),
            bounds=(-np.pi, np.pi)
        )
        
        return result.x

# Usage example
robot = Robot7DOF()

# Example target position in workspace
target_position = np.array([3.5, 2.3, 2.7])
# goal_poses = [
#             (, 0, 0, 0),
#             (1.4, -0.2, 0.6, np.pi/4, 0, np.pi/2),
#             (3.3, 2.4, 0.5, 0, np.pi/4, 0),
#             (2.6, 1.1, 2.8, np.pi/2, 0, np.pi/4),
#             (3.2, -2.3, 1.4, 0, np.pi/2, 0)
#         ]

# Calculate joint angles
joint_angles = robot.inverse_kinematics(target_position)

print("Calculated joint angles:")
for i, angle in enumerate(joint_angles):
    print(f"Joint {i+1}: {angle:.4f} radians")

# Verify the result
final_position = robot.forward_kinematics(joint_angles)[:3, 3]
print("\nFinal end-effector position:")
print(final_position)
print("\nPosition error:")
print(np.linalg.norm(final_position - target_position))
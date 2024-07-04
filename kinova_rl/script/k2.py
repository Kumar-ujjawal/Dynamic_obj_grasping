#!/usr/bin/env python3
import numpy as np
from scipy.optimize import least_squares

class Robot7DOF:
    def __init__(self):
        self.d_parameters = [-0.2755, 0, -(0.2050+0.2050), -0.2073, -(0.1038+0.1038), 0, -(0.1600+0.0098)]
        self.a_parameters = [0, 0, 0, 0, 0, 0, 0]
        self.alpha_parameters = [np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi]
        self.num_joints = 7
        self.joint_lower_limits, self.joint_upper_limits, self.joint_velocity_limits = self.get_joint_limits()

    def get_joint_limits(self):
        joint_lower_limits = [-2*np.pi, 47/180*np.pi, -2*np.pi, 30/180*np.pi, -2*np.pi, 65/180*np.pi, -2*np.pi]
        joint_upper_limits = [2*np.pi, 313/180*np.pi, 2*np.pi, 330/180*np.pi, 2*np.pi, 295/180*np.pi, 2*np.pi]
        joint_velocity_limits = [36/180*np.pi, 36/180*np.pi, 36/180*np.pi, 36/180*np.pi, 48/180*np.pi, 48/180*np.pi, 48/180*np.pi]
        return joint_lower_limits, joint_upper_limits, joint_velocity_limits

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
            theta = joint_angles[i]
            if i == 0:
                theta += np.pi  # Q1(physical) = Q1(DH algo) + 180
            elif i == 5:
                theta += np.pi/2  # Q6(physical) = Q6(DH algo) + 90
            T = T @ self.dh_transformation(
                self.alpha_parameters[i],
                self.d_parameters[i],
                self.a_parameters[i],
                theta
            )
        return T

    def objective_function(self, joint_angles, target_position):
        current_position = self.forward_kinematics(joint_angles)[:3, 3]
        return np.linalg.norm(current_position - target_position)

    def inverse_kinematics(self, target_position, initial_guess):
        if initial_guess is None:
            initial_guess = np.zeros(7)
        
        print("Initial guess:", initial_guess)
        
        # Ensure initial_guess is within bounds
        initial_guess = np.clip(initial_guess, self.joint_lower_limits, self.joint_upper_limits)
        
        print("Clipped initial guess:", initial_guess)
        
        result = least_squares(
            self.objective_function,
            initial_guess,
            args=(target_position,),
            bounds=(self.joint_lower_limits, self.joint_upper_limits)
        )
        
        if result.success:
            return result.x
        else:
            print("Optimization failed:", result.message)
            return None

    def is_valid_solution(self, joint_angles, target_position, tolerance=1e-3):
        current_position = self.forward_kinematics(joint_angles)[:3, 3]
        error = np.linalg.norm(current_position - target_position)
        return error < tolerance

    def inverse_kinematics_multi_attempt(self, target_position, initial_guess, num_attempts=5):
        for _ in range(num_attempts):
            solution = self.inverse_kinematics(target_position, initial_guess)
            if solution is not None and self.is_valid_solution(solution, target_position):
                return solution
            initial_guess = np.random.uniform(self.joint_lower_limits, self.joint_upper_limits)
        return None

    def apply_joint_limits(self, joint_angles):
        return np.clip(joint_angles, self.joint_lower_limits, self.joint_upper_limits)

    def apply_velocity_limits(self, joint_velocities):
        return np.clip(joint_velocities, -np.array(self.joint_velocity_limits), np.array(self.joint_velocity_limits))

    def calculate_jacobian(self, joint_angles):
        epsilon = 1e-6
        jacobian = np.zeros((6, 7))
        
        for i in range(7):
            joint_angles_plus = joint_angles.copy()
            joint_angles_plus[i] += epsilon
            joint_angles_minus = joint_angles.copy()
            joint_angles_minus[i] -= epsilon
            
            T_plus = self.forward_kinematics(joint_angles_plus)
            T_minus = self.forward_kinematics(joint_angles_minus)
            
            position_diff = (T_plus[:3, 3] - T_minus[:3, 3]) / (2 * epsilon)
            rotation_diff = (T_plus[:3, :3] - T_minus[:3, :3]) / (2 * epsilon)
            
            jacobian[:3, i] = position_diff
            jacobian[3:, i] = np.array([rotation_diff[2, 1], rotation_diff[0, 2], rotation_diff[1, 0]])
        
        return jacobian
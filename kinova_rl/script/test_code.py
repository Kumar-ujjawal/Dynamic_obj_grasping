import numpy as np
import csv
import random

# DH transformation function
def dh_transformation(alpha, d, a, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Forward kinematics function
def forward_kinematics(joint_angles, dh_params):
    T = np.eye(4)
    for i in range(7):
        T = T @ dh_transformation(dh_params[i][0], dh_params[i][1], dh_params[i][2], joint_angles[i])
    return T

# Calculate Jacobian function
def calculate_jacobian(joint_angles, dh_params):
    J = np.zeros((3, 7))
    T = np.eye(4)
    for i in range(7):
        T_i = dh_transformation(dh_params[i][0], dh_params[i][1], dh_params[i][2], joint_angles[i])
        T = np.dot(T, T_i)
        pos_i = T[:3, 3]
        z_i = T[:3, 2]
        J[:, i] = np.cross(z_i, (forward_kinematics(joint_angles, dh_params)[:3, 3] - pos_i))
    return J

# Function to generate random joint angles within limits
def generate_random_joint_angles():
    return [random.uniform(joint_limits[joint][0], joint_limits[joint][1]) for joint in joint_limits]

# DH parameters [alpha, d, a, theta]
dh_params = [
    [np.pi/2, 0.2755, 0, 0],
    [-np.pi/2, 0, 0.2050, 0],
    [np.pi/2, 0.2050, 0, 0],
    [-np.pi/2, 0.2073, 0, 0],
    [np.pi/2, 0.1038, 0, 0],
    [-np.pi/2, 0.1038, 0, 0],
    [0, 0.1600, 0, 0]
]

# Joint limits based on the URDF snippet provided
joint_limits = {
    'Joint1': (-2*np.pi, 2*np.pi),
    'Joint2': (47/180*np.pi, 313/180*np.pi),
    'Joint3': (-2*np.pi, 2*np.pi),
    'Joint4': (30/180*np.pi, 330/180*np.pi),
    'Joint5': (-2*np.pi, 2*np.pi),
    'Joint6': (65/180*np.pi, 295/180*np.pi),
    'Joint7': (-2*np.pi, 2*np.pi)
}

# Generate 20 sets of random joint velocities within limits
num_rows = 20
joint_velocities =  [
    [3.5, -15.5, 6.0, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 1
    [1.0, -13.5, 0.0, 5.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 2
    [0.0, 10.0, 3.5, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 3
    [0.0, 10.0, 0.0, 3.5, 0.0, 0.0, 0.0],  # Joint velocities for Goal 4
    [0.0, 10.0, 0.0, 0.0, 3.5, 0.0, 0.0]   # Joint velocities for Goal 5
]

# for _ in range(num_rows):
#     joint_angles = generate_random_joint_angles()
#     joint_velocities.append(joint_angles)

# Calculate corresponding end-effector velocities
end_effector_velocities = []
for joint_velocity in joint_velocities:
    J = calculate_jacobian(joint_velocity, dh_params)
    end_effector_velocity = np.dot(J, joint_velocity)
    end_effector_velocities.append(end_effector_velocity)

# Write joint velocities to j.csv
with open('j.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7'])
    writer.writerows(joint_velocities)

# Write end-effector velocities to m.csv
with open('m.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['Vx', 'Vy', 'Vz'])
    writer.writerows(end_effector_velocities)

print("CSV files 'j.csv' and 'm.csv' generated successfully!")

#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from kinematics import Robot7DOF  # Assuming you have a kinematics module
import numpy as np

class JointTrajectoryController:
    def __init__(self):
        rospy.init_node('joint_trajectory_client')
        self.client = actionlib.SimpleActionClient(
            'j2s7s300/effort_joint_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for joint trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Joint trajectory action server found.")

        self.joint_names = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3',
                            'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7']
        self.trajectory = JointTrajectory()
        self.trajectory.joint_names = self.joint_names
        self.current_joint_positions = None

        # Get joint limits
        self.joint_lower_limits, self.joint_upper_limits, self.joint_velocity_limits = self.get_joint_limits()

        # Subscriber for joint states
        self.joint_state_subscriber = rospy.Subscriber('/j2s7s300/joint_states', JointState, self.joint_state_callback)

    def get_joint_limits(self):
        # Define your joint limits as before
        joint_lower_limits = [-2*3.14, 47/180*3.14, -2*3.14, 30/180*3.14, -2*3.14, 65/180*3.14, -2*3.14]
        joint_upper_limits = [2*3.14, 313/180*3.14, 2*3.14, 330/180*3.14, 2*3.14, 295/180*3.14, 2*3.14]
        joint_velocity_limits = [36/180*3.14, 36/180*3.14, 36/180*3.14, 36/180*3.14, 48/180*3.14, 48/180*3.14, 48/180*3.14]
        return joint_lower_limits, joint_upper_limits, joint_velocity_limits

    def add_trajectory_point(self, positions, velocities, time_from_start):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = velocities
        point.time_from_start = rospy.Duration(time_from_start)
        self.apply_joint_limits(point)
        self.trajectory.points.append(point)
        rospy.loginfo(f"Added point: {point.positions}")

    def apply_joint_limits(self, point):
        # Apply joint limits to positions and velocities
        for i in range(len(point.positions)):
            if point.positions[i] < self.joint_lower_limits[i]:
                point.positions[i] = self.joint_lower_limits[i]
            elif point.positions[i] > self.joint_upper_limits[i]:
                point.positions[i] = self.joint_upper_limits[i]

            if abs(point.velocities[i]) > self.joint_velocity_limits[i]:
                point.velocities[i] = self.joint_velocity_limits[i] * (0.1 if point.velocities[i] > 0 else -0.1)

    def joint_state_callback(self, data):
        self.current_joint_positions = data.position

    def get_current_pose(self):
        if self.current_joint_positions is None:
            return None
        
        robot = Robot7DOF()
        end_effector_pose = robot.forward_kinematics(self.current_joint_positions)
        return end_effector_pose

    def execute_trajectory(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.trajectory
        goal.goal_time_tolerance = rospy.Duration(0.0)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Result: %s", self.client.get_result())

# if __name__ == '__main__':
#     try:
#         controller = JointTrajectoryController()

#         # Example usage
#         target_position = np.array([3.5, 2.3, 2.7])
#         robot = Robot7DOF()
#         positions = robot.inverse_kinematics(target_position)
#         controller.add_trajectory_point(positions, [0.2]*7, 9.0)

#         # Execute the trajectory
#         controller.execute_trajectory()

#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3
import rospy
import random
import numpy as np
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from kinematics import Robot7DOF  
from grasping import GripperController  
from trajectory import JointTrajectoryController  

class ArmController:
    def __init__(self):
        # rospy.init_node('arm_controller_node')
        self.controller = JointTrajectoryController()
        self.robot = Robot7DOF()
        self.grip = GripperController()
        self.joint_states = None
        self.link_pose = None

        # Subscribers
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.joint_states_subscriber = rospy.Subscriber('/j2s7s300/joint_states', JointState, self.joint_states_callback)

    def odom_callback(self, data):
        self.link_pose = data.pose.pose

    def joint_states_callback(self, data):
        self.joint_states = data

    def calculate_position_error(self, target_position, current_position):
        return np.linalg.norm(target_position - current_position)

    def move_to_link(self):
        if self.link_pose is None:
            rospy.logwarn("Link pose not received yet.")
            return False

        target_position = np.array([0.2,0.2,0.1
            # self.link_pose.position.x,
            # self.link_pose.position.y,
            # self.link_pose.position.z + 0.05  # Adjusted upward by 0.05 meters
        ])

        if self.joint_states is None:
            rospy.logwarn("Joint states not received yet.")
            return False

        initial_guess = np.array(self.joint_states.position[:7])
        joint_positions = self.robot.inverse_kinematics(target_position, initial_guess)
        curr = self.joint_states.position[:7]
        mov2pos = joint_positions - curr
        rospy.loginfo(f"the feed movement: {mov2pos}")
        # Calculate position error
        current_position = self.robot.forward_kinematics(joint_positions)
        error = self.calculate_position_error(target_position, current_position[:3, 3])
        rospy.loginfo(f"Position error after move_to_link: {error}")

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.velocities = [0.5] * 7
        point.time_from_start = rospy.Duration(3.0)
        
        # Apply joint limits and execute trajectory
        self.controller.apply_joint_limits(point)
        self.controller.add_trajectory_point(point.positions, point.velocities, 3.0)
        self.controller.execute_trajectory()
        
        return True

    def grasp_object(self):
        # current_pose = self.controller.get_current_pose()
        # if current_pose is None:
        #     rospy.logerr("Current pose not available.")
        #     return
        
        # current_position = current_pose[:3, 3]
        # current_position[2] -= 0.05  # Adjusted downward by 0.05 meters

        # if self.joint_states is None:
        #     rospy.logwarn("Joint states not received yet.")
        #     return
        
        # initial_guess = np.array(self.joint_states.position[:7])
        # joint_positions = self.robot.inverse_kinematics(current_position, initial_guess)
        
        # # Calculate position error
        # current_position_fk = self.robot.forward_kinematics(joint_positions)
        # error = self.calculate_position_error(current_position, current_position_fk[:3, 3])
        # rospy.loginfo(f"Position error after grasp_object: {error}")

        # # Create trajectory point
        # point = JointTrajectoryPoint()
        # point.positions = joint_positions.tolist()
        # point.velocities = [0.2] * 7
        # point.time_from_start = rospy.Duration(6.0)
        
        # # Apply joint limits and execute trajectory
        # self.controller.apply_joint_limits(point)
        # self.controller.add_trajectory_point(point.positions, point.velocities, 6.0)
        # self.controller.execute_trajectory()
        
        # Close gripper
        self.grip.close_gripper()
        rospy.loginfo("Grasping object")

    def move_to_random_place(self):
        random_position = np.array([
            random.uniform(0.5, 1.0),
            random.uniform(-0.5, 0.5),
            random.uniform(0.3, 0.7)
        ])

        if self.joint_states is None:
            rospy.logwarn("Joint states not received yet.")
            return

        initial_guess = np.array(self.joint_states.position[:7])
        joint_positions = self.robot.inverse_kinematics(random_position, initial_guess)
        curr = self.joint_states.position[:7]
        mov2pos = joint_positions - curr
        # Calculate position error
        current_position = self.robot.forward_kinematics(joint_positions)
        error = self.calculate_position_error(random_position, current_position[:3, 3])
        rospy.loginfo(f"Position error after move_to_random_place: {error}")

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.velocities = [0.2] * 7
        point.time_from_start = rospy.Duration(9.0)
        
        # Apply joint limits and execute trajectory
        self.controller.apply_joint_limits(point)
        self.controller.add_trajectory_point(point.positions, point.velocities, 9.0)
        self.controller.execute_trajectory()

    def release_object(self):
        rospy.loginfo("Releasing object")
        self.grip.open_gripper()

    def execute_task(self):
        rospy.loginfo("Waiting for link pose and joint states...")
        while not rospy.is_shutdown() and (self.link_pose is None or self.joint_states is None):
            rospy.sleep(0.1)

        if self.move_to_link():
            self.grasp_object()
            self.move_to_random_place()
            self.release_object()
            rospy.loginfo("Task completed successfully")
        else:
            rospy.logerr("Failed to move to link")

if __name__ == '__main__':
    try:
        arm_controller = ArmController()
        arm_controller.execute_task()
    except rospy.ROSInterruptException:
        pass

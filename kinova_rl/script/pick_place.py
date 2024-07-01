#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory import JointTrajectoryController  
import random
import numpy as np
from tf.transformations import euler_from_quaternion
from kinematics import Robot7DOF
from grasping import GripperController

class ArmController:
    def __init__(self):
        self.controller = JointTrajectoryController()
        self.link_pose = None
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.robot = Robot7DOF()
        self.grip = GripperController()

    def odom_callback(self, data):
        self.link_pose = data.pose.pose

    def move_to_link(self):
        if self.link_pose is None:
            rospy.logwarn("Link pose not received yet.")
            return False

        # Extract position from the pose
        target_position = np.array([
            self.link_pose.position.x,
            self.link_pose.position.y,
            self.link_pose.position.z
        ])
        
     
        target_position[2] += 0.05  

        # Use inverse kinematics to get joint positions
        robot = Robot7DOF()  
        joint_positions = self.robot.inverse_kinematics(target_position)
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()  # Convert numpy array to list
        point.velocities = [0.2] * 7  # Example velocities
        point.time_from_start = rospy.Duration(3.0)  # Example time from start
    
        self.controller.apply_joint_limits(point)
        self.controller.add_trajectory_point(point.positions, point.velocities, 3.0)
        self.controller.execute_trajectory()
        
        # Add trajectory point to move to link
        # self.controller.add_trajectory_point(joint_positions, [0.2]*7, 3.0)
        # self.controller.apply_joint_limits(joint_positions)
        # self.controller.execute_trajectory()
        return True

    def grasp_object(self):
        # Move slightly down to grasp the object
        current_pose = self.controller.get_current_pose()  # Assume this method exists
        current_position = current_pose[:3,3]
        current_position[2] -= 0.05  # Move down by the object height
        
        joint_positions = self.robot.inverse_kinematics(current_position)
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()  # Convert numpy array to list
        point.velocities = [0.2] * 7  # Example velocities
        point.time_from_start = rospy.Duration(6.0)  # Example time from start
    
        self.controller.apply_joint_limits(point)
        self.controller.add_trajectory_point(point.positions, point.velocities, 6.0)
        self.controller.execute_trajectory()
        self.grip.close_gripper()
        

        # Simulate grasping by closing the gripper
        rospy.loginfo("Grasping object")
        # Add gripper closing action here

    def move_to_random_place(self):
        # Generate random position within arm's reach
        random_position = np.array([
            random.uniform(0.5, 1.0),
            random.uniform(-0.5, 0.5),
            random.uniform(0.3, 0.7)
        ])
        
        
        joint_positions = self.robot.inverse_kinematics(random_position)
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()  
        point.velocities = [0.2] * 7  
        point.time_from_start = rospy.Duration(9.0)  
    
        self.controller.apply_joint_limits(point)
        self.controller.add_trajectory_point(point.positions, point.velocities, 9.0)
        self.controller.execute_trajectory()
        

    def release_object(self):
        
        rospy.loginfo("Releasing object")
        self.grip.open_gripper

    def execute_task(self):
        rospy.loginfo("Waiting for link pose...")
        while not rospy.is_shutdown() and self.link_pose is None:
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
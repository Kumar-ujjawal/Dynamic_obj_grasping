#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperController:
    def __init__(self):
        # rospy.init_node('gripper_controller')
        
        self.client = actionlib.SimpleActionClient(
            '/j2s7s300/effort_finger_trajectory_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for finger trajectory action server...")
        self.client.wait_for_server()
        rospy.loginfo("Finger trajectory action server found!")

        self.finger_joint_names = [
            'j2s7s300_joint_finger_1',
            'j2s7s300_joint_finger_2',
            'j2s7s300_joint_finger_3'
        ]

    def move_gripper(self, positions, duration=2.0):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.finger_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(duration)
        
        goal.trajectory.points.append(point)
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        result = self.client.get_result()
        if result.error_code != 0:
            rospy.logerr(f"Gripper movement failed with error code: {result.error_code}")
        else:
            rospy.loginfo("Gripper movement completed successfully")

    def open_gripper(self):
        open_positions = [0.0, 0.0, 0.0]  
        self.move_gripper(open_positions)

    def close_gripper(self):
        close_positions = [0.50, 0.50, 0.50]  
        self.move_gripper(close_positions)

if __name__ == '__main__':
    try:
        gripper = GripperController()
        
        # Example usage
        rospy.loginfo("Opening gripper...")
        gripper.open_gripper()
        rospy.sleep(2)  # Wait for 2 seconds
        
        rospy.loginfo("Closing gripper...")
        gripper.close_gripper()
        
    except rospy.ROSInterruptException:
        pass
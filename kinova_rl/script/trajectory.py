#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def main():
    rospy.init_node('joint_trajectory_client')

    # Define the action client
    client = actionlib.SimpleActionClient('j2s7s300/effort_joint_trajectory_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for joint trajectory action server...")
    client.wait_for_server()
    rospy.loginfo("Joint trajectory action server found.")

    # Define joint names (should match your robot's joint names)
    joint_names = ['j2s7s300_joint_1', 'j2s7s300_joint_2', 'j2s7s300_joint_3',
                   'j2s7s300_joint_4', 'j2s7s300_joint_5', 'j2s7s300_joint_6', 'j2s7s300_joint_7']

    # Create a trajectory message
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    # Create a point to specify the trajectory
    point = JointTrajectoryPoint()
    point.positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  # Example positions for each joint
    point.velocities = [0.0] * 7  # Example velocities for each joint
    point.time_from_start = rospy.Duration(3.0)  # Time to reach this point (in seconds)

    # Assign the point to the trajectory
    trajectory.points.append(point)

    # Create a goal to send to the action server
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory
    goal.goal_time_tolerance = rospy.Duration(0.0)

    # Send the goal
    client.send_goal(goal)

    # Wait for the action to finish (optional)
    client.wait_for_result()

    # Report the result (optional)
    rospy.loginfo("Result: %s", client.get_result())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

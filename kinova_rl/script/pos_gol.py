#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from time import sleep

# Define the joint names
joint_names = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "joint_finger_1",
    "joint_finger_2",
    "joint_finger_3"
]

# Define the goal positions (list of lists)
goal_positions = [
    [3.14, 3.14, 1.5, 2.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0],  # Goal 1
    [2.0, 2.5, 1.0, 1.5, 0.5, 0.0, 0.0, 0.5, 0.5, 0.5],    # Goal 2
    [1.5, 2.0, 1.0, 2.0, 1.0, 0.0, 0.0, 0.2, 0.2, 0.2],    # Goal 3
    [2.5, 3.0, 1.2, 1.8, 0.8, 0.0, 0.0, 0.8, 0.8, 0.8],    # Goal 4
    [3.0, 3.5, 1.0, 1.0, 0.2, 0.0, 0.0, 0.3, 0.3, 0.3]     # Goal 5
]

def move_joints():
    rospy.init_node('kinova_joint_controller', anonymous=True)
    
    # Create publishers for each joint controller
    pub_joint_positions = {}
    for joint in joint_names:
        pub_joint_positions[joint] = rospy.Publisher(f'/j2s7s300/{joint}_position_controller/command', Float64, queue_size=10)

    rospy.loginfo("Publishers created")

    # Wait for all publishers to connect
    rospy.sleep(1.0)

    # Publish commands for each goal position
    for goal_index, goal in enumerate(goal_positions):
        rospy.loginfo(f"Executing Goal {goal_index + 1}")

        # Publish commands to move each joint to the target position
        for i in range(len(joint_names)):
            pub_joint_positions[joint_names[i]].publish(Float64(goal[i]))

        # Wait for movement to complete (adjust duration based on your robot's speed)
        rospy.sleep(5.0)

    rospy.loginfo("Finished executing goals")

    # Stop joints (publish zero positions)
    for joint in joint_names:
        pub_joint_positions[joint].publish(Float64(0.0))

    rospy.loginfo("Stopping joints...")

if __name__ == '__main__':
    try:
        move_joints()
    except rospy.ROSInterruptException:
        pass

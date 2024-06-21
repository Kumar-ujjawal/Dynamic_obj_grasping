#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

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

def move_joints():
    rospy.init_node('kinova_joint_controller', anonymous=True)
    
    # Create publishers for each joint controller
    pub_joint_positions = {}
    for joint in joint_names:
        pub_joint_positions[joint] = rospy.Publisher(f'/j2s7s300/{joint}_position_controller/command', Float64, queue_size=10)

    rospy.loginfo("Publishers created")

    # Wait for all publishers to connect
    rospy.sleep(1.0)

    # Example: Move joint 1 to a specific position
    target_positions = [3.14, 3.14, 1.5, 2.0, 1.0, 0.0, 0.0, 1.0, 1.0, 1.0]  # Example target positions for joints and fingers

    # Publish commands to move each joint to the target position
    for i in range(len(joint_names)):
        pub_joint_positions[joint_names[i]].publish(Float64(target_positions[i]))

    rospy.loginfo("Moving joints...")

    # Wait for movement to complete
    rospy.sleep(5.0)

    # Stop joints (publish zero positions)
    for joint in joint_names:
        pub_joint_positions[joint].publish(Float64(0.0))

    rospy.loginfo("Stopping joints...")

if __name__ == '__main__':
    try:
        move_joints()
    except rospy.ROSInterruptException:
        pass

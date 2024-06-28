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

# Define the velocity commands (rad/s)
velocity_commands = [
    [3.5, -15.5, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 1
    [1.0, -13.5, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 2
    [0.0, 10.0, 3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 3
    [0.0, 10.0, 0.0, 3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Joint velocities for Goal 4
    [0.0, 10.0, 0.0, 0.0, 3.5, 0.0, 0.0, 0.0, 0.0, 0.0]   # Joint velocities for Goal 5
]

def move_joints():
    rospy.init_node('kinova_velocity_controller', anonymous=True)
    
    # Create publishers for each joint velocity controller
    pub_joint_velocities = {}
    for joint in joint_names:
        pub_joint_velocities[joint] = rospy.Publisher(f'/j2s7s300/{joint}_velocity_controller/command', Float64, queue_size=10)

    rospy.loginfo("Publishers created")

    # Wait for all publishers to connect
    rospy.sleep(1.0)

    # Publish velocity commands for each set of joint velocities
    for goal_index, velocities in enumerate(velocity_commands):
        rospy.loginfo(f"Executing Goal {goal_index + 1}")

        # Publish velocity commands for each joint
        for i in range(len(joint_names)):
            pub_joint_velocities[joint_names[i]].publish(Float64(velocities[i]))

        # Wait for the arm to move (adjust duration based on your robot's speed)
        rospy.sleep(5.0)

    rospy.loginfo("Finished executing goals")

    # Stop joints (publish zero velocities)
    for joint in joint_names:
        pub_joint_velocities[joint].publish(Float64(0.0))

    rospy.loginfo("Stopping joints...")

if __name__ == '__main__':
    try:
        move_joints()
    except rospy.ROSInterruptException:
        pass

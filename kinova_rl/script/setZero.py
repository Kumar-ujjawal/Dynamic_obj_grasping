#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def set_initial_velocities():
    rospy.init_node('set_initial_velocities', anonymous=True)
    
    robot_name = rospy.get_param('~robot_name', 'j2s7s300')
    robot_type = rospy.get_param('~robot_type', 'j2s7s300')
    
    pub = rospy.Publisher(f'/{robot_name}/set_joint_velocities', JointTrajectory, queue_size=10)
    
    rospy.sleep(5)  # Wait for 5 seconds to ensure everything is initialized
    
    trajectory = JointTrajectory()
    trajectory.joint_names = [f'{robot_type}joint{i}' for i in range(1, 8)]
    
    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 7
    trajectory.points.append(point)
    
    pub.publish(trajectory)
    rospy.loginfo("Published initial joint velocities")

if __name__ == '__main__':
    try:
        set_initial_velocities()
    except rospy.ROSInterruptException:
        pass
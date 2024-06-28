#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

rospy.init_node('odom_node')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_link_state')
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

last_time = rospy.Time(0)
def publish_link_velocity(link_name, reference_frame):
    global last_time
    try:
        link = GetLinkStateRequest()
        link.link_name = link_name
        link.reference_frame = reference_frame
        
        result = get_link_srv(link)
        
        current_time = rospy.Time.now()
        if current_time < last_time:
            rospy.logwarn("Time moved backwards. Resetting last_time.")
            last_time = current_time
        
        odom = Odometry()
        header = Header()
        header.frame_id = reference_frame
        
        # Set only the twist (velocity) information
        odom.twist.twist = result.link_state.twist
        header.stamp = current_time
        odom.header = header
        
        odom_pub.publish(odom)
        last_time = current_time
    
    except rospy.ROSException as e:
        rospy.logerr(f"ROS Exception: {e}")

link_name = 'j2s7s300_link_7'  # Assuming link 7 is the end effector
reference_frame = 'world'

rate = rospy.Rate(10)  # 10 Hz, but adjust as needed
while not rospy.is_shutdown():
    try:
        publish_link_velocity(link_name, reference_frame)
        rate.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        rospy.logwarn("ROS time moved backwards. Continuing...")
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Exiting...")
        break

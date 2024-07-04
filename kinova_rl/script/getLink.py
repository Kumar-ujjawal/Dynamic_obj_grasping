#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

rospy.init_node('odom_node')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_link_state')
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

last_time = rospy.Time(0)
def publish_link_state(link_name, reference_frame):
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
        
        # Publish Odometry message
        odom = Odometry()
        header = Header()
        header.frame_id = reference_frame
        header.stamp = current_time
        odom.header = header
        
        # Set pose (position) information
        pose = Pose()
        pose.position = result.link_state.pose.position
        pose.orientation = result.link_state.pose.orientation
        odom.pose.pose = pose
        
        # Set twist (velocity) information
        twist = Twist()
        twist.linear = result.link_state.twist.linear
        twist.angular = result.link_state.twist.angular
        odom.twist.twist = twist
        
        odom_pub.publish(odom)
        last_time = current_time
    
    except rospy.ROSException as e:
        rospy.logerr(f"ROS Exception: {e}")

link_name = 'box'  # Adjust to your specific link name
reference_frame = 'world'

rate = rospy.Rate(10)  # 10 Hz, adjust as necessary
while not rospy.is_shutdown():
    try:
        publish_link_state(link_name, reference_frame)
        rate.sleep()
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        rospy.logwarn("ROS time moved backwards. Continuing...")
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("ROS interrupt received. Exiting...")
        break

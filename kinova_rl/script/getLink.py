#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetLinkState, GetLinkStateRequest

rospy.init_node('odom_node')
odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.wait_for_service('/gazebo/get_link_state')
get_link_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

rate = rospy.Rate(10)  # 10Hz

def publish_link_odom(link_name, reference_frame):
    try:
        link = GetLinkStateRequest()
        link.link_name = link_name
        link.reference_frame = reference_frame
        
        result = get_link_srv(link)
        
        odom = Odometry()
        header = Header()
        header.frame_id = reference_frame
        
        odom.pose.pose = result.link_state.pose
        odom.twist.twist = result.link_state.twist
        header.stamp = rospy.Time.now()
        odom.header = header
        
        odom_pub.publish(odom)
    
    except rospy.ROSTimeMovedBackwardsException:
        pass

# List of link names to publish odometry for (adjust these according to your robot's URDF)
link_names = [
    
    'j2s7s300_link_1',
    # 'j2s7s300_link_2',
    # 'j2s7s300_link_3',
    # 'j2s7s300_link_4',
    # 'j2s7s300_link_5',
    # 'j2s7s300_link_6',
    # 'j2s7s300_link_7'
]

reference_frame = 'world'  # Reference frame for odometry

while not rospy.is_shutdown():
    for link_name in link_names:
        publish_link_odom(link_name, reference_frame)
    
    rate.sleep()


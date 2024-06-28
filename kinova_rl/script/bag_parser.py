import rosbag
import csv
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

# Function to extract joint velocities and end-effector velocities
def extract_velocities(bag_file):
    joint_velocities = []
    end_effector_velocity = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            print(topic,"and the msg is:",msg)

    



if __name__ == "__main__":
    # Replace 'your_bag_file.bag' with the path to your ROS bag file
    bag_file = r'/home/vr-lab/kinova_arm/src/kinova_rl/script/2024-06-26-16-03-02.bag'

    # Extract velocities
    joint_velocities, end_effector_velocity = extract_velocities(bag_file)

    

    print("CSV files saved successfully.")

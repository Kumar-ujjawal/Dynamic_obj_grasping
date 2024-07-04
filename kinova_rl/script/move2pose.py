import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from k2 import Robot7DOF  

class KinovaRobotController:
    def __init__(self):
        rospy.init_node('kinova_robot_controller')
        
        # Initialize Robot7DOF kinematics calculator
        self.kinematics = Robot7DOF()
        
        # Initialize ROS publisher for joint trajectory commands
        self.joint_trajectory_pub = rospy.Publisher('/j2s7s300/joint_trajectory_controller/command', JointTrajectory, queue_size=10)
        
        # Subscribe to joint states
        self.joint_state_sub = rospy.Subscriber('/j2s7s300/joint_states', JointState, self.joint_states_callback)
        self.joint_state = JointState()
        
        # Wait for connections to publishers and subscribers
        rospy.sleep(1.0)
        
    def joint_states_callback(self, msg):
        self.joint_state = msg
    
    def move_to_pose(self, target_position):
        # Perform inverse kinematics to find joint angles
        initial_guess = self.joint_state.position[:7]  # You can provide an initial guess if available
        joint_angles = self.kinematics.inverse_kinematics_multi_attempt(target_position, initial_guess)
        
        if joint_angles is None:
            rospy.logerr("Failed to find valid joint angles for the target position")
            return -4
        
        # Create JointTrajectory message
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = [
            'j2s7s300_joint_1',
            'j2s7s300_joint_2',
            'j2s7s300_joint_3',
            'j2s7s300_joint_4',
            'j2s7s300_joint_5',
            'j2s7s300_joint_6',
            'j2s7s300_joint_7'
        ]
        
        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(5.0)  # Adjust timing based on your robot's dynamics
        
        # Add the trajectory point to the trajectory
        joint_trajectory.points.append(point)
        
        # Publish the joint trajectory
        self.joint_trajectory_pub.publish(joint_trajectory)
        
        # Wait for the movement to complete (adjust timing based on your robot's dynamics)
        rospy.sleep(5.0)
        
        # Check if the robot reached the desired position within tolerance
        if not self.kinematics.is_valid_solution(joint_angles, target_position):
            rospy.logerr("Failed to reach the target position")
            return -4
        
        return 0  # Success
    
    def shutdown(self):
        # Clean shutdown: Stop all publishers and unsubscribe from subscribers
        self.joint_trajectory_pub.unregister()
        self.joint_state_sub.unregister()
        rospy.loginfo("KinovaRobotController has been shut down")

if __name__ == '__main__':
    
    
    # Initialize the KinovaRobotController
    controller = KinovaRobotController()
    
    # Define the target pose (x, y, z) in meters
    target_position = [0.5, 0.3, 0.4]  # Example target pose
    
    # Move the robot to the target pose
    result = controller.move_to_pose(target_position)
    
    if result == 0:
        rospy.loginfo("Robot moved to the target pose successfully")
    else:
        rospy.logerr("Failed to move the robot to the target pose")
    
    # Shutdown the controller
    controller.shutdown()
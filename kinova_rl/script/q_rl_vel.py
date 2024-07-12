
import rospy
import random
import numpy as np
from collections import defaultdict
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import PyKDL as kdl
from threading import Thread, Lock


# Assuming Robot7DOF is defined elsewhere
from kinematics import Robot7DOF  # Assuming this is your custom robot kinematics class
from tf.transformations import euler_from_quaternion

class KinovaEnvRL:
    def __init__(self):
        rospy.loginfo("Initializing KinovaEnvRL")
        self.robot = Robot7DOF()  # Initialize your robot class (Robot7DOF)
        self.link_pose = None
        self.joint_positions = [0.0] * 7
        self.joint_velocities = [0.0] * 7
        self.joint_torques = [0.0] * 7
        self.subscriber = rospy.Subscriber('/j2s7s300/joint_states', JointState, self.joint_states_callback)
        
        # Joint velocity publishers
        self.joint_vel_pubs = []
        for i in range(7):
            pub = rospy.Publisher(f'/j2s7s300/joint_{i+1}_velocity_controller/command', Float64, queue_size=1)
            self.joint_vel_pubs.append(pub)
        
        self.pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.unpause_physics = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.action_space = 7  # One action per joint
        self.state_space = 24  # 7 joint positions, 7 joint velocities, 7 joint torques, 3 goal position
        self.action_step = 0.1  # The step size for each action

        self.rate = rospy.Rate(10)
        # URDF SETTING 
        self.robot_urdf = URDF.from_parameter_server('/robot_description')
        self.kdl_tree = kdl_tree_from_urdf_model(self.robot_urdf)
        self.base_link = self.robot_urdf.get_root()
        self.end_effector_link = 'j2s7s300_end_effector'  # Adjust this to your robot's end effector link name

        # Create KDL chain
        self.arm_chain = self.kdl_tree.getChain(self.base_link, self.end_effector_link)

        # Get link names
        self.link_names = [self.arm_chain.getSegment(i).getName() for i in range(self.arm_chain.getNrOfSegments())]


        self.initial_joint_positions = [0.0, 0.0, 2.9, 1.3, -2.07, 1.4, 0.0]
        self.joint_lower_limits, self.joint_upper_limits, self.joint_velocity_limits = self.robot.get_joint_limits()
        self.obstacle_positions = [np.array([0.3, 0.3, 0.5]), np.array([-0.3, -0.3, 0.5])]
        self.curriculum_stage = 0
        self.max_curriculum_stages = 5
        self.episode_count = 0
        self.episodes_per_curriculum = 200
        self.last_distance = None
        self.steps_without_progress = 0
        self.distance_threshold = 0.001
        self.max_steps_without_progress = 10
        self.goal_position = None
        self.recent_rewards = []  # Store recent rewards for feedback
        self.goal_achieved_count = 0  # Counter for goal achievement
        rospy.loginfo("KinovaEnvRL initialized")
        
        # thread
        self.joint_states_lock = Lock()
        self.joint_states_thread = Thread(target=self.joint_states_listener)
        self.joint_states_thread.daemon = True
        self.joint_states_thread.start()

    def joint_states_callback(self, data):
        with self.joint_states_lock:
            self.joint_positions = data.position[:7]
            self.joint_velocities = data.velocity[:7]
            self.joint_torques = data.effort[:7]

    def joint_states_listener(self):
        rospy.loginfo("Joint state listener thread started")
        rospy.spin()

    def _get_state(self):
        with self.joint_states_lock:
            return self.joint_positions, self.joint_velocities, self.joint_torques


    def reset(self):
        max_reset_attempts = 5
        for attempt in range(max_reset_attempts):
            try:
                rospy.loginfo(f"Attempting to reset simulation (attempt {attempt + 1})")
                
                # Pause the simulation
                rospy.wait_for_service("/gazebo/pause_physics", timeout=5.0)
                self.pause_physics()
                rospy.loginfo("Simulation paused")

                # Reset the simulation
                rospy.wait_for_service("/gazebo/reset_simulation", timeout=5.0)
                self.reset_simulation()
                rospy.loginfo("Simulation reset")

                # Set the robot to its initial state
                model_state = ModelState()
                model_state.model_name = "j2s7s300"  
                model_state.pose.position.x = 0
                model_state.pose.position.y = 0
                model_state.pose.position.z = 0
                model_state.pose.orientation.x = 0
                model_state.pose.orientation.y = 0
                model_state.pose.orientation.z = 0
                model_state.pose.orientation.w = 1
                
                rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
                self.set_model_state(model_state)
                rospy.loginfo("Robot state reset")

                # Unpause the simulation
                rospy.wait_for_service("/gazebo/unpause_physics", timeout=5.0)
                self.unpause_physics()
                rospy.loginfo("Simulation unpaused")

                # Wait for joint states to be updated
                timeout = rospy.Duration(5.0)
                start_wait = rospy.Time.now()
                while len(self.joint_positions) < 7 and (rospy.Time.now() - start_wait) < timeout:
                    rospy.sleep(0.1)
                
                if len(self.joint_positions) < 7:
                    rospy.logerr("Joint states not updated after reset")
                    continue
                
                self.goal_position = np.random.uniform(low=-2, high=2, size=3)
                rospy.loginfo(f"New goal generated: {self.goal_position}")
                for i, pos in enumerate(self.initial_joint_positions):
                    self.set_joint_position(i, pos)
                self.last_distance = self._get_distance_to_goal()
                self.steps_without_progress = 0
                rospy.loginfo("Reset successful")
                return self._get_state()
            except (rospy.ServiceException, rospy.ROSException) as e:
                rospy.logerr(f"Reset attempt {attempt + 1} failed: {str(e)}")
            
            # Add a longer sleep between reset attempts
            rospy.sleep(2.0)
        
        rospy.logerr("Failed to reset simulation after multiple attempts")
        return None  # Return None instead of raising an exception

    def set_joint_position(self, joint_index, position):
        self.joint_positions[joint_index] = position
        rospy.logdebug(f"Set joint position[{joint_index}] to {position}")

    def step(self, action):
        if rospy.is_shutdown():
            rospy.logerr("ROS is shutting down. Aborting step.")
            return None, None, True, {}

        # Convert discrete actions to joint velocities
        joint_velocities = np.clip(action, -np.array(self.joint_velocity_limits), np.array(self.joint_velocity_limits))
        
        current_distance = self._get_distance_to_goal()
        
        if abs(current_distance - self.last_distance) < self.distance_threshold:
            self.steps_without_progress += 1
        else:
            self.steps_without_progress = 0

        if self.steps_without_progress >= self.max_steps_without_progress:
            joint_velocities = self._take_random_action()
            self.steps_without_progress = 0

        try:
            for i, pub in enumerate(self.joint_vel_pubs):
                pub.publish(Float64(joint_velocities[i]))
                rospy.loginfo(f"Joint {i+1} velocity: {joint_velocities[i]}")

            rospy.sleep(0.1)  # Allow time for action to take effect
        except rospy.ROSException as e:
            rospy.logerr(f"Error while publishing joint velocities: {str(e)}")
            return None, None, True, {}

        state = self._get_state()
        reward = self._compute_reward(current_distance, joint_velocities)
        done = self._is_done(current_distance)

        if done:
            self.goal_achieved_count += 1
            rospy.loginfo(f"Goal achieved! Total goals achieved so far: {self.goal_achieved_count}")

        self.last_distance = current_distance
        self.recent_rewards.append(reward)  # Store reward for feedback

        return state, reward, done, {}

    def _take_random_action(self):
        return np.random.uniform(-np.array(self.joint_velocity_limits), np.array(self.joint_velocity_limits), size=self.action_space)

    # def _get_state(self):
    #     state = np.concatenate([self.joint_positions, self.joint_velocities, self.joint_torques, self.goal_position])
    #     rospy.loginfo(f"Current state: {state}")
    #     return state

    def _get_distance_to_goal(self):
        end_effector_pos = self.robot.forward_kinematics(self.joint_positions)[:3, 3]
        distance = np.linalg.norm(end_effector_pos - self.goal_position)
        rospy.loginfo(f"Distance to goal: {distance}")
        return distance

    def _compute_reward(self, current_distance, action):
        distance_reward = -current_distance
        goal_reward = 100 if current_distance < 0.1 else 0

        progress_reward = self.last_distance - current_distance

        # Calculate joint limit penalty
        joint_limit_penalty = sum(
            abs(pos - limit[0]) + abs(pos - limit[1])
            for pos, limit in zip(self.joint_positions, zip(self.joint_lower_limits, self.joint_upper_limits))
        )
        rospy.logdebug(f"Joint limit penalty: {joint_limit_penalty}")

        # Calculate velocity limit penalty
        velocity_limit_penalty = sum(
            abs(vel) / limit if limit != 0 else 0  # Handle division by zero gracefully
            for vel, limit in zip(action, self.joint_velocity_limits)
        )
        rospy.logdebug(f"Velocity limit penalty: {velocity_limit_penalty}")

        # Calculate smoothness reward
        smoothness_reward = -np.sum(np.square(self.joint_velocities))
        rospy.logdebug(f"Smoothness reward: {smoothness_reward}")

        # Calculate self-collision penalty
        collision_penalty = self._check_self_collision()
        rospy.logdebug(f"Collision penalty: {collision_penalty}")

        # Calculate total reward
        total_reward = (
            5 * distance_reward +
            2 * goal_reward +
            50 * progress_reward
            - 0.5 * joint_limit_penalty +
            - 0.3 * velocity_limit_penalty +
            0.1 * smoothness_reward +
            - 10 * collision_penalty  # Heavy penalty for self-collision
        )
        rospy.logdebug(f"Total reward: {total_reward}")

        return total_reward

    def _check_self_collision(self):
        collision_threshold = 0.05  # Adjust this value based on your robot's size and desired sensitivity
        collision_penalty = 0

        # Create a KDL joint array from current joint positions
        kdl_joint_array = kdl.JntArray(len(self.joint_positions))
        for i, pos in enumerate(self.joint_positions):
            kdl_joint_array[i] = pos

        # Forward kinematics solver
        fk_solver = kdl.ChainFkSolverPos_recursive(self.arm_chain)

        # Check distances between all pairs of links
        for i in range(len(self.link_names)):
            for j in range(i + 2, len(self.link_names)):  # Start from i+2 to avoid checking adjacent links
                frame_i = kdl.Frame()
                frame_j = kdl.Frame()

                fk_solver.JntToCart(kdl_joint_array, frame_i, i)
                fk_solver.JntToCart(kdl_joint_array, frame_j, j)

                distance = kdl.diff(frame_i.p, frame_j.p).Norm()

                if distance < collision_threshold:
                    collision_penalty += 1
                    rospy.logdebug(f"Potential collision detected between {self.link_names[i]} and {self.link_names[j]}")

        return collision_penalty

    def _is_done(self, current_distance):
        done = current_distance < 0.5 or self.steps_without_progress >= self.max_steps_without_progress
        rospy.logdebug(f"Done: {done}")
        return done

    def get_reward_feedback(self):
        if len(self.recent_rewards) > 0:
            average_reward = np.mean(self.recent_rewards)
            if average_reward < -5.0:
                feedback = 0.1
            elif average_reward > 0.0:
                feedback = 2.0
            else:
                feedback = 1.0
        else:
            feedback = 1.0

        rospy.logdebug(f"Reward feedback: {feedback}")
        return feedback

    def update_curriculum(self):
        self.episode_count += 1
        if self.episode_count % self.episodes_per_curriculum == 0:
            self.curriculum_stage = min(self.curriculum_stage + 1, self.max_curriculum_stages - 1)
        rospy.loginfo(f"Updated curriculum: episode_count={self.episode_count}, curriculum_stage={self.curriculum_stage}")

    def close(self):
        rospy.loginfo("Closing KinovaEnvRL")
        # Add any cleanup code here if necessary
class QLearningAgent:
    def __init__(self, action_space, state_space, learning_rate=0.1, discount_factor=0.95, epsilon=0.1):
        rospy.loginfo("Initializing QLearningAgent")
        self.action_space = action_space
        self.state_space = state_space
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.epsilon = epsilon
        self.q_table = defaultdict(lambda: np.zeros(action_space))
        rospy.loginfo("QLearningAgent initialized")

    def get_action(self, state, feedback_weight=1.0):
        if np.random.random() < self.epsilon * feedback_weight:
            action = self._take_random_action()
            rospy.loginfo(f"Random action taken: {action}")
            return action
        else:
            discretized_state = self._discretize_state(state)
            action = np.argmax(self.q_table[discretized_state])
            rospy.loginfo(f"Action chosen: {action}")
            return action

    def _take_random_action(self):
        return np.random.uniform(-2.0, 2.0, size=self.action_space)

    def update(self, state, action, reward, next_state, done):
        discretized_state = self._discretize_state(state)
        discretized_next_state = self._discretize_state(next_state)

        action_index = np.argmin(np.abs(np.array(self.q_table[discretized_state]) - action))
        
        current_q = self.q_table[discretized_state][action_index]
        if done:
            future_q = reward
        else:
            future_q = reward + self.discount_factor * np.max(self.q_table[discretized_next_state])

        self.q_table[discretized_state][action_index] += self.learning_rate * (future_q - current_q)
        rospy.loginfo(f"Q-value updated for state {discretized_state}, action {action_index}: {self.q_table[discretized_state][action_index]}")

    def _discretize_state(self, state):
        discretized_state = tuple(np.round(state).astype(int))
        return discretized_state

    def decay_epsilon(self, decay_rate=0.995):
        self.epsilon *= decay_rate
        rospy.loginfo(f"Epsilon decayed: {self.epsilon}")

def train():
    rospy.loginfo("Starting training")
    env = KinovaEnvRL()
    agent = QLearningAgent(env.action_space, env.state_space)
    episodes =  100
    max_step = 500

    try:
        for episode in range(episodes):
            rospy.loginfo(f"Starting episode {episode + 1}")
            
            if rospy.is_shutdown():
                rospy.logerr("ROS master is not running. Exiting.")
                break

            rospy.loginfo("Resetting environment...")
            state = env.reset()
            if state is None:
                rospy.logerr("Failed to reset environment. Skipping this episode.")
                continue

            rospy.loginfo("Environment reset complete. Starting new episode...")
            done = False
            total_reward = 0
            step = 0

            try:
                while not done and step < max_step:
                    feedback_weight = env.get_reward_feedback()
                    action = agent.get_action(state, feedback_weight)
                    next_state, reward, done, _ = env.step(action)
                    
                    if next_state is None:
                        rospy.logerr("Received None state from environment. Ending episode.")
                        break

                    agent.update(state, action, reward, next_state, done)
                    state = next_state
                    total_reward += reward
                    step += 1

                    if step % 10 == 0:
                        rospy.loginfo(f"Episode {episode + 1}, Step {step}, Reward: {reward}")

                    if done:
                        rospy.loginfo(f"Goal achieved at step {step}")
                        break

                env.update_curriculum()
                agent.decay_epsilon()
                rospy.loginfo(f"Episode {episode + 1} completed, Total Reward: {total_reward}")
                rospy.sleep(1)  # Allow some time between episodes

            except Exception as e:
                rospy.logerr(f"Error in episode {episode + 1}: {str(e)}")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"An error occurred in training: {str(e)}")
    finally:
        env.close()

    # Print out the total number of goal achievements
    rospy.loginfo(f"Total number of goal achievements: {env.goal_achieved_count}")

if __name__ == '__main__':
    try:
        rospy.init_node('kinova_rl_node', anonymous=True)
        rospy.loginfo("Node initialized")
        train()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")

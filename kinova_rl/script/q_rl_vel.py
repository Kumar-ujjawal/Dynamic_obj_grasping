#!/usr/bin/env python3

import rospy
import random
import numpy as np
from collections import defaultdict
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from kinematics import Robot7DOF  # Assuming Robot7DOF is defined elsewhere
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
        
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

        self.action_space = 7  # One action per joint
        self.state_space = 24  # 7 joint positions, 7 joint velocities, 7 joint torques, 3 goal position
        self.action_step = 0.1  # The step size for each action

        self.rate = rospy.Rate(10)

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
        rospy.loginfo("KinovaEnvRL initialized")

    def joint_states_callback(self, data):
        self.joint_positions = data.position[:7]
        self.joint_velocities = data.velocity[:7]
        self.joint_torques = data.effort[:7]

    def reset(self):
        rospy.wait_for_service("/gazebo/reset_simulation")
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            rospy.logerr("/gazebo/reset_simulation service call failed")

        rospy.sleep(1)  # Allow time to reset
        self.goal_position = np.random.uniform(low=-0.5, high=0.5, size=3)
        for i, pos in enumerate(self.initial_joint_positions):
            self.set_joint_position(i, pos)

        self.last_distance = self._get_distance_to_goal()
        self.steps_without_progress = 0
        return self._get_state()

    def set_joint_position(self, joint_index, position):
        self.joint_positions[joint_index] = position

    def step(self, action):
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

        for i, pub in enumerate(self.joint_vel_pubs):
            pub.publish(Float64(joint_velocities[i]))
            rospy.loginfo(f"Joint {i+1} velocity: {joint_velocities[i]}")

        rospy.sleep(0.1)  # Allow time for action to take effect

        state = self._get_state()
        reward = self._compute_reward(current_distance, joint_velocities)
        done = self._is_done(current_distance)

        self.last_distance = current_distance
        self.recent_rewards.append(reward)  # Store reward for feedback

        return state, reward, done, {}

    def _take_random_action(self):
        return np.random.uniform(-np.array(self.joint_velocity_limits), np.array(self.joint_velocity_limits), size=self.action_space)

    def _get_state(self):
        return np.concatenate([self.joint_positions, self.joint_velocities, self.joint_torques, self.goal_position])

    def _get_distance_to_goal(self):
        end_effector_pos = self.robot.forward_kinematics(self.joint_positions)[:3, 3]
        return np.linalg.norm(end_effector_pos - self.goal_position)

    def _compute_reward(self, current_distance, action):
        distance_reward = -current_distance
        goal_reward = 100 if current_distance < 0.1 else 0
        joint_limit_penalty = sum(
            abs(pos - limit[0]) + abs(pos - limit[1])
            for pos, limit in zip(self.joint_positions, zip(self.joint_lower_limits, self.joint_upper_limits))
        )
        velocity_limit_penalty = sum(
            abs(vel) / limit for vel, limit in zip(action, self.joint_velocity_limits)
        )
        smoothness_reward = -np.sum(np.square(self.joint_velocities))

        total_reward = (
            5 * distance_reward +
            goal_reward +
            -0.1 * joint_limit_penalty +
            -0.3 * velocity_limit_penalty +
            0.1 * smoothness_reward
        )

        return total_reward

    def _is_done(self, current_distance):
        return current_distance < 0.1 or self.steps_without_progress >= self.max_steps_without_progress

    def get_reward_feedback(self):
        if len(self.recent_rewards) > 0:
            average_reward = np.mean(self.recent_rewards)
            if average_reward < -5.0:
                return 0.1  # Increase exploration (lower epsilon)
            elif average_reward > 0.0:
                return 2.0  # Exploit more (higher epsilon)
            else:
                return 1.0  # Default case
        else:
            return 1.0  # Default case if no rewards yet

    def update_curriculum(self):
        self.episode_count += 1
        if self.episode_count % self.episodes_per_curriculum == 0:
            self.curriculum_stage = min(self.curriculum_stage + 1, self.max_curriculum_stages - 1)

    def close(self):
        pass

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
            return self._take_random_action()
        else:
            discretized_state = self._discretize_state(state)
            return np.argmax(self.q_table[discretized_state])

    def _take_random_action(self):
        return np.random.uniform(-1.0, 1.0, size=self.action_space)

    def update(self, state, action, reward, next_state, done):
        discretized_state = self._discretize_state(state)
        discretized_next_state = self._discretize_state(next_state)
    
        current_q = self.q_table[discretized_state][action]
        if done:
            future_q = reward
        else:
            future_q = reward + self.discount_factor * np.max(self.q_table[discretized_next_state])
    
        self.q_table[discretized_state][action] += self.learning_rate * (future_q - current_q)

    def _discretize_state(self, state):
        return tuple(np.round(state, decimals=1))

    def decay_epsilon(self, decay_rate=0.995):
        self.epsilon *= decay_rate

def train():
    rospy.loginfo("Starting training")
    env = KinovaEnvRL()
    agent = QLearningAgent(env.action_space, env.state_space)
    episodes = 10
    max_step = 1000
    for episode in range(episodes):
        rospy.loginfo(f"Starting episode {episode + 1}")
        state = env.reset()
        done = False
        total_reward = 0
        step = 0

        while not done and step < max_step:
            feedback_weight = env.get_reward_feedback()  # Obtain feedback weight
            action = agent.get_action(state, feedback_weight)  # Pass feedback weight to agent
            next_state, reward, done, _ = env.step(action)
            agent.update(state, action, reward, next_state, done)
            state = next_state
            total_reward += reward
            step += 1

            if step % 10 == 0:  # Log every 10 steps to reduce console spam
                rospy.loginfo(f"Episode {episode + 1}, Step {step}, Reward: {reward}")

            if done:
                rospy.loginfo(f"Goal achieved at step {step}")
                break

        env.update_curriculum()
        agent.decay_epsilon()
        rospy.loginfo(f"Episode {episode + 1} completed, Total Reward: {total_reward}")
        rospy.sleep(1)  # Allow some time between episodes

    env.close()

if __name__ == '__main__':
    try:
        rospy.init_node('kinova_rl_node', anonymous=True)
        rospy.loginfo("Node initialized")
        train()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"An error occurred: {str(e)}")

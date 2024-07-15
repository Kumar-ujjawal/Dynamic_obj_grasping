#!/home/vr-lab/anaconda3/envs/bin/python
import rospy
import gym
import threading
from collections import deque
import numpy as np
import random
import matplotlib.pyplot as plt
from gym import spaces
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time
from std_srvs.srv import Empty
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions.normal import Normal
from numpy import inf
import subprocess
from os import path
import os
from stable_baselines import PPO2
from stable_baselines.common.policies import MlpPolicy
from  stable_baselines.common.vec_env import DummyVecEnv
from tqdm import tqdm
from torch.distributions import MultivariateNormal, Normal
from collections import namedtuple,deque
import matplotlib
torch.autograd.set_detect_anomaly(True)
from torch.utils.tensorboard import SummaryWriter
import datetime 
from kinematics import Robot7DOF

class Jaco2Env(gym.Env):
    def __init__(self):
        

        super(Jaco2Env, self).__init__()
        self.action_dim = 7
        self.obs_dim = 18   #(joint_position:7 , joint_velocity:7 , end_effector_position:3, time:1)

        
        self.action_space = spaces.Box(low=-0.6, high=0.6, shape=(7,), dtype=np.float64)

        # Observation space, assuming joint angles and velocities and end effefctor coordinates as states
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(18,), dtype=np.float64)


        port = "11311"
        self.launfile = "velocity_control.launch"
        subprocess.Popen(["roscore","-p",port])
        print("roscore launched!")
        rospy.init_node('dqn_controller')
        subprocess.Popen(["roslaunch","-p", "11311","kinova_rl","velocity_control.launch"])
        print("Gazebo Launched")
        self.states = np.zeros(14)
        self.joint_state_sub = rospy.Subscriber('j2s7s300/joint_states', JointState, self.joint_state_callback)
        self.joint_vel_pub1 = rospy.Publisher('/j2s7s300/joint_1_velocity_controller/command', Float64, queue_size=1)
        self.joint_vel_pub2 = rospy.Publisher('/j2s7s300/joint_2_velocity_controller/command', Float64, queue_size=1)
        self.joint_vel_pub3 = rospy.Publisher('/j2s7s300/joint_3_velocity_controller/command', Float64, queue_size=1)     
        self.joint_vel_pub4 = rospy.Publisher('/j2s7s300/joint_4_velocity_controller/command', Float64, queue_size=1)     
        self.joint_vel_pub5 = rospy.Publisher('/j2s7s300/joint_5_velocity_controller/command', Float64, queue_size=1)     
        self.joint_vel_pub6 = rospy.Publisher('/j2s7s300/joint_6_velocity_controller/command', Float64, queue_size=1)     
        self.joint_vel_pub7 = rospy.Publisher('/j2s7s300/joint_7_velocity_controller/command', Float64, queue_size=1)             
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics",Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics",Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_simulation",Empty)
        # rospy.spin()
        print("all service and topics called")
        self.d_parameters = [0.2755,0.2050,0.2050,0.2073,0.1038,0.1038,0.1600,0.0098] #d1,d2,d3,d4,d5,d6,d7,e2  
        self.states = np.zeros(14)
        self.episode_time = 0
        self.current_step = 0
        self.max_step = 1000
        self.robot = Robot7DOF()
        self.joint_lower_limits, self.joint_upper_limits, self.joint_velocity_limits = self.robot.get_joint_limits()
        self.states_lock = threading.Lock()
        self.callback_thread = threading.Thread(target = self.run_callback)
        self.callback_thread.daemon = True
        self.callback_thread.start()
        # TIME_DELTA=0

    def run_callback(self):
        rospy.spin()

    def joint_state_callback(self,msg):
        self.angles = msg.position[:7]
        self.velocities = msg.velocity[:7]
        #self.states = angles + velocities
        with self.states_lock :
            self.states = self.angles + self.velocities
        print("callback details:",self.states)

    def get_current_state(self):
        with self.states_lock:
            return self.states

    def update_goal_position(self):
        self.goal_position += np.random.uniform(low=-0.05, high=0.05, size=3)  # Random continuous motion
        

    def calculate_reward(self,distance, current_position, action, previous_action=None,previous_distance=None,episode_step=0,max_steps =1000):
        # Distance to goal
        distance_reward = -np.exp(0.2*distance)

        # Progress towards goal
        
        progress = previous_distance - distance
        progress_rate = progress/0.002
        progress_reward = np.clip(progress_rate, -1, 1) * 10
        # if progress > 0:
        #     progress_reward = 3 * progress  # Reward for moving towards the goal
        # else:
        #     progress_reward = -10

        # Action smoothness
        if previous_action is not None:
            action_smoothness = -0.1*np.sum(np.square(action - previous_action))
        else:
            action_smoothness = 0

        # Encourage exploration in early stages 
        exploration_factor = max(0, 1 - episode_step / max_steps)  # Decreases from 1 to 0 over the episode
        exploration_reward = 2 * exploration_factor

        # Penalize being close to joint limits
        
        # joint_limit_penalty = sum(
        #     abs(pos - limit[0]) + abs(pos - limit[1])
        #     for pos, limit in zip(self.angles, zip(self.joint_lower_limits, self.joint_upper_limits))
        # )
        # rospy.logdebug(f"Joint limit penalty: {joint_limit_penalty}")

        # # Calculate velocity limit penalty
        # velocity_limit_penalty = sum(
        #     abs(vel) / limit if limit != 0 else 0  # Handle division by zero gracefully
        #     for vel, limit in zip(action, self.joint_velocity_limits)
        # )
        # rospy.logdebug(f"Velocity limit penalty: {velocity_limit_penalty}")
        # Energy efficiency
        energy_penalty = -0.01 * np.sum(np.square(action))
        # time_penality = -0.1* self.episode_time

        # Combine rewards
        reward = (
            2 * distance_reward +
           5 * progress_reward +
            0.1* action_smoothness +
            exploration_reward  +
            0.2 * energy_penalty +
             exploration_reward #+
            #  2* time_penality
            #  - 0.5 * joint_limit_penalty +
            # - 0.3 * velocity_limit_penalty 
        )

        # Bonus for reaching the goal
        if distance < 1.25:
            reward +=50

        if distance <0.85:
            reward +=100

        if distance < 0.5:
            reward += 200
        if distance < 0.1:
            reward +=1000
        return reward
    
    def compute_position(self,state):
        # Ensure the current joint states are available
        if state is None:
            return np.zeros(len(7))
        joint_angles = np.array(state)
        T__0 = self.robot.forward_kinematics(joint_angles)
        current_position = T__0[:3,3]
        return current_position
    
    
    def step(self, action): #perform an action and read a new state
        joint_vel_msg_1 = Float64()
        joint_vel_msg_2 = Float64()
        joint_vel_msg_3 = Float64()
        joint_vel_msg_4 = Float64()
        joint_vel_msg_5 = Float64()
        joint_vel_msg_6 = Float64()
        joint_vel_msg_7 = Float64()
        joint_vel_msg_1.data = action[0]
        joint_vel_msg_2.data = action[1]        

        
        joint_vel_msg_3.data = action[2] 
        joint_vel_msg_4.data = action[3] 
        joint_vel_msg_5.data = action[4] 
        joint_vel_msg_6.data = action[5] 
        joint_vel_msg_7.data = action[6]  
        self.joint_vel_pub1.publish(joint_vel_msg_1)
        self.joint_vel_pub2.publish(joint_vel_msg_2)
        self.joint_vel_pub3.publish(joint_vel_msg_3)
        self.joint_vel_pub4.publish(joint_vel_msg_4)
        self.joint_vel_pub5.publish(joint_vel_msg_5)
        self.joint_vel_pub6.publish(joint_vel_msg_6)
        self.joint_vel_pub7.publish(joint_vel_msg_7)
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")            
        # time.sleep(TIME_DELTA)
        # TIME_DELTA =0
        time.sleep(0.002)
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        self.update_goal_position() 
        #print("current state taken from topic:", self.states) # Update the goal position continuously
        current_state = self.get_current_state()
        end_effector_position = self.compute_position(current_state[:7])
        self.episode_time +=0.002
        next_state = np.concatenate((current_state, end_effector_position,[self.episode_time]))
        distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)     
        reward = self.calculate_reward(distance_to_goal,current_state,action,self.last_action,self.last_distance,self.current_step, self.max_step)
        done = distance_to_goal < 0.5  # Close enough to goal
         # Large reward for reaching the goal
        self.current_step += 1
        print("reward : ",reward)
        print("next state: ",next_state)
        print("distance to goal:",distance_to_goal)
        # last_action = action
        # last_distance = distance_to_goal
        return next_state, reward, done, {}

    def reset(self):
        rospy.wait_for_service("/gazebo/reset_simulation")
        try :
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")
        rospy.sleep(1)  # Allow time to reset
        #self.update_goal_position()  # Publish initial goal position
        # Initialize goal position at a random position within the workspace
        self.goal_position = np.random.uniform(low=-3, high=3, size=3)
        self.goal_position[2] = abs(self.goal_position[2])

        print("goal position :" , self.goal_position)
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")   
        # time.sleep(TIME_DELTA)
        time.sleep(0.002)
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        current_state = self.get_current_state()
        self.current_step = 0
        self.last_action = 0
        self.last_distance = 0
        #print("current state taken from topic:", current_state)
        end_effector_position = self.compute_position(current_state[:7])
        return np.concatenate((current_state, end_effector_position,[self.episode_time]))
    


class ActorNetwork(nn.Module):
    def __init__(self,n_actions, state_dim,fc1_dims = 256, fc2_dims = 256, chkpt_dir = 'tmp/ppo'):
        super(ActorNetwork,self).__init__()
        self.fc1 = nn.Linear(state_dim, fc1_dims).to(torch.float32)
        self.bn1 = nn.BatchNorm1d(fc1_dims)

        self.fc2 = nn.Linear(fc1_dims, fc2_dims).to(torch.float32)
        self.bn2 = nn.BatchNorm1d(fc2_dims)
        self.mean = nn.Linear(fc2_dims, n_actions).to(torch.float32)
        self.log_std = nn.Parameter(torch.zeros(n_actions, dtype=torch.float32))
        path = 'best_model_episode_400/actor.pth'
        # pretrained_model = torch.load(path)
        # self.load_state_dict(pretrained_model['fc1'])
        # self.load_state_dict(pretrained_model['fc2'])
        self.apply(self._init_weights)
        self.disable_bn()
        
    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            module.bias.data.zero_()


    def forward(self,state):
        x = torch.relu(self.bn1(self.fc1(state)))
        x = torch.relu(self.bn2(self.fc2(x)))
        mean = self.mean(x)
        std = self.log_std.exp() #.expand_as(mean)
        return mean, std
    
    def disable_bn(self):
        self.bn1.eval()
        self.bn2.eval()    



class CriticNetwork(nn.Module):
    def __init__(self, state_dim,fc1_dims = 256, fc2_dims = 256, chkpt_dir = 'tmp/ppo'):
        super(CriticNetwork,self).__init__()
        self.fc1 = nn.Linear(state_dim, fc1_dims).to(torch.float32)
        self.fc2 = nn.Linear(fc1_dims, fc2_dims).to(torch.float32)
        self.value = nn.Linear(fc2_dims, 1).to(torch.float32)
        path = 'best_model_episode_400/critic.pth'
        # pretrained_model = torch.load(path)
        # self.load_state_dict(pretrained_model['fc1'])
        # self.load_state_dict(pretrained_model['fc2'])
        self.apply(self._init_weights)

    def _init_weights(self, module):
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            module.bias.data.zero_()
        
    def forward(self,x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        value = self.value(x)
        return value




import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Normal, MultivariateNormal
import numpy as np
import os

class Agent:
    def __init__(self, state_dim, action_dim, lr=1e-4, gamma=0.99, eps_clip=0.2, 
                 epsilon=0.2, lmbda=0.95, epochs=30, batch_size=32, 
                 value_loss_coef=0.5, entropy_coef=0.01, 
                 clip_range_start=0.2, clip_range_end=0.02, 
                 max_grad_norm=1.0, patience=10):
        self.actor_network = ActorNetwork(action_dim, state_dim)
        self.critic_network = CriticNetwork(state_dim)
        self.actor_optimizer = optim.Adam(self.actor_network.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic_network.parameters(), lr=lr)
        self.actor_scheduler = optim.lr_scheduler.ExponentialLR(self.actor_optimizer, gamma=0.995)
        self.critic_scheduler = optim.lr_scheduler.ExponentialLR(self.critic_optimizer, gamma=0.995)
        
        self.gamma = gamma
        self.epsilon = epsilon
        self.lmbda = lmbda
        self.epochs = epochs
        self.batch_size = batch_size
        self.eps_clip = eps_clip
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.clip_range_start = clip_range_start
        self.clip_range_end = clip_range_end
        self.clip_range = clip_range_start
        self.max_grad_norm = max_grad_norm
        
        self.MseLoss = nn.MSELoss()
        self.actor_loss = 0
        self.critic_loss = 0
        
        # Early stopping
        self.patience = patience
        self.best_reward = -float('inf')
        self.patience_counter = 0
        
        # Replay buffer (simplified, you might want to implement a more sophisticated version)
        self.replay_buffer = []
        self.buffer_size = 10000

    def save_models(self, path='models'):
        if not os.path.exists(path):
            os.makedirs(path)
        torch.save(self.actor_network.state_dict(), os.path.join(path, 'actor.pth'))
        torch.save(self.critic_network.state_dict(), os.path.join(path, 'critic.pth'))
        print(f"Models saved to {path}")

    def load_models(self, path='models'):
        self.actor_network.load_state_dict(torch.load(os.path.join(path, 'actor.pth')))
        self.critic_network.load_state_dict(torch.load(os.path.join(path, 'critic.pth')))
        print(f"Models loaded from {path}")

    def select_action(self, state):
        with torch.no_grad():
            state = torch.tensor(state, dtype=torch.float32).unsqueeze(0)
            mean, std = self.actor_network(state)
        mean = torch.nan_to_num(mean, nan=0.0)
        std = torch.nan_to_num(std, nan=1.0)
        cov_matrix = torch.diag(std**2) 
        dist = MultivariateNormal(mean, covariance_matrix=cov_matrix)
        action = dist.sample()
        action = 2 * (torch.tanh(action))
        action_log_prob = dist.log_prob(action)
        return action.detach().numpy()[0], action_log_prob.detach()
    
    def compute_advantages(self, rewards, values, next_values, dones):
        advantages = []
        gae = 0
        for step in reversed(range(len(rewards))):
            delta = rewards[step] + self.gamma * next_values[step] * (1 - dones[step]) - values[step]
            gae = delta + self.gamma * self.lmbda * (1 - dones[step]) * gae
            advantages.insert(0, gae)
        return advantages

    def update_clip_range(self, progress):
        self.clip_range = self.clip_range_start + progress * (self.clip_range_end - self.clip_range_start)

    def add_to_replay_buffer(self, experience):
        self.replay_buffer.append(experience)
        if len(self.replay_buffer) > self.buffer_size:
            self.replay_buffer.pop(0)

    def sample_from_replay_buffer(self, batch_size):
        return random.sample(self.replay_buffer, min(batch_size, len(self.replay_buffer)))

    def learn(self, trajectories):
        if not trajectories:
            print("Warning: Empty trajectories. Skipping learning step.")
            return
        
        states, actions, log_probs, rewards, next_states, dones = zip(*trajectories)
        
        states = torch.tensor(states, dtype=torch.float32)
        actions = torch.tensor(actions, dtype=torch.float32)
        old_log_probs = torch.stack(log_probs)
        rewards = torch.tensor(rewards, dtype=torch.float32)
        next_states = torch.tensor(next_states, dtype=torch.float32)
        dones = torch.tensor(dones, dtype=torch.bool)
        
        with torch.no_grad():
            values = self.critic_network(states).squeeze()
            next_values = self.critic_network(next_states).squeeze()
            advantages = self.compute_advantages(
                rewards.cpu().numpy(), 
                values.cpu().numpy(), 
                next_values.cpu().numpy(), 
                dones.cpu().numpy()
            )
            
            advantages = torch.tensor(advantages, dtype=torch.float32)
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

            returns = advantages + values

        for _ in range(self.epochs):
            for i in range(0, len(states), self.batch_size):
                batch_indices = slice(i, i + self.batch_size)
                batch_states = states[batch_indices]
                batch_actions = actions[batch_indices]
                batch_log_probs = old_log_probs[batch_indices]
                batch_returns = returns[batch_indices]
                batch_advantages = advantages[batch_indices]

                mean, std = self.actor_network(batch_states)
                dist = Normal(mean, std)
                new_log_probs = dist.log_prob(batch_actions).sum(dim=-1)
                
                log_ratio = new_log_probs - batch_log_probs
                log_ratio = torch.clamp(log_ratio, -20, 20)
                
                ratios = torch.exp(log_ratio)

                surr1 = ratios * batch_advantages
                surr2 = torch.clamp(ratios, 1.0 - self.clip_range, 1.0 + self.clip_range) * batch_advantages
                
                # Actor loss
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Entropy bonus
                entropy = dist.entropy().mean()
                
                # Critic loss
                value_pred = self.critic_network(batch_states).squeeze()
                value_loss = F.mse_loss(value_pred, batch_returns)
                
                # Total loss
                total_loss = policy_loss + self.value_loss_coef * value_loss - self.entropy_coef * entropy

                # Update critic
                self.critic_optimizer.zero_grad()
                value_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.critic_network.parameters(), self.max_grad_norm)
                self.critic_optimizer.step()

                # Update actor
                self.actor_optimizer.zero_grad()
                policy_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.actor_network.parameters(), self.max_grad_norm)
                self.actor_optimizer.step()

        # Step the learning rate schedulers
        self.actor_scheduler.step()
        self.critic_scheduler.step()

        self.actor_loss = policy_loss.item()
        self.critic_loss = value_loss.item()

    def check_early_stopping(self, current_reward):
        if current_reward > self.best_reward:
            self.best_reward = current_reward
            self.patience_counter = 0
        else:
            self.patience_counter += 1
        
        return self.patience_counter >= self.patience

def evaluate(agent, env, num_episodes=5):
    total_rewards = []
    episode_lengths = []
    for _ in range(num_episodes):
        observation = env.reset()
        episode_reward = 0
        done = False
        step = 0
        while not done and step < max_timesteps:
            action, _ = agent.select_action(observation)
            observation, reward, done, _ = env.step(action)
            episode_reward += reward
            step += 1
        total_rewards.append(episode_reward)
        episode_lengths.append(step)
    return np.mean(total_rewards), np.std(total_rewards), np.mean(episode_lengths)


def evaluate_saved_model(model_path, env, num_episodes=50):
    evaluation_agent = Agent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0])
    evaluation_agent.load_models(model_path)
    
    mean_reward, std_reward, mean_length = evaluate(evaluation_agent, env, num_episodes=num_episodes)
    print(f"Evaluation of model from {model_path}:")
    print(f"Mean reward: {mean_reward:.2f} ± {std_reward:.2f}, Mean length: {mean_length:.2f}")
    
    return mean_reward, std_reward, mean_length



if __name__ == '__main__':

    random.seed(1)
    np.random.seed(1)
    torch.manual_seed(1)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


    env = Jaco2Env()
    print("observation space dimension :",env.observation_space.shape[0])
    print("action space dimension :",env.action_space.shape[0])
    agent = Agent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0])
    time.sleep(10)
    actor_lr = 0.001
    critic_lr = 0.001
    batch_size = 32
    # TIME_DELTA = 0


   
    # # env.reset()
    n_episodes = 1000
    max_timesteps = 500
    eval_interval = 100
    episode_rewards = []
    evaluate_interval = 100  # Set to None to disable intermediate evaluation
    save_best_model = True   # Set to False if you only want to save the final model

    best_eval_reward = float('-inf')
    current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    log_dir = f'runs/jaco2_ppo_{current_time}'
    writer = SummaryWriter(log_dir)

    # agent.load_models()
    print("Training begins")
    try :
        for i in range(n_episodes):
            #print("Epsiode :", i)
            observation = env.reset()
            # print(("test2"))
            done = False
            score = 0
            # print("test3")
            trajectories = []
            #print("............Environment resetted..............")
            for t in range(max_timesteps):
                #print(f"  Step: {t}")
                # print("observation :", observation)
                action,log_prob = agent.select_action(observation)
                #print(f"  Action selected: {action}")
                next_observation, reward, done, info = env.step(action)
                #print(f"  Reward: {reward}")
                score += reward
                # print("action :",action)
                trajectories.append([observation, action,log_prob, reward, next_observation, done])
                observation = next_observation

                if done :
                    print("Episodic task completed early")
                    break
            print(f"Total reward for {i} episode is {score}")

            episode_rewards.append(score)
            writer.add_scalar('Training/Episode Reward', score, i)
            writer.add_scalar('Training/Episode Length', t+1, i)


            #print("Learning.....")
            # trajectories = torch.FloatTensor(np.array(trajectories))
            agent.learn(trajectories)
            #print("Learning finished for ", i , "episode")

            writer.add_scalar('Training/Actor Loss', agent.actor_loss, i)
            writer.add_scalar('Training/Critic Loss', agent.critic_loss, i)

            # print(f"Episode {i} finished. Reward: {episode_reward}")

            if (i + 1) % eval_interval == 0:
                mean_reward, std_reward, mean_length = evaluate(agent, env)
                writer.add_scalar('Evaluation/Mean Reward', mean_reward, i)
                writer.add_scalar('Evaluation/Std Reward', std_reward, i)
                writer.add_scalar('Evaluation/Mean Episode Length', mean_length, i)
                
                print(f"Evaluation after {i+1} episodes: Mean reward: {mean_reward:.2f} ± {std_reward:.2f}, Mean length: {mean_length:.2f}")
                
                if mean_reward > best_eval_reward:
                    best_eval_reward = mean_reward
                    agent.save_models(f'best_model_episode_{i+1}')
                    print(f"New best model saved at episode {i+1}")

        print(episode_rewards)
        #Save the model after training
        agent.save_models()

        #Plot episode rewards
        # plt.figure(figsize=(10, 5))
        # plt.plot(episode_rewards)
        # plt.title('Episode Rewards')
        # plt.xlabel('Episode')
        # plt.ylabel('Reward')
        # plt.savefig('episode_rewards.png')
        # plt.show()

    
        # Final evaluation
        print("loading model")
        agent.load_models()
        print("model loaded")
        final_mean_reward, final_std_reward, final_mean_length = evaluate(agent, env, num_episodes=50)
        print(f"Final evaluation: Mean reward: {final_mean_reward:.2f} ± {final_std_reward:.2f}, Mean length: {final_mean_length:.2f}")
        
        # # Evaluate the best model
        # best_model_path = f'best_model_episode_{best_episode}'  # Replace best_episode with the actual episode number
        # evaluate_saved_model(best_model_path, env)

        # # Evaluate the final model
        # evaluate_saved_model('final_model', env)

        writer.close()

            # if(i%10):
        #     agent.save_models()
        # print(f"Episode : {i}, score : {score}")
    
    except rospy.ROSInterruptException:
        pass

    finally : 
        rospy.signal_shutdown("Training complete")
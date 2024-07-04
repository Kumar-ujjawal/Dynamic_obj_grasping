## Q-RL Velocity Control Training Flowchart

```plaintext
Start
|
v
Initialize ROS Node and Logging
|
v
Initialize KinovaEnvRL (Environment Setup)
|
v
Initialize QLearningAgent (Agent Setup)
|
v
For each episode in total_episodes:
|
|--> Reset environment (env.reset())
|    |
|    v
|    Reset Gazebo simulation (/gazebo/reset_simulation service call)
|    |
|    v
|    Wait for simulation reset to complete (rospy.sleep(1))
|    |
|    v
|    Generate random goal position within bounds (np.random.uniform(low=-2, high=2, size=3))
|    |
|    v
|    Set initial joint positions (self.set_joint_position(i, pos))
|    |
|    v
|    Update current distance to goal (self._get_distance_to_goal())
|    |
|    v
|    Initialize done = False, total_reward = 0, step = 0
|    |
|    v
|    While not done and step < max_steps_per_episode:
|    |
|    |--> Get feedback weight from environment (env.get_reward_feedback())
|    |    |
|    |    v
|    |    Choose action using epsilon-greedy policy (agent.get_action(state, feedback_weight))
|    |    |
|    |    v
|    |    Convert action to joint velocities (joint_velocities)
|    |    |
|    |    v
|    |    Publish joint velocities to Gazebo (/j2s7s300/joint_X_velocity_controller/command topics)
|    |    |
|    |    v
|    |    Wait for action to take effect (rospy.sleep(0.1))
|    |    |
|    |    v
|    |    Receive joint states (positions, velocities, torques) from Gazebo (/j2s7s300/joint_states topic)
|    |    |
|    |    v
|    |    Compute current distance to goal (self._get_distance_to_goal())
|    |    |
|    |    v
|    |    Compute reward based on distance, joint limits, and smoothness (self._compute_reward())
|    |    |
|    |    v
|    |    Check if episode is done (self._is_done())
|    |    |
|    |    v
|    |    Update Q-table (agent.update(state, action, reward, next_state, done))
|    |    |
|    |    v
|    |    Update current state to next state
|    |    |
|    |    v
|    |    Accumulate total reward and increment step
|    |
|    End While
|    |
|    v
|    Update curriculum stage if necessary (env.update_curriculum())
|    |
|    v
|    Decay epsilon for exploration (agent.decay_epsilon())
|    |
|    v
|    Log episode completion and total reward
|    |
|    v
|    Sleep for a short interval between episodes
|
End For
|
v
Close environment (env.close())
|
v
Stop

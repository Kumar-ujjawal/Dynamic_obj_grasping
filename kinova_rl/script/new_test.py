#!/home/vr-lab/anaconda3/envs/bin/python
import rospy
import numpy as np
import torch
from ppo import Jaco2Env  # Make sure to import your environment
from ppo import Agent, ActorNetwork, CriticNetwork  # Import your agent and network classes
import time
TIME_DELTA = 0
def test_model(env, agent, target_position, max_steps=1000, distance_threshold=0.9):
    observation = env.reset()
    success = False
    for step in range(max_steps):
        action, _ = agent.select_action(observation)
        observation, reward, done, _ = env.step(action)
        
        current_position = observation[-3:]  # Assuming last 3 elements are x, y, z
        distance = np.linalg.norm(current_position - target_position)
        
        if distance < distance_threshold:
            success = True
            return True, step + 1
        
        if done:
            break
    
    return success, step + 1

def main():
    print("Initializing environment...")
    env = Jaco2Env()
    print("Environment initialized. Waiting for 10 seconds to ensure everything is set up...")
    time.sleep(10)  # Wait for 10 seconds to ensure environment is fully initialized

    print("Creating agent...")
    agent = Agent(state_dim=env.observation_space.shape[0], action_dim=env.action_space.shape[0])
    for i in range(3):

        print("Loading trained model...")
        agent.load_models('mod_lvl1')
        print("Model loaded successfully!")

    # Wait for user input before starting the tests
        input("Press Enter to start testing...")

    # Define 5 target positions
        target_positions = [
        np.array([2.0, 2.0, 2.0]),
        np.array([2.0, 2.0, 2.0]),
        np.array([2.0, 2.0, 2.0]),
        np.array([2.0, 2.0, 2.0]),
        np.array([2.0, 2.0, 2.0]),
        np.array([2.0, 2.0, 2.0])
        ]
    
        successful_attempts = 0
        total_steps = 0
    
        for i, target in enumerate(target_positions):
            print(f"\nAttempting to reach target {i+1}: {target}")
            success, steps = test_model(env, agent, target)
            total_steps += steps
        
            if success:
                successful_attempts += 1
                print(f"Success! Reached the target in {steps} steps.")
            else:
                print(f"Failed to reach the target within the step limit.")
        
        # Wait for a moment between attempts
        time.sleep(2)
    
        print(f"\nResults:")
        print(f"Successful attempts: {successful_attempts} out of 5")
        print(f"Success rate: {successful_attempts/5*100:.2f}%")
        print(f"Average steps per attempt: {total_steps/5:.2f}")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
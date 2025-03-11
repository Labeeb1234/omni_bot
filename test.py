import numpy as np
from omni_bot.env import OmniBotEnv
import time  # For adding delay to prevent high CPU usage

env = OmniBotEnv()  
done = False
state, _ = env.reset()  # Get initial state
step_count = 0
max_steps = 500  # Avoid infinite loops

while not done and step_count < max_steps:
    # Define actions for 3 robots: [vx, vy, omega]
    action = np.array([
        [0.0, 0.0, 0.1],   
        [1.0, 0.0, 0.1],   
        [0.0, -1.0, 0.1]  
    ])
    
    # Ensure action shape is correct
    assert action.shape == (3, 3), f"Action shape mismatch! Expected (3,3), got {action.shape}"
    
    # Take a step in the environment
    new_state, reward, dones, _ = env.step(actions=action)

    # Stop simulation if any robot reaches done state
    done = any(dones)  
    
    # Debugging Output (Shows each robot's progress)
    for i in range(len(new_state)):
        print(f"Step {step_count} | Robot {i+1} | State: {new_state[i]} | Reward: {reward[i]} | Done: {dones[i]}")

    # Update visualization
    env.render()

    # Prevent excessive CPU usage
    time.sleep(0.05)
    
    step_count += 1  

print("Simulation Complete.")


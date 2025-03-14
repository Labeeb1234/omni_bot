import numpy as np
from omni_bot.env import OmniBotEnv
import time  # For adding a slight delay

env = OmniBotEnv()  # Initialize environment
done = False
state, _ = env.reset()  # Get initial state

while not done:
    action = np.array([
        [0.0, 0.0, 0.1],
        [1.0, 0.0, 0.1],
        [0.0, -1.0, 0.1]
    ])  # Rotate in place
    new_state, reward, dones, _ = env.step(actions=action)  # Take action
    done = dones[1]

    print(f"State: {new_state}, Reward: {reward}")  # Debug output
    env.render()  # Update visualization

    time.sleep(0.05)  # Prevents excessive CPU usage, allows rendering 

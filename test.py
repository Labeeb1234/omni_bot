import numpy as np
from omni_bot.env_gym import OmniBotEnv
import time  # For adding a slight delay

env = OmniBotEnv(render_mode="human")  # Initialize environment
done = False
state, _ = env.reset()  # Get initial state
print(state)
while not done:
    action = np.array([
        [1.0, 1.0, 0.0],
    ])  # Rotate in place
    new_state, reward, dones, _ = env.step(actions=action)  # Take action
    print(env.robot.pose)
    print(f"State: {new_state}, Reward: {reward}, dones: {dones}")  # Debug output
    env.render()  # Update visualization


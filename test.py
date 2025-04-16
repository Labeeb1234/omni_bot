import numpy as np
import omni_bot.envs
import gymnasium as gym
import time  # For adding a slight delay

env = gym.make('OmniBot-v0', render_mode='human')

done = False
state, _ = env.reset()
count = 0

while not done:
    action = 0
    new_state, reward, done, trunc, _ = env.step(action=action)
    print(f"State: {new_state}, Reward: {reward}, dones: {done}")
    env.render()
    count += 1

env.close()







import numpy as np
from omni_bot.env import OmniBotEnv


env = OmniBotEnv()
done = False
state, _ = env.reset()

while not done:
    action = [0.0, 0.0, 0.1]
    new_state, reward, done, _ = env.step(action=action)
    print(f"{new_state}")
    env.render()

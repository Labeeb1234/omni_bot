from omni_bot import *
from omni_bot.utils.utils import *
from omni_bot.teleop import *
from omni_bot.envs.env_gym import OmniBotEnv
from gymnasium.envs.registration import register

register(
    id='OmniBot-v0',
    entry_point='omni_bot.envs.env_gym:OmniBotEnv',
)







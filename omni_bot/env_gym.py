# import gym
# from gym import spaces
import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import pygame
from omni_bot.omni_bot import OmniBot  

class OmniBotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, SIZE=[5, 5], dt=0.05):
        super(OmniBotEnv, self).__init__()

        self.robot = OmniBot(a=0.05, L=0.3, W=0.3, t=0.036, m=4, Icz=0.1)
        self.robot_trajectory = []
        [self.size_x, self.size_y] = SIZE # size of the env in [m]

        # Gym Action & Observation Space
        # self.action_space = spaces.Box(low=-1, high=1, shape=(num_robots, 3), dtype=np.float32)
        # self.observation_space = spaces.Box(low=-5, high=5, shape=(num_robots, 9), dtype=np.float32)  # Pos, Vel, Lidar

        self.window_size = 512
        self.scale = self.window_size/(2*self.size_x)
        self.clock = None
        self.window =None

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.dt = 1/30

    def world_to_window(self, pose): # unit conversion from m->pixels
        screen_x = int((pose[0] + self.size_x) * self.scale)
        screen_y = int((self.size_y - pose[1]) * self.scale)
        return screen_x,screen_y
       
    def step(self, actions): # Apply action to each robot, update state, and return observations
        rewards = []
        done_flags = []
        observations = []
        # one step process
        omega = self.robot.inverse_kinematics(body_vel=actions)
        eta_dot = self.robot.forward_kinematics(omega=omega)
        self.robot.update_odom(vel_global=eta_dot, dt=self.dt)
        obs = np.array([
            self.robot.pose[0, 0], self.robot.pose[1, 0], self.robot.pose[2, 0],
            eta_dot[0, 0], eta_dot[1, 0], eta_dot[2, 0]
            # lidar_data[0], lidar_data[1], lidar_data[2]
        ], dtype=np.float32)

        reward = 0.0
        done = np.linalg.norm(self.robot.pose[0:2]) > 4.5

        self.robot_trajectory.append(self.robot.pose)
        observations.append(obs)
        rewards.append(reward)
        done_flags.append(done)

        return np.array(observations), np.array(rewards), np.array(done_flags), {}

    def reset(self):
        #Reset all robots to the origin (for now can add a randomizer to randomize reset pose of the bots).
        self.robot.pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)  
        return np.array([[self.robot.pose[0,0], self.robot.pose[1,0], self.robot.pose[2,0]]], dtype=np.float32), {}

    def render(self):
        if self.render_mode == "human":
            return self._render_frame()
    
    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()
        
        # Create a canvas
        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))
        # Draw origin axes
        pygame.draw.line(canvas, (255, 0, 0), self.world_to_window([0,0]), self.world_to_window([0.5, 0]), 3)  # X-axis
        pygame.draw.line(canvas, (0, 255, 0), self.world_to_window([0,0]), self.world_to_window([0, 0.5]), 3)  # Y-axis

        bot_body = self.robot.get_bot_outline() # (4x2) matrix (array of [x,y])
        body_outline = [self.world_to_window(corner) for corner in bot_body]
        # adding bot_outline
        pygame.draw.polygon(canvas, (255, 216, 0), body_outline)

        # wheel outlines (going with black circular wheels for now)
        wheel_positions = self.robot.get_wheel_positions()
        wheel_centres = [self.world_to_window(wheel_pose) for wheel_pose in wheel_positions]
        for wheel_centre in wheel_centres:
            pygame.draw.circle(canvas, (0, 0, 0), wheel_centre, 5)
        
        if self.render_mode == "human":
            # The following line copies our drawings from `canvas` to the visible window
            self.window.blit(canvas, (0,0))
            pygame.event.pump()
            pygame.display.update()
            # We need to ensure that human-rendering occurs at the predefined framerate.
            # The following line will automatically add a delay to keep the framerate stable.
            self.clock.tick(self.metadata["render_fps"])
        else:  # rgb_array
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(canvas)), axes=(1, 0, 2)
            )

    def save_trajectory(self, filename="robot_trajectory.npy"):
        #Save robot trajectories to a file.
        np.save(filename, np.array(self.robot_trajectory))

    def load_trajectory(self, filename="robot_trajectory.npy"):
        #Load and visualize previous trajectories.
        data = np.load(filename, allow_pickle=True)
        plt.plot(data[0], data[1], 'r--', label=f'Robot')
        plt.legend()
        plt.show()

    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
from typing import Optional
import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import pygame
from omni_bot.omni_bot import OmniBot  

class OmniBotEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode=None, SIZE=[5, 5], dt=0.05, grid_size=(20, 20), cell_size=25):
        super(OmniBotEnv, self).__init__()

        self.robot = OmniBot(a=0.05, L=0.3, W=0.3, t=0.036, m=4, Icz=0.1)
        self.robot_trajectory = [] # in [m]
        self.robot_trace = [] # in pygame pixel units
    
        [self.size_x, self.size_y] = SIZE # size of the env in [m]
        self.grid_size = grid_size
        self.cell_size = cell_size

        # Define the occupancy grid (0 = free, 1 = occupied, 0.5 = unknown (not in use for now))
        self.occupancy_grid = np.random.choice([0, 1], size=grid_size, p=[0.99, 0.01])
        self.resolution = self.size_x/self.grid_size[0] # square physical env

        # defining the gymnasium observation and action spaces
        self.observation_space = gym.spaces.Dict({
            "robot_pose": gym.spaces.Box(-self.size_x+1, self.size_x-1, shape=(3,), dtype=float)
        })
        self.action_space = gym.spaces.Discrete(6)
        self._action_to_direction = {
            0: np.array([1.0, 0.0, 0.0]), # forward
            1: np.array([-1.0, 0.0, 0.0]), # backward
            2: np.array([0.0, 1.0, 0.0]), # strafe forward
            3: np.array([0.0, -1.0, 0.0]), # strafe backward
            4: np.array([0.0, 0.0, 1.0]), # turn left
            5: np.array([0.0, 0.0, -1.0]), # turn right
        }

        # ----------------------------------------------------
        self.window_size = 500
        self.scale = self.window_size/(self.size_x) # from [m] to [pygame pixels]
        self.clock = None
        self.window =None

        assert render_mode is None or render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.dt = 1/self.metadata["render_fps"]

    def world_to_window(self, pose): # unit conversion from m->pixels
        screen_x = int((pose[0] + self.size_x/2) * self.scale)
        screen_y = int((self.size_y/2 - pose[1]) * self.scale)
        return screen_x,screen_y

    def occgrid_to_world(self, pixel, map_origin=(0.0,0.0)): # map origin is in metres scale ,  pixel in (y_pix, x_pix)
        x_m = (pixel[1] - self.grid_size[1] // 2 + 0.5) * self.resolution - map_origin[0]
        y_m = -(pixel[0] - self.grid_size[0] // 2 + 0.5) * self.resolution - map_origin[1]
        return x_m, y_m
    
    def occgrid_to_window(self, pixel):
        pose = self.occgrid_to_world(pixel)
        return self.world_to_window(pose)
    
    def get_obstacle_info(self):
        indices = np.where(self.occupancy_grid==1)
        obstacle_coords = list(zip(indices[0], indices[1]))
        return obstacle_coords
        
    def _draw_obstacle_circle_cost(self, canvas, epsilon=5.0):
        obstacle_coords = self.get_obstacle_info()
        for obs in obstacle_coords:
            world_x, world_y = self.occgrid_to_world(obs)
            print(f"Obstacle at grid {obs} is at world coordinates: ({world_x:.2f}, {world_y:.2f})") 
            x, y = self.occgrid_to_window(obs)
            pygame.draw.circle(canvas, (255, 0, 0), (x, y), epsilon/self.resolution, 2)
    
    def step(self, action): # Apply action to robot, update state, reward and return observations
        # rewards = []
        # done_flags = []
        # observations = []

        # one step process
        actions = self._action_to_direction[action]
        omega = self.robot.inverse_kinematics(body_vel=actions)
        eta_dot = self.robot.forward_kinematics(omega=omega)
        self.robot.update_odom(vel_global=eta_dot, dt=self.dt)
        
        # storing robot trajectory as real world or pixel coords
        self.robot_trace.append(self.world_to_window([self.robot.pose[0,0], self.robot.pose[1,0]])) # in pygame pixel coords
        self.robot_trajectory.append([self.robot.pose[0,0], self.robot.pose[1,0]]) # in real world coords

        observations = {"robot_pose": self.robot.pose}
        reward = 0.0
        terminated = np.linalg.norm(self.robot.pose[0:2]) > 4.5 # done if bot goes out of bounds
        truncated = False
        info = {}

        if self.render_mode == "human":
            self._render_frame()
        
        # observations.append(obs)
        # rewards.append(reward)
        # done_flags.append(done)
        return observations, reward, terminated, truncated, info

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None):
        # Reset all robots to the origin and other observations to initial states of the env.
        super().reset(seed=seed, options=options)

        self.robot.pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)
        observations = {"robot_pose": self.robot.pose}
        info = {}
        return observations, info

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()
    
    def _render_frame(self):
        if self.window is None and self.render_mode == "human":
            pygame.init()
            pygame.display.init()
            pygame.display.set_caption('omni_bot_env')
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        
        if self.clock is None and self.render_mode == "human":
            self.clock = pygame.time.Clock()
        
        # Create a canvas
        canvas = pygame.Surface((self.window_size, self.window_size))
        canvas.fill((255, 255, 255))
        
        # creating the occupancy grid based map-env
        # Draw the occupancy grid
        self._draw_occupancy_grid(canvas)
        self._draw_obstacle_circle_cost(canvas)

        # Draw origin axes
        pygame.draw.line(canvas, (255, 0, 0), self.world_to_window([0,0]), self.world_to_window([0.3, 0]), 3)  # X-axis
        pygame.draw.line(canvas, (0, 255, 0), self.world_to_window([0,0]), self.world_to_window([0, 0.3]), 3)  # Y-axis

        bot_body = self.robot.get_bot_outline() # (4x2) matrix (array of [x,y])
        body_outline = [self.world_to_window(corner) for corner in bot_body]
        # adding bot_outline
        pygame.draw.polygon(canvas, (255, 216, 0), body_outline)

        # wheel outlines (going with black circular wheels for now)
        wheel_positions = self.robot.get_wheel_positions()
        wheel_centres = [self.world_to_window(wheel_pose) for wheel_pose in wheel_positions]
        for wheel_centre in wheel_centres:
            pygame.draw.circle(canvas, (0, 0, 0), wheel_centre, 5)

        # robot trajectory trace
        for trace in self.robot_trace:
            pygame.draw.circle(canvas, (0, 255, 0), trace, 1, 1)
        
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
    
    def _draw_occupancy_grid(self, canvas):
        for y in range(self.grid_size[0]):
            for x in range(self.grid_size[1]):
                pixel_value = self.occupancy_grid[y, x]
                if pixel_value == 0:
                    color = (255, 255, 255) # free space pixel_values < (0,255) >
                elif pixel_value == 1:
                    color = (0, 0, 0)

                # drawing the grid-cell (each cell is a square)
                pygame.draw.rect(
                    canvas,
                    color,
                    pygame.Rect(
                        x*self.cell_size,
                        y*self.cell_size,
                        self.cell_size,
                        self.cell_size
                    )
                )

    def save_trajectory(self, filename="robot_trajectory.npy"):
        #Save robot trajectories to a file.
        np.save(filename, np.array(self.robot_trajectory))

    def load_trajectory(self, filename="robot_trajectory.npy"):
        #Load and visualize previous trajectories.
        data = np.load(filename, allow_pickle=True)
        plt.plot(data[:, 0], data[:, 1], 'r--', label=f'Robot')
        plt.axis('equal')
        plt.legend()
        plt.grid()
        plt.show()
        
    def close(self):
        if self.window is not None:
            pygame.display.quit()
            pygame.quit()
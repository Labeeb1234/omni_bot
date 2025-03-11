# import gym
# from gym import spaces
import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from omni_bot.omni_bot import OmniBot  

class OmniBotEnv(gym.Env):
    def __init__(self, num_robots=3, SIZE=[5, 5], dt=0.05):
        super(OmniBotEnv, self).__init__()

        self.num_robots = num_robots
        self.robots = [OmniBot(a=0.1, L=0.3, W=0.3, t=0.01) for _ in range(num_robots)]
        self.dt = dt
        [self.size_x, self.size_y] = SIZE

        # Gym Action & Observation Space
        # self.action_space = spaces.Box(low=-1, high=1, shape=(num_robots, 3), dtype=np.float32)
        # self.observation_space = spaces.Box(low=-5, high=5, shape=(num_robots, 9), dtype=np.float32)  # Pos, Vel, Lidar

        # Matplotlib Visualization
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-self.size_x, self.size_x)
        self.ax.set_ylim(-self.size_y, self.size_y)
        self.ax.set_aspect('equal')

        # Origin Frame
        self.ax.arrow(0, 0, 0.5, 0, head_width=0.2, fc='red')
        self.ax.arrow(0, 0, 0, 0.5, head_width=0.2, fc='green') 
        self.ax.text(0.6, 0, 'X', color='red')
        self.ax.text(0, 0.6, 'Y', color='green')

        # Robot Patches ( body patches , trails, & wheel patches for animation)
        self.body_patches = []
        self.trails = []
        self.wheel_patches = []
        self.robot_x = [[] for _ in range(num_robots)]
        self.robot_y = [[] for _ in range(num_robots)]

        for bot in self.robots: # Initialize graphical elements for each  robot
            body_patch = patches.Polygon(bot.get_bot_outline(), closed=True, color='blue', alpha=0.6) # polygon patch for robot body 
            self.ax.add_patch(body_patch)
            self.body_patches.append(body_patch)

            trail, = self.ax.plot([], [], 'g--', linewidth=1.0) # trail line for trajectory visualization
            self.trails.append(trail)

            wheels = []
            for pos in bot.get_wheel_positions():
                wheel_patch = patches.Circle((pos[0], pos[1]), bot.a, color='black')
                self.ax.add_patch(wheel_patch)
                wheels.append(wheel_patch)
            self.wheel_patches.append(wheels)

        plt.grid(visible=True)
        self.ani = animation.FuncAnimation(self.fig, self.update_animation, frames=200, interval=100, blit=False)

    def step(self, actions): # Apply action to each robot, update state, and return observations
        rewards = []
        done_flags = []
        observations = []

        goal = np.array([4, 4]).reshape(2,1)  # Goal for all robots ( change accordindly )

        for i, bot in enumerate(self.robots):
            omega = bot.inverse_kinematics(body_vel=actions[i])
            eta = bot.forward_kinematics(omega=omega)
            bot.update_odom(vel_global=eta, dt=self.dt)

            # Save robot trajectory
            self.robot_x[i].append(bot.pose[0, 0])
            self.robot_y[i].append(bot.pose[1, 0])

            # Lidar Data Simulation
            # lidar_data = bot.get_lidar_readings()  # [front, left, right]

            # Reward based on distance to goal
            distance_to_goal = np.linalg.norm(bot.pose[0:2] - goal)
            reward = -distance_to_goal  # Encourage moving closer

            # Penalty for leaving boundaries
            done = np.linalg.norm(bot.pose[0:2]) > 4.5
            if done:
                reward -= 10

            # Observations: [x, y, theta, vx, vy, omega, lidar1, lidar2, lidar3]
            obs = np.array([
                bot.pose[0, 0], bot.pose[1, 0], bot.pose[2, 0],
                eta[0, 0], eta[1, 0], eta[2, 0]
                # lidar_data[0], lidar_data[1], lidar_data[2]
            ], dtype=np.float32)

            observations.append(obs)
            rewards.append(reward)
            done_flags.append(done)

        return np.array(observations), np.array(rewards), np.array(done_flags), {}

    def reset(self):
        #Reset all robots to the origin (for now can add a randomizer to randomize reset pose of the bots).
        for bot in self.robots:
            bot.pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)  
        return np.array([[bot.pose[0,0], bot.pose[1,0], bot.pose[2,0]] for bot in self.robots], dtype=np.float32), {}

    def update_animation(self, frame):
       # Update function for Matplotlib animation.
        for i, bot in enumerate(self.robots):
            self.body_patches[i].set_xy(bot.get_bot_outline())
            self.trails[i].set_data(self.robot_x[i], self.robot_y[i])
            # Update wheel positions
            wheel_positions = bot.get_wheel_positions()
            for j, wheel_patch in enumerate(self.wheel_patches[i]):
                wheel_patch.center = (wheel_positions[j, 0], wheel_positions[j, 1])
            
        return self.body_patches + [p for sublist in self.wheel_patches for p in sublist]

    def render(self, mode='human'):
        #Update visualization manually if needed.
        self.update_animation(None)
        self.fig.canvas.draw_idle()
        plt.pause(0.01)

    def save_trajectory(self, filename="robot_trajectory.npy"):
        #Save robot trajectories to a file.
        np.save(filename, np.array([self.robot_x, self.robot_y]))

    def load_trajectory(self, filename="robot_trajectory.npy"):
        #Load and visualize previous trajectories.
        data = np.load(filename, allow_pickle=True)
        for i in range(self.num_robots):
            plt.plot(data[0][i], data[1][i], 'r--', label=f'Robot {i+1}')
        plt.legend()
        plt.show()

    def close(self):
        plt.close(self.fig)

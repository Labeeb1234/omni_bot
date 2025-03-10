# import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from omni_bot.omni_bot import OmniBot


class OmniBotEnv:
    def __init__(self, SIZE=[5, 5], dt=0.05):
        self.bot = OmniBot(a=0.1, L=0.3, W=0.3, t=0.01) # all measurements in metres
        self.dt = dt # time_step 
        [self.size_x, self.size_y] = SIZE

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-self.size_x, self.size_x)
        self.ax.set_ylim(-self.size_y, self.size_y)
        self.ax.set_aspect('equal')

        # adding origin axis (global frame axis representation)
        self.ax.arrow(0, 0, 0.5, 0, head_width=0.2, head_length=0.2, fc='red', ec='red')  # X-axis arrow
        self.ax.arrow(0, 0, 0, 0.5, head_width=0.2, head_length=0.2, fc='green', ec='green')  # Y-axis arrow
        self.ax.text(0.6, 0, 'X', color='red')
        self.ax.text(0, 0.6, 'Y', color='green')

        # Draw Bot Base
        self.body_patch = patches.Polygon(self.bot.get_bot_outline(), closed=True, color='blue', alpha=0.6)
        self.ax.add_patch(self.body_patch)

        # Trace Bot Trajectory
        self.robot_x = []
        self.robot_y = []
        self.trail, = self.ax.plot([], [], 'g--', linewidth=1.0, label='Path Taken')

        # Draw wheels
        self.wheel_patches = []
        self.wheel_positions = self.bot.get_wheel_positions()
        for pos in self.wheel_positions:
            wheel_patch = patches.Circle((pos[0], pos[1]), self.bot.a, color='black')
            self.ax.add_patch(wheel_patch)
            self.wheel_patches.append(wheel_patch)

        plt.grid(visible=True)

        # Animation setup
        self.ani = animation.FuncAnimation(self.fig, self.update_animation, frames=200, interval=100, blit=False)
    
    def step(self, action):
        self.robot_x.append(self.bot.pose[0,0])
        self.robot_y.append(self.bot.pose[1,0])
        reward = 0.0

        omega = self.bot.inverse_kinematics(body_vel=action) # action is epsilon_desired
        eta = self.bot.forward_kinematics(omega=omega)
        self.bot.update_odom(vel_global=eta, dt=self.dt)
        

        done = np.linalg.norm(self.bot.pose[0:2]) > 4.5
        observations = np.array([self.bot.pose[0,0], self.bot.pose[1,0], self.bot.pose[2,0], eta[0,0], eta[1,0], eta[2,0]], dtype=np.float32) # need to add lidar data too (future stuff)  

        return observations, reward, done, {}

    def reset(self):
        """Reset the environment to its initial state."""
        self.bot.pose = np.array([0.0, 0.0, 0.0]).reshape(3,1)  # Reset pose to origin
        return np.array([self.bot.pose[0,0], self.bot.pose[1,0], self.bot.pose[2,0]], dtype=np.float32), {}

    def update_animation(self, frame):
        """Update function for Matplotlib animation."""
        self.body_patch.set_xy(self.bot.get_bot_outline())
        self.trail.set_data(self.robot_x, self.robot_y) # update trajectory visualization

        wheel_positions = self.bot.get_wheel_positions()
        for i, wheel_patch in enumerate(self.wheel_patches):
            wheel_patch.center = (wheel_positions[i, 0], wheel_positions[i, 1])

        return [self.body_patch] + self.wheel_patches
    
    def render(self):
        """Manually update the Matplotlib plot (if using without animation)."""
        self.update_animation(None)  # Call the animation update function
        self.fig.canvas.draw_idle()  # Redraw figure
        plt.pause(0.01)  # Allow GUI update
   
    def close(self):
        plt.close(self.fig)
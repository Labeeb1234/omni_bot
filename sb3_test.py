import time
import numpy as np

import pygame
from omni_bot.omni_bot import OmniBot

# defining the bot
bot = OmniBot(a=0.05, L=0.3, W=0.3, t=0.036, m=4, Icz=0.1)
robot_trajectory = []

# pygame window size
size_x, size_y = 5, 5  # ---> in metres
window_size = 500
scale = window_size / (2 * size_x)

def world_to_pixel_coords(pose): # pose -->[x, y, yaw]
    screen_x = int((pose[0] + size_x) * scale)
    screen_y = int((size_y - pose[1]) * scale)
    return screen_x,screen_y


# Initialize Pygame
pygame.init()
pygame.display.init()

# Set up the display
window = pygame.display.set_mode((window_size, window_size))
clock = pygame.time.Clock()

obstacles = [
    (1.0, 1.0, 0.4, 0.4),  # Obstacle 1
    (2.0, 1.5, 0.3, 0.3),  # Obstacle 2
    (2.0, -0.5, 0.5, 0.5),  # Obstacle 3
    (-2.0, -1.0, 0.6, 0.4), # Obstacle 4
    (0.0, -2.0, 0.3, 0.6)   # Obstacle 5
]

target_pose = [3.0, 1.0, 0.0] # [x, y, yaw]

def render_frame():
    # Create a canvas
    canvas = pygame.Surface((window_size, window_size))
    canvas.fill((255, 255, 255))

    # Draw origin axes
    pygame.draw.line(canvas, (255, 0, 0), world_to_pixel_coords([0,0]), world_to_pixel_coords([0.5, 0]), 3)  # X-axis
    pygame.draw.line(canvas, (0, 255, 0), world_to_pixel_coords([0,0]), world_to_pixel_coords([0, 0.5]), 3)  # Y-axis

    # Draw obstacles
    for obs in obstacles:
        obs_x, obs_y, obs_w, obs_h = obs
        pixel_x, pixel_y = world_to_pixel_coords([obs_x, obs_y])
        pixel_w = int(obs_w * scale)
        pixel_h = int(obs_h * scale)

        pygame.draw.rect(canvas, (128, 0, 128), (pixel_x, pixel_y, pixel_w, pixel_h))  # Gray obstacles


    bot_body = bot.get_bot_outline() # (4x2) matrix (array of [x,y])
    body_outline = [world_to_pixel_coords(corner) for corner in bot_body]
    # adding bot_outline
    pygame.draw.polygon(canvas, (255, 216, 0), body_outline)

    # wheel outlines (going with black circular wheels for now)
    wheel_positions = bot.get_wheel_positions()
    wheel_centres = [world_to_pixel_coords(wheel_pose) for wheel_pose in wheel_positions]
    for wheel_centre in wheel_centres:
        pygame.draw.circle(canvas, (0, 0, 0), wheel_centre, 5)

    
    # trying to trace the bot trajectory
    for trace in robot_trajectory:
        pygame.draw.circle(canvas, (0, 255, 0), trace, 1, 1)  # Green trajectory dots


    window.blit(canvas, (0, 0))
    pygame.display.flip() 

# Game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # moving bot
    body_vel = [1.0, 1.0, 1.0]
    omega = bot.inverse_kinematics(body_vel=body_vel)
    # print(f"wheel velocity: {omega}[rad/s]")
    vel_global = bot.forward_kinematics(omega=omega)
    # print(f"global frame vel: {vel_global}")
    bot.update_odom(vel_global=vel_global, dt=0.033)
    print(f"x: {bot.pose[0,0]}, y: {bot.pose[1,0]}, yaw: {bot.pose[2,0]}")
    robot_trace = world_to_pixel_coords([bot.pose[0, 0], bot.pose[1, 0]])
    robot_trajectory.append(robot_trace)

    render_frame()
    clock.tick(30)  # Limit FPS to 30

pygame.quit()

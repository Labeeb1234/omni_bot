import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from omni_bot.omni_bot import OmniBot
import threading
import numpy as np

bot = OmniBot(a=0.05, L=0.3, W=0.3, t=0.036, m=4, Icz=0.1) # all measurements in metres

fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal')

# adding origin axis (global frame axis representation)
ax.arrow(0, 0, 0.5, 0, head_width=0.2, head_length=0.2, fc='red', ec='red')  # X-axis arrow
ax.arrow(0, 0, 0, 0.5, head_width=0.2, head_length=0.2, fc='green', ec='green')  # Y-axis arrow
ax.text(0.6, 0, 'X', color='red')
ax.text(0, 0.6, 'Y', color='green')

# Draw Bot Base
body_patch = patches.Polygon(bot.get_bot_outline(), closed=True, color='blue', alpha=0.6)
ax.add_patch(body_patch)

# Draw wheels
wheel_patches = []
wheel_positions = bot.get_wheel_positions()
for pos in wheel_positions:
    wheel_patch = patches.Circle((pos[0], pos[1]), bot.a, color='black')
    ax.add_patch(wheel_patch)
    wheel_patches.append(wheel_patch)


dt = 0.05 # time_step
body_vel = np.array([0.0, 0.0, 0.0]).reshape((3,1))
n_v = np.array([0.0, 0.0, 0.0]).reshape((3,1))
def control_loop(frame):
    global body_vel, n_v
    # body_vel = [0.5, 0.0, 0.0]  # Move in x, y, and rotate
    # giving a control force/moment input (through the com of the bot)
    tau = [0.0, 1.0, 0.0] # along x
    zeta_dot = bot.forward_dynamics(tau=tau, n_v=n_v)
    print(f"body_accel: {zeta_dot}")
    body_vel = body_vel + zeta_dot*dt # ---> zeta
    print(f"calculated body frame velocity: {body_vel}")
    # Compute wheel velocities
    omega = bot.inverse_kinematics(body_vel)
    # print(f"Wheel Velocity(RPM): {omega*60/(2*np.pi)}\n")
    # Compute velocity in global frame
    vel_global = bot.forward_kinematics(omega)
    # print(f"Bot Global Velocity: {vel_global}\n")
    # bot.update_odom(vel_global, dt)
    bot.dynamic_odom_update(vel_global, zeta_dot, dt)
    print(f"Bot Pose: {bot.pose}\n")

    # Update body
    body_patch.set_xy(bot.get_bot_outline())

    # Update wheels
    wheel_positions = bot.get_wheel_positions()
    for i, wheel_patch in enumerate(wheel_patches):
        wheel_patch.center = (wheel_positions[i, 0], wheel_positions[i, 1])
    
    ax.set_xlim(bot.pose[0,0]-5, bot.pose[0,0] + 5)
    ax.set_ylim(bot.pose[1,0]-5, bot.pose[1,0] + 5)

    return body_patch, *wheel_patches

# Create animation
ani = animation.FuncAnimation(fig, control_loop, frames=200, interval=100)

plt.grid()
plt.show()


exit(0)
import numpy as np
import math
import matplotlib.pyplot as plt

def euler_to_quaternion(roll, pitch, yaw):
    '''
    Description:    Convert Euler angles (roll, pitch and yaw) to quaternion 

    Args:
        roll (float): Roll angle in radians 
        pitch (float): Pitch angle in radians
        yaw (float): Yaw angle in radians

    Returns:
        A tuple representation of quaternion (w, x, y, z)
    '''

    cy = np.cos(yaw*0.5)
    sy = np.sin(yaw*0.5)
    cp = np.cos(pitch*0.5)
    sp = np.sin(pitch*0.5)
    cr = np.cos(roll*0.5)
    sr = np.sin(roll*0.5)

    w = cy*cp*cr + sy*sp*sr
    x = cy*cp*sr - sy*sp*cr
    y = sy*cp*sr + cy*sp*cr
    z = sy*cp*cr - cr*sp*sr 
    return w, x, y, z

def quaternion_to_euler(quaternion):
    """
    Convert a quaternion [qx, qy, qz, qw] to Euler angles [roll, pitch, yaw] in radians.
    Uses the common aerospace sequence of rotations: roll (x), pitch (y), yaw (z).
    
    Args:
        quaternion: List or array containing [qx, qy, qz, qw] quaternion components
        
    Returns:
        tuple: (roll, pitch, yaw) angles in radians
    """

    qx, qy, qz, qw = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        # Use 90 degrees if out of range
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    
    return (roll, pitch, yaw)

def save_data(data: dict):
    pass

def plot_metrics(data: dict, target_path=None):
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Navigation Metrics', fontsize=16)
    plt.subplots_adjust(hspace=0.3)
    
    # Convert deque objects to lists for plotting
    real_time = list(data["real_time_s"])
    time_stamp = list(data["time_stamp_s"])
    
    # Extract x, y, yaw from pose tuples
    x_positions = [pose[0] for pose in data["pose"]]
    y_positions = [pose[1] for pose in data["pose"]]
    yaw_values = [pose[2] for pose in data["pose"]]
    
    # Extract error components
    x_errors = [err[0] for err in data["error_axis"]]
    y_errors = [err[1] for err in data["error_axis"]]
    yaw_errors = [err[2] for err in data["error_axis"]]

    if target_path is not None:
        # Extract target path points
        target_x = [pose.pose.position.x for pose in target_path.poses]
        target_y = [pose.pose.position.y for pose in target_path.poses]
        axes[0, 0].plot(target_x, target_y, 'r--', label='Global Target Path')
    
    # Plot trajectory (Path Taken By Bot)
    axes[0, 0].plot(x_positions, y_positions, 'b-')
    axes[0, 0].scatter(x_positions[0], y_positions[0], color='green', s=100, marker='o', label='Start')
    axes[0, 0].scatter(x_positions[-1], y_positions[-1], color='red', s=100, marker='x', label='End')
    axes[0, 0].set_xlabel('X Position [m]')
    axes[0, 0].set_ylabel('Y Position [m]')
    axes[0, 0].set_title('Robot Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    
    # Plot position over time
    axes[0, 1].plot(time_stamp, x_positions, 'r-', label='X Position')
    axes[0, 1].plot(time_stamp, y_positions, 'b-', label='Y Position')
    axes[0, 1].set_xlabel('Sim Time [s]')
    axes[0, 1].set_ylabel('Position [m]')
    axes[0, 1].set_title('Bot Position vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True)
    
    # Plot yaw over time
    axes[1, 0].plot(time_stamp, yaw_values, 'g-')
    axes[1, 0].set_xlabel('Sim Time [s]')
    axes[1, 0].set_ylabel('Yaw [rad]')
    axes[1, 0].set_title('Bot Orientation vs Time')
    axes[1, 0].grid(True)
    
    # Plot displacement over time
    axes[1, 1].plot(time_stamp, data["current_displacement"], 'm-')
    axes[1, 1].set_xlabel('Sim Time [s]')
    axes[1, 1].set_ylabel('Displacement [m]')
    axes[1, 1].set_title('Bot Displacement from Origin vs Time')
    axes[1, 1].grid(True)
    
    # Plot position errors over time
    axes[2, 0].plot(time_stamp, np.abs(x_errors), 'r-', label='X Error')
    axes[2, 0].plot(time_stamp, np.abs(y_errors), 'b-', label='Y Error')
    axes[2, 0].plot(time_stamp, np.abs(yaw_errors), 'g-', label='Yaw Error')
    axes[2, 0].set_xlabel('Sim Time [s]')
    axes[2, 0].set_ylabel('Errors')
    axes[2, 0].set_title('Bot Axis Errors vs Time')
    axes[2, 0].legend()
    axes[2, 0].grid(True)
    
    # Plot total error magnitude over time
    error_magnitude = [np.sqrt(err_x**2 + err_y**2) for err_x, err_y in zip(x_errors, y_errors)]
    axes[2, 1].plot(time_stamp, error_magnitude, 'k-')
    axes[2, 1].set_xlabel('Sim Time [s]')
    axes[2, 1].set_ylabel('Error Magnitude [m]')
    axes[2, 1].set_title('Total Bot Displacement Error vs Time')
    axes[2, 1].grid(True)
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])  # Adjust for title
    plt.savefig('navigation_metrics.png')
    plt.show()
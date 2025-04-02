#! /usr/bin/python3
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from threading import Thread
import numpy as np
import math
import matplotlib.pyplot as plt
from collections import deque
import pickle
import datetime

from utils import quaternion_to_euler, euler_to_quaternion  

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

    
def main(args=None):
    try:
        rclpy.init(args=args)
        # navigator node creation
        nav_node = BasicNavigator()
        callback_group = ReentrantCallbackGroup()

        # ================================================================================
        RTF = 0.06 # real time factor to convert sim time to real time
        goal_pose = [-4.0, 12.64, 0.0]
        previous_position = None
        distance = 0.0
        # goal_pose = [1.0, 0.0, 0.0]
        # initializing data dictionary
        data = {
            "real_time_s": deque(maxlen=50000),
            "time_stamp_s": deque(maxlen=50000),
            "pose": deque(maxlen=50000),
            "current_displacement": deque(maxlen=50000),
            "error_axis": deque(maxlen=50000),
            "error": deque(maxlen=50000),
        }
        
        # ================================== setting initial position of the bot ==============================================
        init_pose = PoseStamped()
        init_pose.header.frame_id = "map"
        init_pose.header.stamp = nav_node.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0
        init_pose.pose.position.y = 0.0
        q = init_pose.pose.orientation 
        q.w, q.x, q.y, q.z = euler_to_quaternion(0.0, 0.0, 0.0)
        nav_node.setInitialPose(init_pose)
        # wait until all the essential nodes are started and active as well as the initial pose setup
        nav_node.waitUntilNav2Active()
        # ================================================================================

        # feedback subscriptions
        odom_sub_ = nav_node.create_subscription(Odometry, "odometry/filtered", lambda msg: odometry_callback(msg), qos_profile=10, callback_group=callback_group)
        def odometry_callback(msg):
            nonlocal previous_position, distance
            time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1.0e-9
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            _, _, yaw = quaternion_to_euler(orientation)
            
            # print(f"\n"+"-"*80)
            nav_node.get_logger().info(f"time: {time_stamp} -> position: [{position.x:.2f}, {position.y:.2f}], yaw: {yaw:.2f}")
            nav_node.get_logger().info(f"current distance covered: {distance}")
            # print(f"-"*80+"\n")

            # data collection
            data["real_time_s"].append(time_stamp/RTF)
            data["time_stamp_s"].append(time_stamp)
            data["pose"].append((position.x, position.y, yaw))
            data["current_displacement"].append(np.sqrt(position.x**2+position.y**2))
            data["error_axis"].append((goal_pose[0]-position.x, goal_pose[1]-position.y, goal_pose[2]-yaw))
            if previous_position is not None:
                distance += np.sqrt((position.x-previous_position.x)**2+(position.y-previous_position.y)**2)
            else:
                distance = 0.0
            previous_position = position

        # =================================== goal pose setup ================================================
        goal_pose1 = PoseStamped()
        goal_pose1.header.frame_id = "map" # important for the transformation
        goal_pose1.header.stamp = nav_node.get_clock().now().to_msg()
        goal_pose1.pose.position.x = goal_pose[0]
        goal_pose1.pose.position.y = goal_pose[1]
        goal_pose1.pose.position.z = 0.0
        q = goal_pose1.pose.orientation 
        q.w, q.x, q.y, q.z = euler_to_quaternion(0.0, 0.0, goal_pose[2])
        _, _, goal_yaw = quaternion_to_euler(q)
        # ====================================================================================================

        print(f"*"*40+"STARTING NAV_TO_POSE"+"*"*40+"\n")
        print(f"Goal Pose")
        nav_node.get_logger().info(f"[{goal_pose1.pose.position.x:.2f}, {goal_pose1.pose.position.y:.2f}, {goal_yaw:.2f}]\n")
        print(f"Initial Distance To Goal")
        nav_node.get_logger().info(f"{np.sqrt(goal_pose1.pose.position.x**2+goal_pose1.pose.position.y**2):.2f}\n")
        print(f"*"*80+"\n")
        target_global_path = nav_node.getPath(start=init_pose, goal=goal_pose1, use_start=True)

        nav_start = nav_node.get_clock().now()
        nav_node.goToPose(goal_pose1)
        count = 0
        while not nav_node.isTaskComplete():
            count += 1
            # print(f"{count}")
            feedback = nav_node.getFeedback()
            if feedback:
                now = nav_node.get_clock().now()
                # timeout
                if now-nav_start > rclpy.duration.Duration(seconds=600.0):
                    nav_node.cancelTask()
        
        result = nav_node.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!\n')
            
            print(f"="*40+"DATA"+"="*40)
            print(data)
            print(f"="*80)
            # Save data to file
            # Create a timestamp for the filename
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"navigation_data_{timestamp}.pickle"
            
            # Convert deques to lists for serialization
            serializable_data = {
                "real_time_s": list(data["real_time_s"]),
                "time_stamp_s": list(data["time_stamp_s"]),
                "pose": list(data["pose"]),
                "current_displacement": list(data["current_displacement"]),
                "error_axis": list(data["error_axis"]),
                "error": list(data["error"]),
            }
            
            # Save to file
            with open(filename, 'wb') as f:
                pickle.dump(serializable_data, f)
            
            print(f"\nData saved to {filename}")

            print(f"\n"+"="*40+"SHOWING DATA PLOTS"+"="*40)
            plot_metrics(data=data, target_path=target_global_path)
            print(f"\n"+"="*80)
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
    except KeyboardInterrupt:
        nav_node.get_logger().warn(f"User Interrupted program!")

if __name__ == '__main__':
    main()
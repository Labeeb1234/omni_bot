# A repo for keeping track of path planning tests on a 4WODR

## CAD Model (for simulation purposes hardware design not finalized yet)

- Designed on Autodesk Fusion-360
<div align="center">
  <img src="https://github.com/user-attachments/assets/9665e8e7-e29e-4002-96fb-ba597ee87805" alt="CAD-Model" height="320">
</div>

- NVIDIA IsaacSim View
<div align="center">
  <img src="https://github.com/user-attachments/assets/2b375081-0e96-4112-92a6-ad9d69afaf21" alt="OmniBot IsaacSim">
</div>

## Lower Kinematics Controller (custom)

## simulation integration process
- same as before just like in my previous setups for manipulators and mobile manipulators in Isaac-Sim
- Integration with ROS2,  this step is quite important for testing ROS2 stack on Isaac-Sim
- While interfacing LiDAR from IsaacSim to ROS2 you may encounter an issue with the angle unit conversions. This issue may prevent you from doing SLAM (mainly) sometimes navigation too. The terminal msg indicating this issue is given below.

``` bash
[async_slam_toolbox_node-3] LaserRangeScan contains 720 range readings, expected 14
```

- Issue: when sending the azimuthal range from "Isaac Read LiDAR beam Node" (omni-graph) to "ROS2 LaserScan Node" (omni-graph) a mis-conversion of units occurs as in when the azimuthal range is set as say 360 degrees in the LiDAR sensor prim in IsaacSim, the "Isaac Read LiDAR beam Node" converts it into radians but the "ROS2 LaserScan Node" takes it in as degree so it becomes +- 3.14 degrees instead of += 3.14 radians.
- Fix for this issue is shown below in the image

<div align="center">
  <img src="https://github.com/user-attachments/assets/4e48c5b6-1c84-4c0b-89b2-dd49a036cb00" alt="Fix-1">
</div>

**Note**:  - Issue was encountered in IsaacSim ver: 4.2.0, ROS2-Humble Ubuntu 22.04; not sure if it still persists in IsaacSim ver: 4.5.0; I am not sure if everyone may encounter this issue as I have not seen any one report this issue before; I noticed this during a SLAM test with my environment.
  - Another note the terminal msg show above can come in other ways too, this is one such method to get this msg.
  - While simulating holonomic bot usually the wheel and roller collision (default one) causes the bot have an uneven and jerky motion to smoothen this motion follow these steps: -> First fix the collisions of the wheels and rollers by making sure the collision model is convex decompostion (best as of now to make collisions around oblique shapes like the rollers of a mecanum bot or omni bot) make sure the 


### Updated on simulation process
- Using newer version of [Isaac-Sim](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html) version: 4.5.0 (from 2025)
- The pkg dependency paths as well as the libraries had a full on revamp in the new version, the new version is also directly compatible with IsaacLab as well as the NVIDIA Cosmos for Physical-AI integration, Mobility Gen for get datasets for training model for path planning and also AI based rendering of the environments created in IsaacSim
- Moving on the first few steps to handling asset creation from URDF remains the same --> enable the extensions required the difference being you just have to go the path of your URDF assets (hoping the paths to the meshes (obj/stls) is properly specified in your URDF file) double clicking the file would open up the URDF exported extenstion for import the model to a USD scene.
- Make sure the joint drives are properly selected for each joints, also one thing to note is that the current step does not explictly import fixed joints into the USD from the URDF it just puts the corresponding fixed joints under the parent joint as sub-prims (as show below in the image). The active joints/drivable joints, be it position control or velocity or effort controlled, are explictly created in the USD from the URDF spec and put under joints scope in the USD layers. 

<div align="center">
  <img src="https://github.com/user-attachments/assets/005c8cd4-e563-4a5a-8256-20074be20ac6" alt="new joint specs">
</div>

- The next steps will be fixing collisions and materials for the colliding components. Here is the major difference, once the URDF is imported a directory containing the base asset is made, if 'create collisions from visuals is checked' the a single USD file will be created and this USD scene will have the bot model with three scopes (meshes, collisions and visuals) and one main prim of the asset. For fixing visuals and colllision we have to go to the respective scopes and change the required params, we can't change these params under the main prim (major change from the previous version), just for reference an image below shows a template for what was mentioned.

<div align="center">
  <img src="https://github.com/user-attachments/assets/91c48e4a-8cf8-44ea-8f7d-b5e600ac0da1" alt="new config">
</div>






## Demo Videos/Gifs

- Teleop Test for testing the kinematic and dynamic motion of the system 
<div align="center">
  <img src="https://github.com/user-attachments/assets/c38af580-9c29-49cf-a4d7-2dde2743a6a3" alt="teleop test">
</div>

- ### Nav2 Testing w/ DWB-NAVFN Planner Combination (Tuned for Omni-Motion Model)

  - Bot X-Motion (linear_motion)
    <div align="center">
      <img src="https://github.com/user-attachments/assets/a00cf2be-1857-4491-9415-c1ecd66b9d03" alt="linear_motion">
    </div>

  - Bot Y-Motion (strafe_motion)
    <div align="center">
     <img src="https://github.com/user-attachments/assets/77bf3863-7099-4e29-9ece-14140c710ea8" alt="strafe_motion">
    </div>

  - Bot Holonomic-Motion
    <div align="center">
     <img src="https://github.com/user-attachments/assets/5edfca49-4f8a-4bbd-8fc8-4f7a64a72c06" alt="holonomic_motion">
    </div>


  - To achieve strafing or the holonomic motion in general we have to tune a few nav params under the amcl and controller server section.
  - Under the amcl section this param needs to be added/changed
  ``` yaml
  amcl:
    ros__parameters:
      robot_model_type: "nav2_amcl::OmniMotionModel"
  ```

  - Under the controller server as well as the dwb local planner params these changes need to be made
  ``` yaml
  controller_server:
    ros__parameters:
      min_x_velocity_threshold: 0.001 
      min_y_velocity_threshold: 0.001
      min_theta_velocity_threshold: 0.001
  
      # add the other default params too don't copy paste this as it is 
  
      FollowPath:
        plugin: "dwb_core::DWBLocalPlanner"
        angular_dist_threshold: 0.785
        forward_sampling_distance: 0.5
        rotate_to_heading_angular_vel: 5.0
        max_angular_accel: 3.2
        simulate_ahead_time: 1.0
        rotate_to_goal_heading: false
        
        debug_trajectory_details: True
        min_vel_x: -0.5
        min_vel_y: -0.5
        max_vel_x: 0.5
        max_vel_y: 0.5
        max_vel_theta: 0.35
        min_speed_xy: -0.5
        max_speed_xy: 0.5
        min_speed_theta: -0.35
  
        acc_lim_x: 2.5
        acc_lim_y: 2.5
        acc_lim_theta: 3.2
        decel_lim_x: -2.5
        decel_lim_y: -2.5
        decel_lim_theta: -3.2
  
        vx_samples: 20
        vy_samples: 20
        vtheta_samples: 20
  
        sim_time: 1.7
        linear_granularity: 0.05
        angular_granularity: 0.025
        transform_tolerance: 0.2
        xy_goal_tolerance: 0.22
        trans_stopped_velocity: 0.25
        short_circuit_trajectory_evaluation: True
        stateful: True
  
        critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
        
        BaseObstacle.scale: 0.02
  
        PathAlign.scale: 32.0
        PathAlign.forward_point_distance: 0.1
  
        GoalAlign.scale: 24.0
        GoalAlign.forward_point_distance: 0.1
  
        PathDist.scale: 32.0
        GoalDist.scale: 24.0
  
        RotateToGoal.scale: 32.0
        RotateToGoal.slowing_factor: 5.0
        RotateToGoal.lookahead_time: -1.0
  
  ```
  
    
  - Note if you are also using velocity smoothers those params also require a few changes to get holonomic motion, given below are the changes params for the same
  ``` yaml
  velocity_smoother:
    ros__parameters:
      smoothing_frequency: 20.0
      scale_velocities: false
      feedback: "OPEN_LOOP"
      max_velocity: [0.5, 0.5, 0.35]
      min_velocity: [-0.5, -0.5, -0.35]
      deadband_velocity: [0.0, 0.0, 0.0]
      velocity_timeout: 1.0
      max_accel: [2.5, 2.5, 3.2]
      max_decel: [-2.5, -2.5, -3.2]
      odom_topic: "odom"
      odom_duration: 0.1
      use_realtime_priority: false
      enable_stamped_cmd_vel: false
  ```

- ### NAV2 Testing w/ DWB-SMAC Combination





## An RL developer module specifically for 4wheeled-OmniBot using python (just for fun 😐, got sidetracked)  

- Here is a working demo of a sample multi-bot environment 

<div align="center">
  <img src="https://github.com/user-attachments/assets/a90cd675-712b-48ba-964b-8ab58b674637" alt="multi-bot-feature-test-demo">
</div>

- The module is still subject to changes and is unpolished 


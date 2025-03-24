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
  <img src="https://github.com/user-attachments/assets/f90847ee-fc3f-46b3-a15c-5dcf6e306469" alt="Fix-1">
</div>

- **Note**:  - Issue was encountered in IsaacSim ver: 4.2.0, ROS2-Humble Ubuntu 22.04; not sure if it still persists in IsaacSim ver: 4.5.0; I am not sure if everyone may encounter this issue as I have not seen any one report this issue before; I noticeed this during a SLAM test with my environment.
  - Another note the terminal msg show above can come in other ways too, this is one such method to get this msg.  


## Demo Videos/Gifs

- Teleop Test for testing the kinematic and dynamic motion of the system 
<div align="center">
  <img src="https://github.com/user-attachments/assets/c38af580-9c29-49cf-a4d7-2dde2743a6a3" alt="teleop test">
</div>

## Nav2 Testing w/ DWB-NAVFN Planner Combination

- Bot X-Motion (linear_motion)
<div align="center">
  <img src="https://github.com/user-attachments/assets/a00cf2be-1857-4491-9415-c1ecd66b9d03" alt="linear_motion">
</div>





## An RL developer module specifically for 4wheeled-OmniBot using python (just for fun 😐, got sidetracked)  

- Here is working demo of a sample multi-bot environment 

<div align="center">
  <img src="https://github.com/user-attachments/assets/a90cd675-712b-48ba-964b-8ab58b674637" alt="multi-bot-feature-test-demo">
</div>

- The module is still subject to changes and is unpolished 


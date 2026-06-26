# Opening custom ROS2 Gazebo world
## Purpose
The intention for this document is outlining the steps required to open a simulated world to run navigation and stability simulations with a virtual twin of the robot.  
## Procedure[^1]
Necessary files/packages:
>contested_env.world
>model.sdf
>ROS2

1. Open 'contested_env.world'. Navigate to the following directory:
>~/circe_ws/src/CIRCE_Desc/worlds

And run the following command:
> gz sim contested_env.world

2. In a second terminal window running in parallel, run the following command:
> ros2 run ros_gz_sim create -world contested_env -file [ABSOLUTE_PATH_TO_ROBOT_SDF_FILE] -x 0.0 -y 0.0 -z 0.3

As of 6/26/26 (Hey thats a palindrome date), this will create a black box in an environment with two pale red walls. 

[^1]: This procedure assumes you have ROS2, Gazebo, and all necessary packages and files correctly installed.  

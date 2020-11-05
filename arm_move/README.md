# Differential Drive Package
### Author: Arun Kumar

#### This package is used to control the px100 arm using either a fake node or the physical hardware. The package provides services to set and store arm waypoints. There is also a service to execute all waypoints in order. This package can be used for arm trajectory planning.

![](videos/flip.gif)

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile: `catkin_make`
1. Start simulation: `roslaunch arm_move arm.launch`
1. Use reset service to clear waypoints from parameter server and return to home position
1. Use step service to test and add new waypoints
1. Use follow service to follow all waypoints in parameter server
1. Run command `rosparam dump /arm_move/config/waypoints.yaml /px100/mover` to save collected waypoints to waypoints.yaml

#### Configuration Instructions:
1. Configure waypoints in config/waypoints.yaml
1. config/video_waypoints.yaml contain the waypoints used to generate the gif
1. arm.launch configuration
    * The arm launch file contains arguements robot_name and use_hardware
    * modify robot_name to use a different robot
    * set use_hardware to false to use a fake node instead of the physical hardware
    * Set arguements true/false via the command line or the launch file
    * The following command launches with using a fake node:
```
roslaunch arm_move arm.launch use_hardware:=false
```
#### Testing Instructions:
1 run `catkin_make run_tests` in home directory of workspace to test the step function
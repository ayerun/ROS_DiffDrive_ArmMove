# Differential Drive Package
### Author: Arun Kumar

#### This package contains xacro files for a differential drive robot that can be used in Gazebo and rviz. Nodes in this package are used to make the robot perform different tasks in Gazebo. One node makes the robot flip. The other node makes the robot follow a rectangular path. Robot dimensions can be varied.

![](videos/flip.gif)

#### Usage Instructions:
1. Add package to the src folder in your ROS workspace
1. Compile: `catkin_make`
1. Start simulation: `roslaunch diff_drive [launchfilename]`
1. ddrive.launch is used to launch the robot in gazebo and make the flip or follow a rectangular trajectory
1. ddrive_rviz.launch is a debugging launch file used to vizualize the urdf file in rviz

#### Configuration Instructions:
1. Configure robot base dimensions in /config/dims.yaml
1. Configure rectangle dimensions in /config/rect_dims.yaml
1. ddrive.launch configuration
    * The ddrive launch file contains arguements (gaz, rect, flip) for three optional launch nodes
    * gaz launches Gazebo
    * flip launches a flip node
    * rect launches a follow_rect node
    * When these arguments are true the corresponding nodes are launched
    * By default gaz and flip are true
    * Do not simulataneous set flip and rect to true
    * Gazebo is launched in a paused state, but the flip and follow_rect nodes contain code that unpauses Gazebo
    * Set arguements true/false via the command line or the launch file
1. ddrive_rviz.launch configuration
    * The ddrive_rviz launch file one arguements (gui) for one optional launch node
    * gui launches the joint state publisher gui
    * When these arguments are true the corresponding nodes are launched
    * By default gui is true
    * Set arguements true/false via the command line or the launch file
    * The following command launches without joint state publisher gui:
```
roslaunch diff_drive ddrive_rviz.launch gui:=false
```

#### Videos:

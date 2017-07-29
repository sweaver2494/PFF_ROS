# ROS_URDF

## About
This repository contains the following:
* a simple urdf differential drive model
* a custom robot state publisher node that subscribes to the joint_states topic and publishes to the tf topic. This node broadcasts tf transformations from the map to the robot's central axle.
* a launch file to launch the following:
  * joint state publisher GUI
  * robot state publisher (to publish transformations from the central axle to the wheels)
  * custom robot state publisher (to publish transformations from the map to the robot's central axle)
  * rvis to visualize the robot's motion

## Run Instructions
From this README's directory, type:
```
source /opt/ros/kinetic/setup.bash
roscore& 

catkin_make install

source install/setup.bash
roslaunch pff pff.launch
```

Once rvis launches, add the Robot Model, and then use the joint state publisher GUI to control the rotation angles of the robot's wheels.

## Dependencies
* tf
* urdf
* rvis
* xacro

# ROS_URDF

This repository contains a simple usrd differential drive model, a robot state publisher node that broadcasts tf transformations from the map to the robot, and a launch file to launch the following:
* joint state publisher
* robot state publisher (for the wheel joints)
* custom robot state publisher (transform the map to the robot)
* rvis

 
From this directory, type:

source /opt/ros/kinetic/setup.bash
roscore& 
catkin_make install

source install/setup.bash
roslaunch pff pff.launch


Once rvis launches, add the Robot Model, and then use the joint state publisher GUI to control the robot's wheels.



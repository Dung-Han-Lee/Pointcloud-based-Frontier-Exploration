## Overview
This repo contains c++ codes for pointcloud-base frontier generation.  
This code was meant for frontier based exploration using LIDAR sensor.  
The purple points represent the acculumated points i.e. world model.The  
 rainbow points represents clustered center of frontier points; the more  
 violet the more information that cluster represents. In addition, a na-  
ive sensor model has been applied such that frontier points within raidus  
R of the robot would be suppressed.

Topics subsribed to: `velodyne_cloud_registered`, `aft_mapped_to_init`(robot position).  
Topics published to: `frontier`, `world_model`  
[!Example](https://media.giphy.com/media/JsEMHz8UokICGwypCi/giphy.gif)

## Build and Run
    catkin_make
    rosbag play <bagfile>
    rosrun exploration exploration_node

## Limitations
Frontier points could appear in previously visited area due to geometry  
of the environment e.g. part of the wall is not visible from one side of  
the corridor.


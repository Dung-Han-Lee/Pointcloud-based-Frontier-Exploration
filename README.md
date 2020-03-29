##Overview
This repository contains codes that subsribes to LIDAR pointclouds, from them build a world model, detect frontier points and clustered frointier points. Topics that are subsribed: `velodyne_cloud_registered`, `aft_mapped_to_init`(robot position). Example [video](https://www.youtube.com/watch?v=AmljFj6Loq0&fbclid=IwAR0LnJ0lbV-ZV7EVbq4Vx_MjzY3_magAM62Dfc9GdNHgn5H4PYt2AIs06ps)

##Build and Run
  catkin_make
  (run bag file)
  rosrun exploration exploration_node

## File Structures
  planner.cpp -- main file
    frontier_detector.cpp -- detector/cluster
       commond_header -- PCL/ROS headers



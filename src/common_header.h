#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
#include <vector>

typedef pcl::PointXYZI Ptype;
typedef pcl::PointCloud<Ptype> PC;
typedef sensor_msgs::PointCloud2 RosPC;
typedef geometry_msgs::PoseArray PoseArray; 
typedef geometry_msgs::Point RosPoint;
typedef nav_msgs::Odometry Odom;
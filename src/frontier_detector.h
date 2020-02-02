/*  This class would return a downsampled pointcloud, with intensity equals 
    zero for all frontier points  */

#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <sensor_msgs>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>

typedef pcl::PointXYZI ptype;
typedef pcl::PointCloud<ptype> pc;

class FrontierDetector{
  public:
    FrontierDetector() = default;

    FrontierDetector(int vdim): vdim_(vdim) {};
    
    ~FrontierDetector() = default;
    
    pc::ptr Detect(const sensor_msgs::PointCloud2::ConstPtr& msg, pc::Ptr old);
      // inputs: current LIDAR pointcloud, previous LIDAR pointcloud
      // return: merged-downsampled pointcloud with non frontier points having
      //         non-zero intensity values
  
  private:
    int vdim_; // voxel dimensions for downsampling
};
#endif
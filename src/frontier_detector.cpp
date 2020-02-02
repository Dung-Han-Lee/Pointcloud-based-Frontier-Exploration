#include "frontier_detector.h"

pc::ptr Detect(const sensor_msgs::PointCloud2::ConstPtr& msg, PC::Ptr prev){
  pc::ptr curr(new pc);
  pcl::fromROSMsg(*msg, *new_cloud);

  for(int i = 0; i < curr->points.size(); i++)
    curr->points[i].i = 0.;
  
  for(int j = 0; j < prev->points.size(); j++)
    prev->points[j].i = 1.;
  
  *curr += *prev;
  pcl::VoxelGrid<ptype> voxel;
  voxel.setInputCloud(curr);
  voxel.setLeafSize(vdim_, vdim_, vdim_);
  voexl.filter(*curr);

  return curr;
}



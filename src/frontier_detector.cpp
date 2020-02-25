#include "frontier_detector.h"
#include <pcl/filters/statistical_outlier_removal.h>

// https://stackoverflow.com/questions/49819248/which-pcl-filter-to-use-to-downsample-a-point-cloud
// TODO: apply range filter to get rid off ceiling

PC::Ptr FrontierDetector::Detect(const RosPC::ConstPtr& msg, PC::Ptr world_model){

  auto Downsample = [](PC::Ptr cloud, float leafsz){
    PC::Ptr tmp (new PC);
    pcl::VoxelGrid<Ptype> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafsz, leafsz, leafsz);
    voxel.filter(*tmp);
    *cloud = *tmp;
  };

  auto SetIntensity = [](PC::Ptr cloud, float val){
    auto it = cloud->points.begin();
    for(; it != cloud->points.end(); it++)
      it->intensity = val;
  };

  // Get nput pointcloud 
  PC::Ptr curr = PC::Ptr(new PC);
  pcl::fromROSMsg(*msg, *curr);

  // Prepare for frontier detection
  SetIntensity(curr, 0.);
  SetIntensity(world_model, 1.);

  // Identify all downsample-points of value 1. as frontier  
  *world_model += *curr;
  Downsample(world_model, 0.2f);
  curr->clear();
  for(const auto& pt : world_model->points){
    if(pt.intensity == 0.) curr->points.emplace_back(pt);
  }

  SetIntensity(world_model, 1.);
  if(verbose_){
    std::cout<<" world model sz is "<<world_model->points.size();
    std::cout<<" # of frotiner pt: "<<curr->points.size()<<"\n";
  } 

  return curr;
}



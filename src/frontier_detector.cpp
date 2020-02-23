#include "frontier_detector.h"
#include <algorithm>

PC::Ptr FrontierDetector::Detect(const RosPC::ConstPtr& msg, PC::Ptr world_model){

  auto Downsample = [](PC::Ptr cloud, float leafsz){
    pcl::VoxelGrid<Ptype> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(leafsz, leafsz, leafsz);
    voxel.filter(*cloud);
  };

  auto SetIntensity = [](PC::Ptr cloud, float val){
    auto it = cloud->points.begin();
    for(; it != cloud->points.end(); it++)
      it->intensity = val;
  };

  // Downsmaple input pointcloud 
  PC::Ptr curr = PC::Ptr(new PC);
  pcl::fromROSMsg(*msg, *curr);
  Downsample(curr, 0.1f);

  // Prepare for frontier detection
  SetIntensity(curr, 0.);
  SetIntensity(world_model, 1.);
  *world_model += *curr;

  // Identify all downsample-points of value 1. as frontier  
  PC::Ptr merge = PC::Ptr(new PC);
  *merge = *world_model;
  Downsample(merge, 0.1f);
  curr->clear();
  for(auto pt : merge->points){
    if(pt.intensity == 0.) curr->points.emplace_back(pt);
  }

  SetIntensity(world_model, 1.);

  if(verbose_){
    std::cout<<" world model sz is "<<world_model->points.size();
    std::cout<<" # of frotiner pt: "<<curr->points.size()<<"\n";
  } 

  return curr;
}



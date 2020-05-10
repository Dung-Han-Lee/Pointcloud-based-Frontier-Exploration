#include "frontier_detector.h"

const double RAIDUS = 10.;

void FrontierDetector::Downsample(PC::Ptr cloud, float leafsz){
  PC::Ptr merged (new PC);
  pcl::VoxelGrid<Ptype> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(leafsz, leafsz, leafsz);
  voxel.filter(*merged);
  *cloud = *merged;
}

void FrontierDetector::SetIntensity(PC::Ptr cloud, float val){
  for(auto& pt : cloud->points) pt.intensity = val;
}

void FrontierDetector::Detect(const RosPC::ConstPtr& msg,\
    PC::Ptr world_model, PC::Ptr frontier){

  // Get input pointcloud 
  PC::Ptr curr = PC::Ptr(new PC);
  pcl::fromROSMsg(*msg, *curr);

  // Prepare for frontier detection
  SetIntensity(curr, 0.);
  SetIntensity(world_model, 1.);
  
  // Identify all downsample-points of value 0. as new frontier
  *world_model += *curr;
  Downsample(world_model, 0.2f);
  for(auto& pt : world_model->points){
    if(pt.intensity == 0.){
      frontier->points.emplace_back(pt);
    }
  }
  SetIntensity(world_model, 1.);

  if(verbose_){
    std::cout<<" world model sz is "<<world_model->points.size();
    std::cout<<" # of frotiner pt: "<<frontier->points.size()<<"\n";
  } 
}

void FrontierDetector::ApplySensorModel(const RosPoint& robot_pos, PC::Ptr frontier){
  
  auto Distance = [](const RosPoint& a, const Ptype& b){
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  };

  // Remove all raw frontiers within radius R of robot poistion
  PC::Ptr preserved = PC::Ptr(new PC);
  for(const auto& p : frontier->points){
    if(Distance(robot_pos, p) > RAIDUS) preserved->points.emplace_back(p);
  }
  *frontier = *preserved;
}

void FrontierDetector::ClusterPoints(PC::Ptr frontier){

  auto InBox = [](const Ptype& a, const Ptype& b, double dim){
    return std::abs(a.x - b.x) < dim && std::abs(a.y - b.y) < dim && std::abs(a.z - b.z) < dim;
  };

  // Downsample raw frontier points to make cluster
  double dim = 10.f; 
  PC::Ptr cluster = PC::Ptr(new PC);
  *cluster = *frontier;
  SetIntensity(cluster, 1.);
  Downsample(cluster, dim);

  // Set intensity of cluster to number of points inside the voxel
  int size = cluster->points.size();
  std::vector<int> cnt(size, 0);
  for(const auto& pt: frontier->points){
    for(int i = 0; i < size; i++)
      if(InBox(pt, cluster->points[i], dim/2)) cnt[i]++; 
  }
  for(int i =0; i < size; i++)  
    cluster->points[i].intensity = cnt[i];
 
  *frontier = *cluster;
}

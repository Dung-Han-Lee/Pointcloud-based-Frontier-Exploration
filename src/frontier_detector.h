/*  This class would return a downsampled pointcloud, with intensity equals 
    zero for all frontier points  */

#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <vector>
#include "common_header.h"



class FrontierDetector{
  public:
    FrontierDetector() = default;
    
    ~FrontierDetector() = default;
    
    PC::Ptr Detect(const RosPC::ConstPtr& msg, PC::Ptr world_model);
      // inputs: current LIDAR pointcloud, previous LIDAR pointcloud
      // return: frontier points
  
  private:
    float ds_input_ = 0.5f;  // downsampling for input
    float ds_frontier = 0.1f; 
    bool verbose_ = true;
};
#endif
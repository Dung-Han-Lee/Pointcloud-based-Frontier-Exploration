/*  This class would return a downsampled pointcloud, with intensity equals 
    zero for all frontier points  */

#ifndef FRONTIER_DETECTOR_H
#define FRONTIER_DETECTOR_H

#include <vector>
#include <algorithm>
#include "common_header.h"



class FrontierDetector{
  public:
    FrontierDetector() = default;
    
    ~FrontierDetector() = default;
    
    void Detect(const RosPC::ConstPtr& msg,\
        PC::Ptr world_model, PC::Ptr frontier);

    void ApplySensorModel(const RosPoint& robot_pos, PC::Ptr frontier);

    void Downsample(PC::Ptr cloud, float leafsz);

    void SetIntensity(PC::Ptr cloud, float val);

    void ClusterPoints(PC::Ptr frontier);

  private:
    bool verbose_ = true;
};
#endif
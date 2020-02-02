#ifndef CANDIDATE_SCORER_H
#define CANDIDATE_SCORER_H

#include <vector>
#include <math.h>  
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZI ptype;
typedef pcl::PointCloud<ptype> pc;
constexpr float kLidarRange = 100.; 

class CandidateScorer{
  public:
    CandidateScorer(float alp): alp_(alp) {};
    ~CandidateScorer() = default;
    std::vector<float> Score(std::vector<vector<float> >& candidates, 
                              pc::ptr pc,
                              std::vector<float> curr_pos);
      // Score candidate points according to #frontiers and travel dist 
  private:
    float alp_; // trade off parameter
};

#endif
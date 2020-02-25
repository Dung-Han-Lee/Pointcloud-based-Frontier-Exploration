#ifndef CANDIDATE_SCORER_H
#define CANDIDATE_SCORER_H

#include <math.h>
#include <utility>
#include <algorithm>
#include "common_header.h"

constexpr double kTimeValue = 0.1; // sec per point
constexpr double kSpeed = 1.3; // meter per sec
constexpr double kRadius  = 10.0; // radius for frontier to be attributed to propsed point

class CandidateScorer{
  public:
    CandidateScorer()  = default;

    ~CandidateScorer() = default;

    std::vector<std::pair<float, RosPoint>> Score( 
        const PoseArray& proposals, const RosPoint& cur, const PC::Ptr frontiers);
      // Inputs: convex hull centroids, current robot position, fronteirs
      // evaluate candidates accoridng to travel distance and # of frontiers
      // results are saved as a sorted RosPoint* vectors internally
    
    bool verbose_ = 0;
};

#endif
#ifndef CANDIDATE_SCORER_H
#define CANDIDATE_SCORER_H

#include <vector>
#include <math.h>
#include "common_header.h"

constexpr float kLidarRange = 100.; 

class CandidateScorer{
  public:
    CandidateScorer(float k): kfrontier_(k) {};
    ~CandidateScorer() = default;
    std::vector<float> Score(std::vector<std::vector<float> >& candidates, 
                              PC::Ptr PC,
                              std::vector<float> curr_pos);
      // Score candidate points according to #frontiers and travel dist 
  private:
    float kfrontier_; // trade off parameter
};

#endif
#include "candidate_scorer.h"

std::vector<float> CandidateScorer::Score(
    std::vector<vector<float> >& candidates, 
    pc::ptr pc,
    std::vector<float> curr_pos)
{
  const float sz = static_cast<float>(pc->points.size());
  const int n = candidates.size();
  std::vector<float> scores = {};

  // Evaluate euclidean dist between the curr_pos and a candiate point
  auto dist = [](std::vector<float> c, std::vector<float> curr_pos){
    float dx = c[0] - curr_pos[0];
    float dy = c[1] - curr_pos[1];
    float dz = c[2] - curr_pos[2];
    float dist = std::hypot(std::hypot(dx, dy), dz);
    return dist/kLidarRange;
  };

  // Assign score to candidate: #frontier/#pts - cost
  float num_frontier, cost;
  for(const auto& c : candidates){
    cost = dist(c, curr_pos);
    num_frontier = 0;
    for(auto it = pc->points.begin(); it!=pc->points.end(); it++)
      if(it->i == 0.) num_frontier += 1.;
    scores.push_back(alp_ * num_frontier/sz - (1-alp_) * cost);
  }
  return scores;
}

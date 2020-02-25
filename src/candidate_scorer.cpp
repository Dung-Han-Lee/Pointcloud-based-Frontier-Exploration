#include "candidate_scorer.h"
typedef std::pair<float, RosPoint> Pair;

std::vector<Pair> CandidateScorer::Score(
   const PoseArray& proposals, const RosPoint& cur, const PC::Ptr frontiers)
{
  // Evaluate euclidean dist between pose
  auto Dist = [](const RosPoint& a, const RosPoint& b){
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  };

  auto Inlier = [](const RosPoint& a, const Ptype& b){
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z) < kRadius;
  };

  auto Value = [](int nfrontier, double dist){
    return nfrontier * kTimeValue - dist/kSpeed;
  };

  // Assign score to each proposal point
  std::vector<Pair> res;
  auto it = proposals.poses.begin();
  int cnt = 0;
  for(; it != proposals.poses.end(); it++, cnt = 0){
    auto& pos = it->position;
    for(const auto& pt : frontiers->points)
      if(Inlier(pos, pt)) cnt++;
    res.emplace_back( Value(cnt, Dist(pos, cur)), pos);
    std::cout<<"cnt = "<<cnt;
  }

  // Sort by score
  std::sort(res.begin(), res.end(), 
      [](const Pair& a, const Pair& b){return a.first < b.first;});
  
  if(verbose_){
    auto p = res.front().second;
    //std::cout<<"robot pos is x = "<<cur.x<<" y = "<<cur.y<<" z = "<<cur.z<<" ";
    std::cout<<"Top proposal is x = "<<p.x<<" y = "<<p.y<<" z = "<<p.z<<"\n";
  }

  return res;
}



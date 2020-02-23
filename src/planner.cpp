#include <ros/ros.h>
#include "candidate_scorer.h"
#include "frontier_detector.h"

class Master{
  public:
    Master(ros::NodeHandle& nh);
    ~Master();
    int Loop();

  private:
    PC::Ptr curr_;
    PC::Ptr world_;
    ros::Subscriber lidar_sub_, pos_sub_;
    ros::Publisher frontier_pub_, model_pub_;
    CandidateScorer* scorer_;
    FrontierDetector* detector_;
    

    void lidarCb(const RosPC::ConstPtr& msg);
    void Plan(std::vector<std::vector<float> >& candidates, 
              std::vector<float> curr_pos);
};

Master::~Master(){
  delete scorer_;
  delete detector_;
}

Master::Master(ros::NodeHandle& nh){
  // Instantiate member objects
  float ratio_frontier = 0.5;
  scorer_ = new CandidateScorer(ratio_frontier);
  detector_ = new FrontierDetector();

  // Create empty cloud
  curr_ = PC::Ptr (new PC);
  world_ = PC::Ptr (new PC);

  lidar_sub_ = nh.subscribe("velodyne_cloud_registered", 10, &Master::lidarCb, this);
  //pos_sub_   = nh.subscribe("topic", 10, &Master::posCb, this);
  frontier_pub_ = nh.advertise<RosPC>("frontier", 10);
  model_pub_ = nh.advertise<RosPC>("world_model", 10);
}

int Master::Loop(){
  while(ros::ok())
  {
    ros::Rate loop_rate(1);
    ros::spinOnce();
    loop_rate.sleep();
    // TODO
    // Get candidates
    // Evaluate candidates
    // Plan search path
  }    
}


void Master::lidarCb(const RosPC::ConstPtr& imsg){
  curr_ = detector_->Detect(imsg, world_);
  RosPC omsg1, omsg2;
  pcl::toROSMsg(*curr_ , omsg1);
  pcl::toROSMsg(*world_, omsg2);
  frontier_pub_.publish(omsg1);
  omsg2.header.frame_id = omsg1.header.frame_id;
  model_pub_.publish(omsg2);
}

/* 
void Master::Plan(std::vector<std::vector<float>>& candidates, 
                  std::vector<float> curr_pos){
  std::vector<float> scores = scorer_(candidates, curr_, curr_pos);
}*/

int main(int argc, char** argv){
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh;
  Master master(nh);
  master.Loop();
}
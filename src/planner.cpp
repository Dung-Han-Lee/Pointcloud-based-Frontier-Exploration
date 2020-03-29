#include <ros/ros.h>
#include "candidate_scorer.h"
#include "frontier_detector.h"
#include <geometry_msgs/PoseStamped.h>

PoseArray FakeInput(){
  PoseArray pv;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 1.;
  pose.pose.position.y = 1.;
  pose.pose.position.z = 1.;
  pv.poses.push_back(pose.pose);
  return pv;
}

class Master{
  public:
    Master(ros::NodeHandle& nh);
    ~Master();
    int Loop();

  private:
    RosPoint robot_pos_;
    PC::Ptr frontier_;
    PC::Ptr world_;
    ros::Subscriber lidar_sub_, pos_sub_;
    ros::Publisher frontier_pub_, model_pub_;
    CandidateScorer* scorer_;
    FrontierDetector* detector_;
    
    void LidarCb(const RosPC::ConstPtr& msg);
    void PosCb(const Odom& odom);
    void Plan(std::vector<std::vector<float> >& candidates, 
              std::vector<float> curr_pos);
};

Master::~Master(){
  delete scorer_;
  delete detector_;
}

Master::Master(ros::NodeHandle& nh){
  // Instantiate member objects
  scorer_ = new CandidateScorer();
  detector_ = new FrontierDetector();

  // Create empty cloud
  frontier_ = PC::Ptr (new PC);
  world_ = PC::Ptr (new PC);

  // Initialize subcribers and publishers
  lidar_sub_ = nh.subscribe("velodyne_cloud_registered", 10, &Master::LidarCb, this);
  pos_sub_ = nh.subscribe("aft_mapped_to_init", 1, &Master::PosCb, this);
  frontier_pub_ = nh.advertise<RosPC>("frontier", 10);
  model_pub_ = nh.advertise<RosPC>("world_model", 10);
}

int Master::Loop(){
  while(ros::ok())
  {
    ros::Rate loop_rate(10);
    ros::spinOnce();
    loop_rate.sleep();
  }    
}


void Master::LidarCb(const RosPC::ConstPtr& imsg){
  
  // Get accumlated raw-frontier points
  detector_->Detect(imsg, world_, frontier_);
  detector_->ApplySensorModel(robot_pos_, frontier_);
  detector_->ClusterPoints(frontier_);

  // Convert pcl pointcloud to sensor_msg pointcloud2
  RosPC omsg1, omsg2;
  pcl::toROSMsg(*frontier_ , omsg1);
  pcl::toROSMsg(*world_, omsg2);

  // Publish to sensor frame 
  omsg1.header = imsg->header;
  omsg2.header = imsg->header;
  frontier_pub_.publish(omsg1);
  model_pub_.publish(omsg2);
  
  //PoseArray pv = FakeInput();
  //auto points = scorer_->Score(pv, robot_pos_, frontier_);
}

void Master::PosCb(const Odom& odom){
  robot_pos_ = odom.pose.pose.position;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh;
  Master master(nh);
  master.Loop();
}
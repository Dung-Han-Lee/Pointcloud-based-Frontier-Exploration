#include <ros/ros.h>
#include "candidate_scorer.h"
#include "frontier_detector.h"

class Master{
  public:
    Master(ros::NodeHandle& node_handle);
    ~Master() = default;
    int Loop();
  private:
    ros::Subscribe lidar_sub_, pos_sub_;
    ros::Publisher lidar_pub_;
    CandidateScorer _cs;
    FrontierDetector _fd;
    pc::ptr cloud_(new pc);
    pc::prt prev_(new pc);

    void lidarCb(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void Plan(std::vector<vector<float> >& candidates, 
              std::vector<float> curr_pos);
};

Master::Master(ros::NodeHandle& node_handle){
  lidar_sub_ = nh.subscribe("topic", 10, &Mater::lidarCb, this);
  pos_sub_   = nh.subscribe("topic", 10, &Master::posCb, this);
  lidar_pub_ = nh.advertise<pc>("frontier", 10);
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

void Master::lidarCb(const sensor_msgs::PointCloud2::ConstPtr& msg){
  cloud_ = _fd.Detect(msg, cloud_);
}

void Master::Plan(std::vector<vector<float> >& candidates, 
                  std::vector<float> curr_pos){
  std::vector<float> scores = _cs(candidates, cloud_, curr_pos);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  Master _master(nh);
  _master.Loop();
}
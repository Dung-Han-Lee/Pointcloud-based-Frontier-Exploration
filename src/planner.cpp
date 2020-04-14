#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class Master{
  public:
    Master(ros::NodeHandle& nh);

    ~Master() = default;
    
    void VizPolyhedron();

    int Loop();

  private:

    ros::Publisher convex_pub;
    
};

Master::Master(ros::NodeHandle& nh){
  convex_pub = nh.advertise<visualization_msgs::Marker>("convex hull", 10);
}

int Master::Loop(){
  while(ros::ok())
  {
    ros::Rate loop_rate(10);
    this->VizPolyhedron();
    ros::spinOnce();
    loop_rate.sleep();
  }    
}

void Master::VizPolyhedron()
{
  visualization_msgs::Marker scan_marker;
  scan_marker.header.frame_id = "/robot_0/base_laser_link";
  scan_marker.header.stamp = ros::Time::now();
  scan_marker.ns = "scan";
  scan_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  scan_marker.action = visualization_msgs::Marker::ADD;
  scan_marker.id = 0;
  scan_marker.scale.x = scan_marker.scale.y = scan_marker.scale.z = 1.0;
  std_msgs::ColorRGBA scan_color;
  scan_marker.pose.position.x = 0.0;
  scan_marker.pose.position.y = 0.0;
  scan_marker.pose.position.z = 0.0;
  scan_marker.pose.orientation.x = 0.0;
  scan_marker.pose.orientation.y = 0.0;
  scan_marker.pose.orientation.z = 0.0;
  scan_marker.pose.orientation.w = 1.0;
  scan_color.a = 0.8f;
  scan_color.r = 0.25f;
  scan_color.g = 0.5f;
  scan_color.b = 0.9f;
  scan_marker.color = scan_color;
  geometry_msgs::Point temp;

  std::vector<std::vector<double>> points = {{0,0,0}, {11.,0,0}, {0,10,0}};
  for (int i = 0; i < points.size(); i++) {
    temp.x = points[i][0];
    temp.y = points[i][1];
    temp.z = points[i][2];
    scan_marker.points.push_back(temp);
  }
  this->convex_pub.publish(scan_marker);
}


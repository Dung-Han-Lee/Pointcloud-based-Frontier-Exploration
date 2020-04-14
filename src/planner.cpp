#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "convexhull.h"

class Master{
  public:
    Master(ros::NodeHandle& nh);

    ~Master() = default;
    
    void VizPolyhedron(const std::vector<Face>& triangles) const;

    int Loop();

  private:

    ros::Publisher convex_pub;
    
};

Master::Master(ros::NodeHandle& nh){
  convex_pub = nh.advertise<visualization_msgs::Marker>("convex_hull", 10);
}

int Master::Loop(){
  while(ros::ok())
  {
    std::vector<std::vector<int>> tests = {{0,0,0}, {0,3,0}, {3,0,0}, {0,0,3}};
    std::vector<Vec3> vecs;
    std::vector<Face> faces;
    for(auto test : tests) vecs.emplace_back(test[0], test[1], test[2]);
    ConvexHull c(vecs);
    ros::Rate loop_rate(10);
    this->VizPolyhedron(c.GetFaces());
    ros::spinOnce();
    loop_rate.sleep();
  }    
}

void Master::VizPolyhedron(const std::vector<Face>& triangles) const
{
  visualization_msgs::Marker scan_marker;
  scan_marker.header.frame_id = "map";
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
  scan_color.a = 0.5f;
  scan_color.r = 0.25f;
  scan_color.g = 0.9f;
  scan_color.b = 0.9f;
  scan_marker.color = scan_color;
  geometry_msgs::Point temp;

  auto viz_one_point = [&](const Vec3& p)
  {
    temp.x = p.x; temp.y = p.y; temp.z = p.z;
    scan_marker.points.push_back(temp);
  };

  for (const auto& triangle : triangles) 
  {
    viz_one_point(triangle.a);
    viz_one_point(triangle.b);
    viz_one_point(triangle.c);
  }

  this->convex_pub.publish(scan_marker);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "exploration_node");
  ros::NodeHandle nh;
  Master master(nh);
  master.Loop();
}
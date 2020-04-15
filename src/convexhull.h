#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include "utility.h"


// Defined in CCW
struct Face
{
  Face(const Point3D& p1, const Point3D& p2, const Point3D& p3): visible(false)
      { vertices[0] = p1; vertices[1] = p2; vertices[2] = p3;};

  void Reverse(){std::swap(vertices[0], vertices[1]);};

  friend std::ostream& operator<<(std::ostream& os, const Face& f)
  {
    os << "[face pt1 = "   << f.vertices[0].ToString()
       << " | face pt2 = " << f.vertices[1].ToString()
       << " | face pt3 = " << f.vertices[2].ToString()<<"] ";
    return os;
  }

  bool visible;
  Point3D vertices[3];
};

struct Edge
{
  Edge(const Point3D& p1, const Point3D& p2): 
      adjface1(nullptr), adjface2(nullptr), remove(false) 
      { endpoints[0] = p1; endpoints[1] = p2; };
  
  void LinkAdjFace(Face* face) 
  {
    
    if( adjface1 != NULL && adjface2 != NULL )
    {
      std::cout<<"warning: property violated!\n";
      std::cout<<"associate points: "<<endpoints[0]<<" "<<endpoints[1]<<"\n";
      std::cout<<"associate face: "<<*face<<"\n";
    }
    (adjface1 == NULL ? adjface1 : adjface2)= face;
  };

  void Erase(Face* face) 
  {
    if(adjface1 != face && adjface2 != face) return;
    (adjface1 == face ? adjface1 : adjface2) = nullptr;
  };

  int Size() const 
  {
    return (adjface1 != NULL) + (adjface2 != NULL);
  }

  friend std::ostream& operator<<(std::ostream& os, const Edge& e)
  {
    os << "[edge pt1 = "   << e.endpoints[0].ToString()
       << " | edge pt2 = " << e.endpoints[1].ToString() 
       << "| " << std::to_string(e.Size()) << "] ";
    return os;
  }

  bool remove = false; 
  Face *adjface1, *adjface2;
  Point3D endpoints[2];
};

class ConvexHull
{
  public:
    template<typename T> ConvexHull(const std::vector<T>& points);

    ~ConvexHull() = default;

    template<typename T> bool Inside(T p);

    const std::vector<Face>& GetFaces() const {return this->faces;};

    void Print(const std::string mode);
    // get vertices 

  //private:

    bool Colinear(Point3D& a, Point3D& b, Point3D& c);

    bool CoPlanar(Face& f, Point3D& p);

    bool OutofFace(const Face& f, const Point3D& p) const;

    size_t Key2Edge(const Point3D& a, const Point3D& b) const;

    void AddOneFace(const Point3D& a, const Point3D& b, 
        const Point3D& c, const Point3D& inner_pt);

    void BuildFirstHull(const std::vector<Point3D>& pointcloud);

    void IncreHull(const Point3D& p);

    void ConstructHull(const std::vector<Point3D>& pointcloud);

    void CleanUp();

    Point3D FindInnerPoint(const Face* f, const Edge& e);

    int Size() const {return this->pointcloud.size();};

    std::vector<Point3D> pointcloud = {};
    std::vector<Face> faces = {};
    std::unordered_map<size_t, Edge> map_edges;
};

template<typename T> ConvexHull::ConvexHull(const std::vector<T>& points)
{
  const int n = points.size();
  this->pointcloud.resize(n);
  for(int i = 0; i < n; i++)
  { 
    this->pointcloud[i].x = points[i].x;
    this->pointcloud[i].y = points[i].y;
    this->pointcloud[i].z = points[i].z;
  }
  this->ConstructHull(this->pointcloud);
}

size_t ConvexHull::Key2Edge(const Point3D& a, const Point3D& b) const
{
  point_hash ph;
  return ph(a) ^ ph(b);
}

bool ConvexHull::OutofFace(const Face& f, const Point3D& p) const
{
  double vol;
  double ax, ay, az, bx, by, bz, cx, cy, cz;
  ax = f.vertices[0].x - p.x;  
  ay = f.vertices[0].y - p.y;  
  az = f.vertices[0].z - p.z;
  bx = f.vertices[1].x - p.x;  
  by = f.vertices[1].y - p.y;  
  bz = f.vertices[1].z - p.z;
  cx = f.vertices[2].x - p.x;  
  cy = f.vertices[2].y - p.y;  
  cz = f.vertices[2].z - p.z;
  vol = ax * (by * cz - bz * cy) +\
        ay * (bz * cx - bx * cz) +\
        az * (bx * cy - by * cx);
  return vol < 0;
}

void ConvexHull::AddOneFace(const Point3D& a, const Point3D& b, 
    const Point3D& c, const Point3D& inner_pt)
{
  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(a, b, c);
  auto& new_face = this->faces.back();
  if(this->OutofFace(new_face, inner_pt)) new_face.Reverse();

  // Create edges and link them to face_id
  auto create_edge = [&](const Point3D& p1, const Point3D& p2)
  {
    size_t key = this->Key2Edge(p1, p2);
    if(!this->map_edges.count(key)) 
    {
      this->map_edges.insert({key, Edge(p1, p2)});
    }

    this->map_edges.at(key).LinkAdjFace(&new_face);
  };
  
  create_edge(a, b);
  create_edge(a, c);
  create_edge(b, c);
}



void ConvexHull::BuildFirstHull(const std::vector<Point3D>& pointcloud)
{
  for(int i = 0; i < 4; i++)
    for(int j = i + 1; j < 4; j++)
      for(int k = j + 1; k < 4; k++) 
      {
        auto p1 = pointcloud[i];
        auto p2 = pointcloud[j];
        auto p3 = pointcloud[k];
        auto p4 = pointcloud[6-i-j-k];
        p1.processed = p2.processed = \
        p3.processed = p4.processed = true;
        this->AddOneFace(p1, p2, p3, p4);
      }
}
 
Point3D ConvexHull::FindInnerPoint(const Face* f, const Edge& e)
{
  for(int i = 0; i < 3; i++)
  {
    if(f->vertices[i] == e.endpoints[0]) continue;
    if(f->vertices[i] == e.endpoints[1]) continue;
    return f->vertices[i];
  } 
}

void ConvexHull::IncreHull(const Point3D& pt)
{
  bool vis = false;
  for(auto& f : this->faces)
  {
    if(OutofFace(f, pt)) 
    {
      f.visible = vis = true;
      std::cout<<"visible face = \n"<<f<<"\n";
      auto a = f.vertices[0];
      auto b = f.vertices[1];
      auto c = f.vertices[2];

      this->map_edges.at(this->Key2Edge(a, b)).Erase(&f);
      this->map_edges.at(this->Key2Edge(a, c)).Erase(&f);
      this->map_edges.at(this->Key2Edge(b, c)).Erase(&f);
      this->Print("edge");
    }
  }
  if(!vis) return;


  for(auto it = this->map_edges.begin(); it != this->map_edges.end(); it++)
  {
    auto& edge = it->second;
    auto face1 = edge.adjface1;
    auto face2 = edge.adjface2;

    if(face1 != NULL && face2 != NULL) 
      continue;

    if(face1 == NULL && face2 == NULL) 
    {
      edge.remove = true;
      continue;
    }

    // Only one of the adjacent face is visible: this edge
    // would be used for constructing new face
    if(face1 == NULL) std::swap(face1, face2);
    auto inner_pt = this->FindInnerPoint(face1, edge);
    this->AddOneFace(edge.endpoints[0], edge.endpoints[1], pt, inner_pt);
  }
}

void ConvexHull::ConstructHull(const std::vector<Point3D>& pointcloud)
{
  this->BuildFirstHull(pointcloud);
  std::cout<<"init\n";  this->Print("face");
  for(const auto& pt : pointcloud)
  {
    if(pt.processed) continue;
    this->IncreHull(pt);
  }
  std::cout<<"built\n";  this->Print("face");
  this->CleanUp();
  std::cout<<"cleaned\n";  this->Print("face");
}

void ConvexHull::CleanUp()
{
  for(auto it = this->map_edges.begin(); it != this->map_edges.end(); it++)
  {
    if(it->second.remove) map_edges.erase(it);
  }

  const int n = this->faces.size();
  for(size_t i = 0; i < n; i++)
  {
    if(this->faces[i].visible)
    {
      this->faces[i] = this->faces.back();
      this->faces.pop_back();
    }
  }
}

void ConvexHull::Print(const std::string mode)
{
  if(mode == "point")
    for(const auto& pt : this->pointcloud)
      std::cout<<pt<<"\n";
  else if(mode == "edge")
    for(auto it = this->map_edges.begin(); it!= this->map_edges.end(); it++)
      std::cout<<(it->second)<<"\n";
  else if( mode == "face")
    for(const auto& f : this->faces)
      std::cout<<f<<"\n";
}

#endif
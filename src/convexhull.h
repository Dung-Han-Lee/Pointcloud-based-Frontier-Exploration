#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>
#include <list>
#include "utility.h"


// Defined in CCW
struct Face
{
  Face(const Point3D& p1, const Point3D& p2, const Point3D& p3): visible(false)
      { vertices[0] = p1; vertices[1] = p2; vertices[2] = p3;};

  void Reverse(){std::swap(vertices[0], vertices[2]); };

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
      exit(1);
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
       << "| " << std::to_string(e.Size()) << " ";
       if(e.adjface1) std::cout<< e.adjface1 <<" ";
       if(e.adjface2) std::cout<< e.adjface2  <<" ] ";
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

    const std::list<Face>& GetFaces() const {return this->faces;};

    void Print(const std::string mode);
    // get vertices 

  //private:

    bool Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const;

    bool CoPlanar(Face& f, Point3D& p);

    int VolumeSign(const Face& f, const Point3D& p) const;

    size_t Key2Edge(const Point3D& a, const Point3D& b) const;

    void AddOneFace(const Point3D& a, const Point3D& b, 
        const Point3D& c, const Point3D& inner_pt);

    bool BuildFirstHull(std::vector<Point3D>& pointcloud);

    void IncreHull(const Point3D& p);

    void ConstructHull(std::vector<Point3D>& pointcloud);

    void CleanUp();

    Point3D FindInnerPoint(const Face* f, const Edge& e);

    int Size() const {return this->pointcloud.size();};

    std::vector<Point3D> pointcloud = {};
    std::list<Face> faces = {};
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

int ConvexHull::VolumeSign(const Face& f, const Point3D& p) const
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
  if(vol == 0) return 0;
  return vol < 0 ? -1 : 1;
}

void ConvexHull::AddOneFace(const Point3D& a, const Point3D& b, 
    const Point3D& c, const Point3D& inner_pt)
{
  // Make sure face is CCW with face normal pointing outward
  this->faces.emplace_back(a, b, c);
  auto& new_face = this->faces.back();
  if(this->VolumeSign(this->faces.back(), inner_pt) < 0) new_face.Reverse();

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

bool ConvexHull::Colinear(const Point3D& a, const Point3D& b, const Point3D& c) const
{
  return ((c.z - a.z) * (b.y - a.y) - 
          (b.z - a.z) * (c.y - a.y)) == 0 &&\
         ((b.z - a.z) * (c.x - a.x) - 
          (b.x - a.x) * (c.z - a.z)) == 0 &&\
         ((b.x - a.x) * (c.y - a.y) - 
          (b.y - a.y) * (c.x - a.x)) == 0;
}

bool ConvexHull::BuildFirstHull(std::vector<Point3D>& pointcloud)
{
  const int n = pointcloud.size();
  if(n <= 3)
  {
    std::cout<<"Tetrahedron: points.size() < 4\n";
    return false;    
  }

  int i = 2;
  while(this->Colinear(pointcloud[i], pointcloud[i-1], pointcloud[i-2]))
  {
    if(i++ == n - 1)
    {
      std::cout<<"Tetrahedron: All points are colinear!\n";
      return false;
    }
  }

  Face face(pointcloud[i], pointcloud[i-1], pointcloud[i-2]);

  int j = i;
  while(!this->VolumeSign(face, pointcloud[j]))
  {
    if(j++ == n-1)
    {
      std::cout<<"Tetrahedron: All pointcloud are coplanar!\n";
      return false;    
    }
  }


  auto& p1 = pointcloud[i];    auto& p2 = pointcloud[i-1];
  auto& p3 = pointcloud[i-2];  auto& p4 = pointcloud[j];
  p1.processed = p2.processed = p3.processed = p4.processed = true;
  this->AddOneFace(p1, p2, p3, p4);
  this->AddOneFace(p1, p2, p4, p3);
  this->AddOneFace(p1, p3, p4, p2);
  this->AddOneFace(p2, p3, p4, p1);
  return true;
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
  //for(auto it = this->faces.begin(); it!= this->faces.end(); it++)
  //  std::cout<<&(*it)<<"\n";

  bool vis = false;
  for(auto it = this->faces.begin(); it!= this->faces.end(); it++)
  {
    auto& face = *it;
    if(VolumeSign(face, pt) < 0) 
    {
      face.visible = vis = true;
      std::cout<<"visible face by point "<<pt<<"\n"<<face<<"\n";
      auto a = face.vertices[0];
      auto b = face.vertices[1];
      auto c = face.vertices[2];

      this->map_edges.at(this->Key2Edge(a, b)).Erase(&face);
      this->map_edges.at(this->Key2Edge(a, c)).Erase(&face);
      this->map_edges.at(this->Key2Edge(b, c)).Erase(&face);
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

    if(face1 != NULL && face2 != NULL && ) 
    {
      edge.remove = true;
      continue;
    }

    this->Print("edge");
    std::cout<<"extending edge: "<<edge<<"\n";
    // Only one of the adjacent face is visible: this edge
    // would be used for constructing new face
    if(face1 == NULL) std::swap(face1, face2);
    auto inner_pt = this->FindInnerPoint(face1, edge);
    this->AddOneFace(edge.endpoints[0], edge.endpoints[1], pt, inner_pt);
  }
}

void ConvexHull::ConstructHull(std::vector<Point3D>& pointcloud)
{
  if(!this->BuildFirstHull(pointcloud)) return;
  std::cout<<"init\n";  this->Print("face"); this->Print("edge");
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

  auto it = this->faces.begin();
  while(it != this->faces.end())
  {
    if(it->visible) this->faces.erase(it++);
    else it++;
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
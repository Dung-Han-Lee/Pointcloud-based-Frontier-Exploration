#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

struct Vec3
{
  Vec3() = default;

  Vec3(double x, double y, double z): x(x), y(y), z(z) {};
  
  std::string ToString() const
  {
    return std::to_string(x).substr(0,4) + ", " + 
           std::to_string(y).substr(0,4) + ", " + 
           std::to_string(z).substr(0,4);
  };
  
  friend std::ostream& operator<<(std::ostream& os, const Vec3& v)
  {
    os << "["<< v.ToString() << "] ";
    return os;
  }

  double x, y, z;
};

// Defined in CCW
struct Face
{
  Face(const Vec3& p1, const Vec3& p2, const Vec3& p3): a(p1), b(p2), c(p3){};

  void Reverse(){std::swap(a, c);};

  friend std::ostream& operator<<(std::ostream& os, const Face& f)
  {
    os << "[face pt1 = " << f.a.ToString()
       << " | face pt2 = " << f.b.ToString()
       << " | face pt3 = " << f.c.ToString()<<"] ";
    return os;
  }

  Vec3 a, b, c;
};

struct Edge
{
  Edge(const Vec3& p1, const Vec3& p2):\
      a(p1), b(p2), id1(-1), id2(-1) {};
  
  void LinkAdjFace(int x) 
  {
    if( id1 != -1 && id2 != -1 ) std::cout<<"warning: property violated!\n";
    (id1 == -1 ? id1 : id2) = x;
  };

  void Erase(int x) 
  {
    if(id1 != x && id2 != x) return;
    (id1 == x ? id1 : id2) = -1;
  };

  int Size() { return (id1 != -1) + (id2 != -1); }
  
  std::string ids () const
  {
    return std::to_string(id1) + " " + std::to_string(id2);
  }

  friend std::ostream& operator<<(std::ostream& os, const Edge& e)
  {
    os << "[edge pt1 = " << e.a.ToString()
       << " | edge pt2 = " << e.b.ToString() 
       << " | face ids = " << e.ids() << "] ";
    return os;
  }

  int id1, id2; //index of adjacent face id
  Vec3 a, b;
};

class ConvexHull
{
  public:
    template<typename T> ConvexHull(const std::vector<T>& points);

    ~ConvexHull() = default;

    template<typename T> bool Inside(T p);

    const std::vector<Face>& GetFaces() const {return this->faces;};

  private:

    bool Colinear(Vec3& a, Vec3& b, Vec3& c);

    bool CoPlanar(Face& f, Vec3& p);

    bool OutofFace(const Face& f, const Vec3& p) const;

    size_t Key2Edge(const Vec3& a, const Vec3& b) const;

    Face MakeOneFace(const Vec3& a, const Vec3& b, 
        const Vec3& c, const Vec3& inner_pt);

    void BuildFirstHull();

    int Size() const {return this->vertices.size();};

    std::vector<Vec3> vertices = {};
    std::vector<Face> faces = {};
    std::unordered_map<size_t, Edge> map_edges;
};

template<typename T> ConvexHull::ConvexHull(const std::vector<T>& points)
{
  const int n = points.size();
  this->vertices.resize(n);
  for(int i = 0; i < n; i++)
  { 
    this->vertices[i].x = points[i].x;
    this->vertices[i].y = points[i].y;
    this->vertices[i].z = points[i].z;
  }
  this->BuildFirstHull();
}

#endif
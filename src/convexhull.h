#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include <vector>
#include <string>
#include <iostream>
#include <unordered_map>

struct Vec3
{
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
  
  friend std::ostream& operator<<(std::ostream& os, const Edge& e)
  {
    os << "[edge pt1 = " << e.a.ToString()
       << " | edge pt2 = " << e.b.ToString() <<"] ";
    return os;
  }

  int id1, id2; //index of adjacent face id
  Vec3 a, b;
};

size_t Key2Edge(Vec3& a, Vec3& b)
{
  size_t hash_a = std::hash<std::string>{}(a.ToString());
  size_t hash_b = std::hash<std::string>{}(b.ToString());
  return hash_a & hash_b;
}

#endif
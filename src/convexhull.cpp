#include "convexhull.h"

size_t ConvexHull::Key2Edge(Vec3& a, Vec3& b) const
{
  size_t hash_a = std::hash<std::string>{}(a.ToString());
  size_t hash_b = std::hash<std::string>{}(b.ToString());
  return hash_a & hash_b;
}

bool ConvexHull::OutofFace(const Face& f, const Vec3& p) const
{
  double vol;
  double ax, ay, az, bx, by, bz, cx, cy, cz;
  ax = f.a.x - p.x;  ay = f.a.y - p.y;  az = f.a.z - p.z;
  bx = f.b.x - p.x;  by = f.b.y - p.y;  bz = f.b.z - p.z;
  cx = f.c.x - p.x;  cy = f.c.y - p.y;  cz = f.c.z - p.z;
  vol = ax * (by * cz - bz * cy) +\
        ay * (bz * cx - bx * cz) +\
        az * (bx * cy - by * cx);
  return vol < 0;
}
#include "convexhull.h"

size_t ConvexHull::Key2Edge(const Vec3& a, const Vec3& b) const
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

Face ConvexHull::MakeOneFace(const Vec3& a, const Vec3& b, 
    const Vec3& c, const Vec3& inner_pt)
{
  // Use index of faces as their id
  int face_id = this->faces.size();

  // Create edges and link them to face_id
  auto create_edge = [&](const Vec3& p1, const Vec3& p2)
  {
    size_t key = Key2Edge(p1, p2);
    if(!map_edges.count(key)) 
    {
      map_edges.insert({key, Edge(p1, p2)});
    }
    map_edges.at(key).LinkAdjFace(face_id);
  };
  create_edge(a, b);
  create_edge(a, c);
  create_edge(b, c);

  // Make sure face is CCW with face normal pointing outward
  Face face(a, b, c);
  if(this->OutofFace(face, inner_pt)) face.Reverse();
  return face;
}

void ConvexHull::BuildFirstHull()
{
  for(int i = 0; i < 4; i++)
    for(int j = i + 1; j < 4; j++)
      for(int k = j + 1; k < 4; k++) 
      {
        auto p1 = this->vertices[i];
        auto p2 = this->vertices[j];
        auto p3 = this->vertices[k];
        auto p4 = this->vertices[6-i-j-k];
        faces.push_back(this->MakeOneFace(p1, p2, p3, p4));
      }
}
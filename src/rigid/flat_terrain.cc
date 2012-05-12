#include "flat_terrain.hh"
#include "../common/segment.hh"

FlatTerrain::FlatTerrain(uint nx, uint ny, double dx, double dy, double h, double ela, double st_fric, double dy_fric):
    Terrain(ela, st_fric, dy_fric), nx_(nx), ny_(ny), dx_(dx), dy_(dy), height_(h){}

double FlatTerrain::getHeight(double x, double y) const {
  return height_;
}

vec3d FlatTerrain::getNormal(double x, double y) const {
  return vec3d(0.0, 0.0, 1.0);
}

bool FlatTerrain::getSegmentCollision(const Segment& seg, double& coll_frac, coord& coll_pos, vec3d& coll_norm) const{
  //!FIXME: there is a bug
  if(seg.start[2]<0.0 || seg.dir[2] > -0.0001) return false;
  if(seg.start[2] + seg.dir[2] > 0.0) return false;

  coll_frac = seg.start[2] / -seg.dir[2];
  coll_pos = seg.start + coll_frac * seg.dir;
  coll_norm = vec3d(0, 0, 1.0);
  return true;
}

void FlatTerrain::draw() const {
}

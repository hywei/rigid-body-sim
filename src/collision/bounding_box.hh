#ifndef RIGID_SIM_BOUNDING_BOX_H_
#define RIGID_SIM_BOUNDING_BOX_H_

#include "../common/types.hh"

struct BoundingBox{
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  double dx, dy, dz;
  double radius; // radius of bounding sphere
};

// get sqr distance from a point to bounding box
double getSqrDist2BB(coord p, const BoundingBox& b){
  coord cp;
  cp[0] = p[0] > b.max_x ? b.max_x : (p[0] < b.min_x ? b.min_x : p[0]);
  cp[1] = p[1] > b.max_y ? b.max_y : (p[1] < b.min_y ? b.min_y : p[1]);
  cp[2] = p[2] > b.max_z ? b.max_z : (p[1] < b.min_z ? b.min_z : p[2]);
  return (p-cp).mag2();
}

#endif

#ifndef SEGMENT_HPP
#define SEGMENT_HPP

#include "types.hh"

struct Segment{
  Segment(const coord & start, const vec3d & dir) : start(start), dir(dir) {}
  coord start;
  vec3d dir; // not normalised
};

#endif

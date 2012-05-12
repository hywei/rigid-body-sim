#ifndef COLL_DETECT_HPP
#define COLL_DETECT_HPP

#include "collision_body.hh"

#include "types.hh"
#include <vector>

class CollBody;
class SimHeightMap;

struct CollInfo{
  CollInfo() {}
  CollInfo(CollBody* body0, CollBody* body1, SimHeightMap* heightmap,
           vec3d dir_to_0, coord pos, double penetration_depth):
      body0(body0), body1(body1), heightmap(heightmap),
      dir_to_0(dir_to_0),
      position(pos), penetration_depth(penetration_depth) {}

  // body 0 is alsways set
  CollBody* body0;
  // body 1 may be zero, if it's a body-terrain collision
  CollBody* body1;
  // terrain may be zero, if it's a body-body collision
  SimHeightMap* heightmap;

  // normalised direction in world space pointing towards body 0 from the other body/terrain
  vec3d dir_to_0;

  // collision point, in world space (i.e. sensible place to apply a restoring force)
  coord position;
  // depth of the penetration. Can be -ve, indicating penetration isn't happening
  double penetration_depth;

  /// Set/Used by Physics in process/preprocess_collision
  coord R0; // position relative to body 0 (in world space)
  coord R1; // position relative to body 1 (if there is a body1)
  double vr_extra; // extra speed (in dir_to_0) for restoring deviation
  double elasticity;
  double static_friction;
  double dynamic_friction;
  double denominator;
};

/// detects all collisions between active bodies and other active/frozen bodies and heightmaps. 
/// Appends the result to collisions.
/// brute force approach for now ;-)
void detect_all_collisions(const std::vector<CollBody*>& collision_bodies,
                           SimHeightMap* heightmaps,
                           std::vector<CollInfo> & collisions);

/// detects all collisions between a single bodies and other frozen/immovable bodies and heightmaps. 
/// Appends the result to collisions.
/// brute force approach for now ;-)
/// collision_body should not be in collision_bodies
void detect_frozen_collisions_with_body(CollBody * collision_body,
                                        const std::vector<CollBody *> & collision_bodies,
                                        const std::vector<SimHeightMap *>  & heightmaps,
                                        std::vector<CollInfo> & collisions);

#endif

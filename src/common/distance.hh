#ifndef DISTANCE_HPP
#define DISTANCE_HPP

#include "types.hh"


// The fundamental distance functions (sqr and normal versions)
struct Triangle{
  Triangle(const coord & origin, const vec3d & edge0, const vec3d & edge1) 
    : origin(origin), edge0(edge0), edge1(edge1) {}
  coord origin;
  vec3d edge0;
  vec3d edge1;
};

struct Ellipsoid_shape{
  Ellipsoid_shape(const vec3d & extents) : extents(extents) {}
  vec3d extents;
};

double distance_sqr(const coord & point,
                    const Triangle & triangle,
                    double * SParam,
                    double * TParam);

double distance(const coord & point,
                const Triangle & triangle,
                double * SParam,
                double * TParam);

double distance_sqr(vec3d rkPoint, 
                    const Ellipsoid_shape & rkEllipsoid,
                    vec3d & rkClosest);




// returns the squared distance, and calculates the closest point
// on the triangle
double distance_sqr_point_triangle(const coord & point,
                                   const coord & tri_pos0,
                                   const coord & tri_pos1,
                                   const coord & tri_pos2,
                                   coord & point_on_triangle,
                                   bool & point_on_triangle_edge);

#endif

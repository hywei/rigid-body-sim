#include "intersection.hh"

bool intersect_segment_triangle(const Segment & seg, const Triangle & tri,
                                coord & pos, 
                                double & SegP,
                                bool & seg_in_dir_of_tri_normal){
  // Calculate the intersection point of the segment and the triangle plane.
  // Then check if the point is within the 3 triangle edge planes.

  vec3d tri_normal = cross(tri.edge0, tri.edge1);
  tri_normal.normalise();
  double tri_dist = -dot(tri_normal, tri.origin);

  double perp_dist_seg0 = dot(seg.start, tri_normal) + tri_dist;
  double perp_dist_seg1 = dot(seg.start + seg.dir, tri_normal) + tri_dist;

  if (perp_dist_seg0 * perp_dist_seg1 >= 0.0f)
    return false; // both on same side.

  SegP = (perp_dist_seg0 / (perp_dist_seg0 - perp_dist_seg1));

  pos = seg.start + SegP * seg.dir;

  // check each triangle edge plane in turn. Each plane normal points out.
  // edge 0
  vec3d plane_normal = cross(tri.edge0, tri_normal).normalise();
  double plane_dist = -dot(plane_normal, tri.origin);

  double point_dist = dot(pos, plane_normal) + plane_dist;
  if (point_dist > 0.0f)
    return false;

  // edge 1
  plane_normal = cross(tri_normal, tri.edge1).normalise();
  plane_dist = -dot(plane_normal, tri.origin);

  point_dist = dot(pos, plane_normal) + plane_dist;
  if (point_dist > 0.0f)
    return false;

  // the other edge - from the end of edge0 to the end of edge 1
  vec3d edge = tri.edge1 - tri.edge0;
  plane_normal = cross(edge, tri_normal).normalise();
  plane_dist = -dot(plane_normal, tri.origin + tri.edge0);

  point_dist = dot(pos, plane_normal) + plane_dist;
  if (point_dist > 0.0f)
    return false;

  // point is in triangle! just set the direction flag
  seg_in_dir_of_tri_normal = (perp_dist_seg0 < 0.0f);

  return true;
}

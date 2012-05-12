#ifndef INTERSECTION_HPP
#define INTERSECTION_HPP

// use distance.hh definitions of a triangle (etc)
#include "distance.hh"
#include "segment.hh"

/// Indicates if a segment intersects a triangle. The intersection parameters
/// are also calculated. If no intersection occurs, these MAY BE MODIFIED.
/// Not very efficient, since various things need to be calculated from scratch.
bool intersect_segment_triangle(const Segment & seg, const Triangle & tri,
                                coord & pos, 
                                double & SegP,
                                bool & seg_in_dir_of_tri_normal); // "hit" from behind the face"

#endif

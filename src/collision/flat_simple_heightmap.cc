#include "flat_simple_heightmap.hh"

Flat_simple_heightmap::Flat_simple_heightmap(double dx, double dy, unsigned nx, unsigned ny) :
SimpleHeightMap(m_heightmap),
m_heightmap(nx, ny, 0.0f),
m_dx(dx), m_dy(dy),
m_x0(-0.5f * (nx-1) * dx ), m_y0(-0.5f * (ny-1) * dy)
{
  TRACE_METHOD_ONLY(ONCE_1);
}

// position and normal are returned if there is a collision
bool Flat_simple_heightmap::get_segment_collision(const Segment & segment, double & frac, coord & position, vec3d & normal) const
{
  if (segment.start[2] < 0.0)
    return false;

  if (segment.dir[2] > -0.0001)
    return false;

  if (segment.start[2] + segment.dir[2] > 0.0)
    return false;
  
  frac = segment.start[2] / -segment.dir[2];

  position = segment.start + frac * segment.dir;
  normal = vec3d(0.0, 0.0, 1.0);
  return true;
}

void Flat_simple_heightmap::get_z_and_normal(double x, double y, 
                                             double & z, 
                                             vec3d & normal) const
{
  normal[0] = 0.0f;
  normal[1] = 0.0f;
  normal[2] = 1.0f;
  z = 0.0f;
}


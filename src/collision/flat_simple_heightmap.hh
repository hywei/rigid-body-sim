#ifndef FLAT_SIMPLE_HEIGHTMAP_HPP
#define FLAT_SIMPLE_HEIGHTMAP_HPP

#include "simple_heightmap.hh"
#include "array_2d.hh"

class Flat_simple_heightmap : public SimpleHeightMap
{
public:
  Flat_simple_heightmap(double dx, double dy, unsigned nx, unsigned ny);

  // From SimpleHeightMap
  void get_dx_dy(double & dx, double & dy) const {dx = m_dx; dy = m_dy;}
  void get_nx_ny(int & nx, int & ny) const {nx = m_heightmap.get_nx(); ny = m_heightmap.get_ny();}
  void get_x0_y0(double & x0, double & y0) const {x0 = m_x0; y0 = m_y0;}

  // position and normal are returned if there is a collision
  bool get_segment_collision(const Segment & segment, double & frac, 
                             coord & position, vec3d & normal) const;

  void get_z_and_normal(double x, double y, double & z, vec3d & normal) const;

private:
  Array_2D<double> m_heightmap;
  double m_dx, m_dy;
  double m_x0, m_y0;
};

#endif

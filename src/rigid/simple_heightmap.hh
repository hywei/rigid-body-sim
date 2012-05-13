#ifndef SIMPLE_HEIGHTMAP_HPP
#define SIMPLE_HEIGHTMAP_HPP

#include "sim_heightmap.hh"
#include "array_2d.hh"
#include "../render/glut_utils.hh"

class SimpleHeightMap : public SimHeightMap{
public:
  SimpleHeightMap(Array_2D<double>& heightmap);
  void draw();

  // Implemented by the derived class
  virtual void get_dx_dy(double& dx, double& dy) const = 0;
  virtual void get_nx_ny(int& nx, int& ny) const = 0;
  virtual void get_x0_y0(double& x0, double& y0) const = 0;

  // from SimHeightMap
  double get_z(int i, int j) const;
  void get_z_and_normal(double x, double y, double & z, 
                        vec3d & normal) const;

  // position and normal are returned if there is a collision
  bool get_segment_collision(const Segment & segment, double & frac, 
                             coord & position, vec3d & normal) const;

private:
  Array_2D<double>& m_heightmap;
  GLuint m_display_list_num;
};

#endif
